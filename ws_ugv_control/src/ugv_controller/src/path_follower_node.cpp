/* ws_ugv_control/src/ugv_controller/src/path_follower_node.cpp */

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <filesystem>

using std::placeholders::_1;
using namespace std::chrono_literals;

struct Waypoint { double x, y; int mission; };

class PathFollower : public rclcpp::Node
{
public:
  PathFollower()
  : Node("path_follower_tf_only"),
    current_x_(0.0), current_y_(0.0), current_yaw_(0.0),
    pose_received_(false), mode_two_(false), start_sent_(false),
    max_speed_1_(6.0),    // mission1 최대 속도 8.0 m/s
    max_speed_3_(6.0),    // mission3 최대 속도 6.0 m/s
    lookahead_radius_(3.0), axle_offset_(0.2559),
    slow_radius_(10.0), stop_radius_(0.5),
    accel_(0.8), decel_(1.5), current_speed_(0.0),
    max_gap_(1.0)         // ← 두 웨이포인트 간 허용 최대 거리 [m]
  {
    /* ---------- 파라미터 선언 및 읽기 ---------- */
    declare_parameter("max_gap", max_gap_);
    get_parameter("max_gap", max_gap_);

    /* ---------- ① 경로 로드 & 밀도화 ---------- */
    load_path();      // CSV 읽기
    densify_path();   // mission3→4 구간 보간

    /* ---------- ② 퍼블리셔 ---------- */
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/ugv/global_path", rclcpp::QoS{1}.transient_local());

    path_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/ugv/global_path_markers", rclcpp::QoS{1}.transient_local());

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "/ugv/lookahead_marker", 10);

    /* Path & MarkerArray 1초마다 재발행 */
    path_timer_ = create_wall_timer(
        1s, std::bind(&PathFollower::publish_path_and_markers, this));

    /* ---------- ③ TF 구독 ---------- */
    tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
        "/model/X1_asp/pose_static", 10,
        std::bind(&PathFollower::tf_callback, this, _1));

    /* ---------- ④ cmd_vel ---------- */
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        "/model/X1_asp/cmd_vel", 10);

    /* ---------- ⑤ UAV/UGV 피드백 ---------- */
    feedback_pub_ = create_publisher<std_msgs::msg::Int32>("one_feedback", 10);

    two_fb_sub_ = create_subscription<std_msgs::msg::Int32>(
        "two_feedback", 10,
        std::bind(&PathFollower::two_feedback_callback, this, _1));

    /* ---------- ⑥ 제어 루프 ---------- */
    timer_ = create_wall_timer(
        100ms, std::bind(&PathFollower::control_loop, this));
  }

private:
  /* ---------- 경로 파일 읽기 ---------- */
  void load_path()
  {
    namespace fs = std::filesystem;
    fs::path csv_path =
        fs::path(ament_index_cpp::get_package_share_directory("ugv_controller"))
        / "data" / "ugv_waypoint.csv";

    std::ifstream ifs(csv_path);
    if (!ifs.is_open()) {
      RCLCPP_ERROR(get_logger(), "Cannot open CSV: %s", csv_path.c_str());
      return;
    }
    std::string line;
    while (std::getline(ifs, line)) {
      if (line.empty()) continue;
      std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream ss(line);
      double x, y; int m;
      if (ss >> x >> y >> m) path_.push_back({x, y, m});
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu original waypoints", path_.size());
  }

  /* ---------- mission3 → 4 구간 자동 보간 ---------- */
  void densify_path()
  {
    if (path_.size() < 2) return;

    std::vector<Waypoint> dense;
    dense.reserve(path_.size() * 2);   // 대략 두 배 여유
    dense.push_back(path_.front());

    for (size_t i = 1; i < path_.size(); ++i) {
      const auto &p0 = path_[i - 1];
      const auto &p1 = path_[i];

      // mission 3 → 4 구간만 보간
      bool do_densify = (p0.mission == 3 && p1.mission == 4);

      if (!do_densify) {
        dense.push_back(p1);
        continue;
      }

      double dx = p1.x - p0.x;
      double dy = p1.y - p0.y;
      double dist = std::hypot(dx, dy);
      int n_seg = static_cast<int>(std::ceil(dist / max_gap_));

      if (n_seg > 1) {
        for (int k = 1; k < n_seg; ++k) {
          double t = static_cast<double>(k) / n_seg;
          Waypoint mid{
            p0.x + t * dx,
            p0.y + t * dy,
            3                       // 중간 점은 mission 3
          };
          dense.push_back(mid);
        }
      }
      dense.push_back(p1);
    }

    path_.swap(dense);
    RCLCPP_INFO(get_logger(), "Path densified → %zu waypoints (gap ≤ %.2f m)",
                path_.size(), max_gap_);
  }

  /* ---------- Path + MarkerArray ---------- */
  void publish_path_and_markers()
  {
    if (path_.empty()) return;

    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp    = now();

    visualization_msgs::msg::MarkerArray marr;
    auto header = path_msg.header;

    visualization_msgs::msg::Marker line_m1, line_m3;
    line_m1.header = line_m3.header = header;
    line_m1.ns = line_m3.ns = "global_path";
    line_m1.type = line_m3.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_m1.action = line_m3.action = visualization_msgs::msg::Marker::ADD;
    line_m1.id = 1; line_m3.id = 3;
    line_m1.scale.x = line_m3.scale.x = 0.08;

    line_m1.color.r = 0.0; line_m1.color.g = 0.0; line_m1.color.b = 1.0; line_m1.color.a = 1.0;
    line_m3.color.r = 1.0; line_m3.color.g = 1.0; line_m3.color.b = 0.0; line_m3.color.a = 1.0;

    visualization_msgs::msg::Marker sphere_m2, text_m2;
    bool m2_found = false;

    visualization_msgs::msg::Marker sphere_end, text_end;
    const Waypoint final_wp = path_.back();

    for (auto &wp : path_) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = header;
      ps.pose.position.x = wp.x;
      ps.pose.position.y = wp.y;
      ps.pose.orientation.w = 1.0;
      path_msg.poses.emplace_back(ps);

      geometry_msgs::msg::Point p; p.x = wp.x; p.y = wp.y; p.z = 0.05;

      if (wp.mission == 1) line_m1.points.emplace_back(p);
      else if (wp.mission == 3) line_m3.points.emplace_back(p);
      else if (wp.mission == 2 && !m2_found) {
        sphere_m2.header = header; sphere_m2.ns = "global_path"; sphere_m2.id = 2;
        sphere_m2.type = visualization_msgs::msg::Marker::SPHERE;
        sphere_m2.action = visualization_msgs::msg::Marker::ADD;
        sphere_m2.pose.position = p; sphere_m2.pose.position.z = 0.2;
        sphere_m2.scale.x = sphere_m2.scale.y = sphere_m2.scale.z = 0.5;
        sphere_m2.color.r = 1.0; sphere_m2.color.g = 0.0; sphere_m2.color.b = 0.0; sphere_m2.color.a = 1.0;

        text_m2 = sphere_m2; text_m2.id = 4;
        text_m2.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_m2.pose.position.z = 0.8; text_m2.scale.z = 0.4;
        text_m2.color.r = text_m2.color.g = text_m2.color.b = 1.0;
        text_m2.text = "Offboard Point";
        m2_found = true;
      }
    }

    sphere_end.header = header; sphere_end.ns = "global_path"; sphere_end.id = 5;
    sphere_end.type = visualization_msgs::msg::Marker::SPHERE; sphere_end.action = visualization_msgs::msg::Marker::ADD;
    sphere_end.pose.position.x = final_wp.x; sphere_end.pose.position.y = final_wp.y; sphere_end.pose.position.z = 0.2;
    sphere_end.scale.x = sphere_end.scale.y = sphere_end.scale.z = 0.5;
    sphere_end.color.r = 0.0; sphere_end.color.g = 1.0; sphere_end.color.b = 0.0; sphere_end.color.a = 1.0;

    text_end = sphere_end; text_end.id = 6;
    text_end.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_end.pose.position.z = 0.8; text_end.scale.z = 0.4;
    text_end.color.r = text_end.color.g = text_end.color.b = 1.0;
    text_end.text = "Rendezvous";

    marr.markers.emplace_back(line_m1);
    marr.markers.emplace_back(line_m3);
    if (m2_found) { marr.markers.emplace_back(sphere_m2); marr.markers.emplace_back(text_m2); }
    marr.markers.emplace_back(sphere_end); marr.markers.emplace_back(text_end);

    path_pub_->publish(path_msg);
    path_marker_pub_->publish(marr);
  }

  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    for (auto &t : msg->transforms) {
      if (t.child_frame_id == "X1_asp") {
        current_x_ = t.transform.translation.x;
        current_y_ = t.transform.translation.y;
        auto &q = t.transform.rotation;
        current_yaw_ = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z));
        pose_received_ = true;
        break;
      }
    }
  }

  void two_feedback_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (msg->data == 1 && !mode_two_) {
      mode_two_ = true;
      RCLCPP_INFO(get_logger(), "[UGV] two_feedback=1 수신 → mission3 진행");
    }
  }

  void control_loop()
  {
    if (!pose_received_) return;

    const double rx = current_x_ - axle_offset_ * std::cos(current_yaw_);
    const double ry = current_y_ - axle_offset_ * std::sin(current_yaw_);

    const int first = mode_two_ ? 3 : 1;
    std::vector<Waypoint> active;
    for (auto &wp : path_)
      if (wp.mission == first || wp.mission == first + 1)
        active.push_back(wp);
    if (active.empty()) return;

    const double lx = rx + lookahead_radius_ * std::cos(current_yaw_);
    const double ly = ry + lookahead_radius_ * std::sin(current_yaw_);
    double best_d2 = 1e9;
    Waypoint target{0,0,0};
    for (auto &wp : active) {
      double d2 = std::pow(wp.x - lx, 2) + std::pow(wp.y - ly, 2);
      if (d2 < best_d2) { best_d2 = d2; target = wp; }
    }

    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = "map"; mk.header.stamp = now();
    mk.ns = "lookahead"; mk.id = 0;
    mk.type = visualization_msgs::msg::Marker::SPHERE;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.pose.position.x = target.x;
    mk.pose.position.y = target.y;
    mk.pose.position.z = 0.3;
    mk.scale.x = mk.scale.y = mk.scale.z = 0.35;
    mk.color.r = 1.0; mk.color.g = 0.5; mk.color.b = 0.0; mk.color.a = 1.0;
    marker_pub_->publish(mk);

    double dx = target.x - rx, dy = target.y - ry;
    double alpha = std::atan2(dy, dx) - current_yaw_;
    alpha = std::atan2(std::sin(alpha), std::cos(alpha));
    double curvature = 2 * std::sin(alpha) / lookahead_radius_;

    int stop_m = first + 1;
    double best_stop = 1e9;
    for (auto &wp : active) {
      if (wp.mission == stop_m) {
        double d2 = std::pow(wp.x - rx, 2) + std::pow(wp.y - ry, 2);
        if (d2 < best_stop) best_stop = d2;
      }
    }
    double d_stop = std::sqrt(best_stop);

    if (!start_sent_ && d_stop < stop_radius_) {
      std_msgs::msg::Int32 flag;
      flag.data = 1;
      feedback_pub_->publish(flag);
      start_sent_ = true;
      RCLCPP_INFO(get_logger(), "[UGV] one_feedback=1 발행");
    }

    // mission1/mission3 구간별 최대 속도 적용
    double base_max = (first == 1) ? max_speed_1_ : max_speed_3_;
    double local_max = (std::hypot(rx + 77.14, ry - 92.14) < 15.0)
                       ? 3.0 
                       : base_max;

    bool waiting = (!mode_two_) && start_sent_;
    double desired = waiting ? 0.0
                     : (d_stop < stop_radius_ ? 0.0
                       : (d_stop < slow_radius_ ? local_max * (d_stop / slow_radius_)
                                                : local_max));

    double dt = 0.1;
    if (current_speed_ < desired)
      current_speed_ = std::min(desired, current_speed_ + accel_ * dt);
    else
      current_speed_ = std::max(desired, current_speed_ - decel_ * dt);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = current_speed_;
    cmd.angular.z = current_speed_ * curvature;
    cmd_pub_->publish(cmd);
  }

  // 멤버 변수
  double current_x_, current_y_, current_yaw_;
  bool pose_received_, mode_two_, start_sent_;
  double max_speed_1_, max_speed_3_;
  double lookahead_radius_, axle_offset_;
  double slow_radius_, stop_radius_, accel_, decel_, current_speed_;
  double max_gap_;                  // ← 추가: 보간 간격 파라미터
  std::vector<Waypoint> path_;

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr two_fb_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr feedback_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_, path_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollower>());
  rclcpp::shutdown();
  return 0;
}
