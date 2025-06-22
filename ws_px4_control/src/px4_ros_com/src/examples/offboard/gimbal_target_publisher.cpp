// gimbal_target_publisher.cpp (Build Fix Final Version)

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <optional>
#include <chrono>

using namespace std::chrono_literals;

struct Marker { std::string name; double e,n,u; };

class GimbalAngleController : public rclcpp::Node {
public:
  GimbalAngleController() : Node("gimbal_target_publisher"), tf_buf_(this->get_clock()), tf_lis_(tf_buf_) {
    declare_parameter("map_frame", "map");
    declare_parameter("base_frame", "x500_gimbal_0");
    declare_parameter("camera_frame", "x500_gimbal_0/camera_link");
    declare_parameter("marker_csv", "");

    map_frame_ = get_parameter("map_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    cam_frame_ = get_parameter("camera_frame").as_string();

    std::string csv = get_parameter("marker_csv").as_string();
    if (csv.empty())
      csv = ament_index_cpp::get_package_share_directory("px4_ros_com") + "/data/aruco_markers.csv";
    if (!loadCSV(csv)) {
        RCLCPP_FATAL(get_logger(), "Failed to load marker CSV file from %s", csv.c_str());
        throw std::runtime_error("Marker CSV loading failed");
    }

    auto vis_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    arrow_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("gimbal_arrow", vis_qos);

    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/gimbal_mode", 10,
        [this](const std_msgs::msg::String::SharedPtr msg){
            if (this->current_gimbal_mode_ != msg->data) {
                RCLCPP_INFO(this->get_logger(), "Gimbal mode changed to: %s", msg->data.c_str());
            }
            this->current_gimbal_mode_ = msg->data;
        });

    timer_ = create_wall_timer(50ms, [this]{ onTimer(); });

    RCLCPP_INFO(get_logger(), "Gimbal Direct Angle Controller started.");
  }

private:
  bool loadCSV(const std::string& path) {
    std::ifstream fs(path);
    if (!fs.is_open()) return false;
    std::string line; bool first=true;
    while (std::getline(fs, line)) {
      if (first) { first = false; continue; }
      std::stringstream ss(line); std::string tok; std::vector<std::string> col;
      while (std::getline(ss, tok, ',')) col.push_back(tok);
      if (col.size() < 4) continue;
      markers_.push_back({col[0], std::stod(col[1]), std::stod(col[2]), std::stod(col[3])});
    }
    return !markers_.empty();
  }

  uint64_t usec() { return get_clock()->now().nanoseconds() / 1000ULL; }

  void onTimer() {
    if (current_gimbal_mode_ == "LOOK_DOWN") {
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.timestamp = usec();
        cmd.target_system = 1; cmd.target_component = 1;
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_MOUNT_CONTROL;
        cmd.param1 = -90.0f;
        cmd.param3 = 0.0f;
        cmd.param7 = 2.0f;
        cmd_pub_->publish(cmd);
        return;
    }

    if (!tf_buf_.canTransform(map_frame_, base_frame_, tf2::TimePointZero) ||
        !tf_buf_.canTransform(map_frame_, cam_frame_, tf2::TimePointZero)) {
      RCLCPP_INFO_ONCE(get_logger(), "Waiting for required transforms to become available...");
      return;
    }
    
    geometry_msgs::msg::TransformStamped tf_drone, tf_cam;
    try {
      tf_drone = tf_buf_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      tf_cam = tf_buf_.lookupTransform(map_frame_, cam_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(get_logger(), "Could not transform: %s", ex.what());
      return;
    }

    const double cam_e = tf_cam.transform.translation.x;
    const double cam_n = tf_cam.transform.translation.y;
    const double cam_u = tf_cam.transform.translation.z;

    const Marker* best_marker = nullptr; double best_d2 = std::numeric_limits<double>::max();
    for (const auto& m : markers_) {
      double dE = m.e - cam_e, dN = m.n - cam_n, dU = m.u - cam_u;
      double d2 = dE*dE + dN*dN + dU*dU;
      if (d2 < best_d2) { best_d2 = d2; best_marker = &m; }
    }
    if (!best_marker) return;

    tf2::Quaternion q_drone;
    tf2::fromMsg(tf_drone.transform.rotation, q_drone);
    double drone_roll, drone_pitch, drone_yaw;
    tf2::Matrix3x3(q_drone).getRPY(drone_roll, drone_pitch, drone_yaw);

    const double drone_e = tf_drone.transform.translation.x;
    const double drone_n = tf_drone.transform.translation.y;
    const double vec_e = best_marker->e - drone_e;
    const double vec_n = best_marker->n - drone_n;
    const double vec_u = best_marker->u - cam_u;

    const double horizontal_dist = std::hypot(vec_e, vec_n);
    double target_pitch_rad = std::atan2(vec_u, horizontal_dist); 

    double target_world_yaw_rad = std::atan2(vec_n, vec_e);
    double target_body_yaw_rad = target_world_yaw_rad - drone_yaw;
    
    while (target_body_yaw_rad > M_PI) target_body_yaw_rad -= 2.0 * M_PI;
    while (target_body_yaw_rad < -M_PI) target_body_yaw_rad += 2.0 * M_PI;

    double gimbal_pitch_deg = target_pitch_rad * 180.0 / M_PI;
    double gimbal_yaw_deg = -target_body_yaw_rad * 180.0 / M_PI;

    px4_msgs::msg::VehicleCommand cmd{};
    cmd.timestamp = usec();
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_MOUNT_CONTROL;
    cmd.param1 = gimbal_pitch_deg;
    cmd.param2 = 0.0;
    cmd.param3 = gimbal_yaw_deg;
    cmd.param7 = 2.0;
    cmd_pub_->publish(cmd);

    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = map_frame_;
    arrow.header.stamp = get_clock()->now();
    arrow.ns = "gimbal_goal_real"; arrow.id = 0;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x = 0.05; arrow.scale.y = 0.1; arrow.scale.z = 0.1;
    arrow.color.r = 1.0; arrow.color.a = 1.0;
    geometry_msgs::msg::Point p_start, p_end;
    p_start.x = cam_e; p_start.y = cam_n; p_start.z = cam_u;
    p_end.x = best_marker->e; p_end.y = best_marker->n; p_end.z = best_marker->u;
    arrow.points = {p_start, p_end};
    arr.markers.push_back(arrow);
    arrow_pub_->publish(arr);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
        "Target: %s | Cmd (P,Y): (%.1f, %.1f) deg",
        best_marker->name.c_str(), gimbal_pitch_deg, gimbal_yaw_deg);
  }

  // Member variables
  std::vector<Marker> markers_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
  // =========================== 오타 수정 ===========================
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr arrow_pub_;
  // ===============================================================
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  std::string current_gimbal_mode_ = "TRACK";
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buf_;
  tf2_ros::TransformListener tf_lis_;
  std::string map_frame_, base_frame_, cam_frame_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalAngleController>());
  rclcpp::shutdown();
  return 0;
}