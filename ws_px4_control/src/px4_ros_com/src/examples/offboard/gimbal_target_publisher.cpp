// gimbal_target_publisher.cpp  (Body–Gimbal Cooperative v3)
//   • 짐벌 Pitch·Yaw → VEHICLE_CMD_DO_MOUNT_CONTROL (종전과 동일)
//   • 카메라 시선의 수평 ENU-yaw 를 /desired_body_yaw_enu 토픽으로 발행
//     ─ 드론 본체 yaw 제어 노드가 TrajectorySetpoint.yaw(NED) 로 변환·사용 가능
//   • rate-limit 옵션: body_yaw_rate_deg 파라미터(기본 30 deg/s)

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>           // ★ yaw 퍼블리시

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;

struct Marker { std::string name; double e,n,u; };

static inline double wrap_pi(double x) { return std::remainder(x, 2.0*M_PI); }

class GimbalAngleController : public rclcpp::Node {
public:
  GimbalAngleController()
  : Node("gimbal_target_publisher"),
    tf_buf_(this->get_clock()), tf_lis_(tf_buf_),
    last_cmd_yaw_(0.0)
  {
    /* ───────────────── Parameters ───────────────── */
    declare_parameter("map_frame",       "map");
    declare_parameter("base_frame",      "x500_gimbal_0");
    declare_parameter("camera_frame",    "x500_gimbal_0/camera_link");
    declare_parameter("marker_csv",      "");
    declare_parameter("body_yaw_rate_deg", 30.0);   // max slew-rate

    map_frame_   = get_parameter("map_frame").as_string();
    base_frame_  = get_parameter("base_frame").as_string();
    cam_frame_   = get_parameter("camera_frame").as_string();
    max_rate_rad_= get_parameter("body_yaw_rate_deg").as_double() * M_PI/180.0;

    /* ─────────── Load marker list ─────────── */
    std::string csv = get_parameter("marker_csv").as_string();
    if(csv.empty())
      csv = ament_index_cpp::get_package_share_directory("px4_ros_com") + "/data/aruco_markers.csv";
    if(!loadCSV(csv)){
      RCLCPP_FATAL(get_logger(),"Failed to load CSV %s", csv.c_str());
      throw std::runtime_error("marker csv");
    }

    /* ─────────── Pub / Sub  ─────────── */
    auto vis_qos = rclcpp::QoS(1).reliable().transient_local();

    cmd_pub_   = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    arrow_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("gimbal_arrow", vis_qos);
    yaw_pub_   = create_publisher<std_msgs::msg::Float32>("/desired_body_yaw_enu", 10);

    mode_sub_  = create_subscription<std_msgs::msg::String>(
      "/gimbal_mode", 10,
      [this](const std_msgs::msg::String::SharedPtr msg){
        if(current_gimbal_mode_ != msg->data)
          RCLCPP_INFO(get_logger(),"Gimbal mode → %s", msg->data.c_str());
        current_gimbal_mode_ = msg->data;
      });

    timer_     = create_wall_timer(50ms, [this]{ onTimer(); });
    prev_time_ = now();
    RCLCPP_INFO(get_logger(),"Gimbal Target Publisher v3 started.");
  }

private:
  /* ===== CSV ===== */
  bool loadCSV(const std::string& p){
    std::ifstream f(p); if(!f.is_open()) return false;
    std::string l; bool first=true;
    while(std::getline(f,l)){
      if(first){first=false; continue;}
      std::stringstream ss(l); std::string tok; std::vector<std::string> col;
      while(std::getline(ss,tok,',')) col.push_back(tok);
      if(col.size()>=4) markers_.push_back({col[0],std::stod(col[1]),std::stod(col[2]),std::stod(col[3])});
    }
    return !markers_.empty();
  }
  uint64_t usec(){ return now().nanoseconds()/1000ULL; }

  /* ===== main loop ===== */
  void onTimer()
  {
    rclcpp::Time t_now = now();
    double dt = (t_now - prev_time_).seconds();
    if(dt<=0.0) dt=0.05;
    prev_time_=t_now;

    /* LOOK_DOWN 모드 – 짐벌 pitch –90°, yaw 유지, 본체 yaw cmd NaN */
    if(current_gimbal_mode_=="LOOK_DOWN"){
      sendGimbalCmd(-90.0, 0.0);
      publishYaw(NAN);
      return;
    }

    /* TF ready? */
    if(!tf_buf_.canTransform(map_frame_, base_frame_, tf2::TimePointZero) ||
       !tf_buf_.canTransform(map_frame_, cam_frame_,  tf2::TimePointZero)){
      RCLCPP_INFO_ONCE(get_logger(),"Waiting for transforms…");
      return;
    }
    auto tf_drone = tf_buf_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    auto tf_cam   = tf_buf_.lookupTransform(map_frame_, cam_frame_,  tf2::TimePointZero);

    /* 카메라 위치 */
    double cam_e = tf_cam.transform.translation.x;
    double cam_n = tf_cam.transform.translation.y;
    double cam_u = tf_cam.transform.translation.z;

    /* 최근접 마커 */
    const Marker* best=nullptr; double best_d2=std::numeric_limits<double>::max();
    for(const auto& m:markers_){
      double dE=m.e-cam_e, dN=m.n-cam_n, dU=m.u-cam_u;
      double d2=dE*dE+dN*dN+dU*dU;
      if(d2<best_d2){ best_d2=d2; best=&m; }
    }
    if(!best) return;

    /* 드론 yaw */
    tf2::Quaternion q; tf2::fromMsg(tf_drone.transform.rotation,q);
    double roll,pitch,psi; tf2::Matrix3x3(q).getRPY(roll,pitch,psi);

    /* 타깃 yaw & pitch */
    double vec_e=best->e-tf_drone.transform.translation.x;
    double vec_n=best->n-tf_drone.transform.translation.y;
    double vec_u=best->u-cam_u;

    double yaw_world = std::atan2(vec_n, vec_e);                         // ENU
    double yaw_body  = wrap_pi(yaw_world - psi);
    double pitch_rad = std::atan2(vec_u, std::hypot(vec_e,vec_n));

    /* ───── 1) 짐벌 명령 ───── */
    sendGimbalCmd(pitch_rad*180/M_PI, -yaw_body*180/M_PI);

    /* ───── 2) 본체 yaw 목표 퍼블리시 ───── */
    double dyaw = wrap_pi(yaw_world - last_cmd_yaw_);
    double max_step = max_rate_rad_ * dt;
    dyaw = std::clamp(dyaw, -max_step, max_step);
    last_cmd_yaw_ = wrap_pi(last_cmd_yaw_ + dyaw);

    publishYaw(static_cast<float>(last_cmd_yaw_));

    /* ───── RViz arrow ───── */
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = map_frame_; arrow.header.stamp = t_now;
    arrow.ns="gimbal_goal"; arrow.id=0;
    arrow.type=visualization_msgs::msg::Marker::ARROW; arrow.action=arrow.ADD;
    arrow.scale.x=0.05; arrow.scale.y=0.1; arrow.scale.z=0.1;
    arrow.color.r=1.0; arrow.color.a=1.0;
    geometry_msgs::msg::Point p0,p1;
    p0.x=cam_e; p0.y=cam_n; p0.z=cam_u;
    p1.x=best->e; p1.y=best->n; p1.z=best->u;
    arrow.points={p0,p1}; arr.markers.push_back(arrow);
    arrow_pub_->publish(arr);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "Marker %s | Gimbal P,Y = %.1f, %.1f deg | BodyYaw_cmd(ENU)=%.1f°",
      best->name.c_str(), pitch_rad*180/M_PI, -yaw_body*180/M_PI,
      last_cmd_yaw_*180/M_PI);
  }

  /* ===== util ===== */
  void sendGimbalCmd(double pitch_deg,double yaw_deg){
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.timestamp=usec();
    cmd.target_system=1; cmd.target_component=1;
    cmd.command=px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_MOUNT_CONTROL;
    cmd.param1=pitch_deg; cmd.param2=0.0f; cmd.param3=yaw_deg; cmd.param7=2.0f;
    cmd_pub_->publish(cmd);
  }
  void publishYaw(float yaw_enu){
    std_msgs::msg::Float32 m; m.data=yaw_enu; yaw_pub_->publish(m);
  }

  /* ===== members ===== */
  std::vector<Marker> markers_;

  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr        cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr arrow_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr               yaw_pub_;   // ★
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr             mode_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buf_; tf2_ros::TransformListener tf_lis_;

  std::string map_frame_, base_frame_, cam_frame_;
  std::string current_gimbal_mode_="TRACK";

  double max_rate_rad_;          // yaw slew-rate
  double last_cmd_yaw_;          // ENU absolute
  rclcpp::Time prev_time_;
};

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<GimbalAngleController>());
  rclcpp::shutdown();
  return 0;
}
