// gimbal_target_publisher.cpp
// PX4 – Gazebo 짐벌 제어 + 시각화
// 2025-06-27  ChatGPT

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>
#include <chrono>
using namespace std::chrono_literals;

/* ─ CSV 행 → Marker ─ */
struct Marker {
  std::string name;
  double e,n,u;        // ENU
};

class GimbalTargetPublisher : public rclcpp::Node
{
public:
  GimbalTargetPublisher()
  : Node("gimbal_target_publisher"),
    tf_buf_(this->get_clock()),
    tf_lis_(tf_buf_)
  {
    /* ─ Parameter ─ */
    declare_parameter("map_frame",      "map");
    declare_parameter("camera_frame",   "x500_gimbal_0/camera_link");
    declare_parameter("marker_csv",     "");
    declare_parameter("max_range_m",    40.0);
    declare_parameter("pitch_min_deg", -90.0);
    declare_parameter("pitch_max_deg",   0.0);

    map_frame_ = get_parameter("map_frame").as_string();
    cam_frame_ = get_parameter("camera_frame").as_string();
    max_range_ = get_parameter("max_range_m").as_double();
    pitch_min_ = get_parameter("pitch_min_deg").as_double();
    pitch_max_ = get_parameter("pitch_max_deg").as_double();

    std::string csv = get_parameter("marker_csv").as_string();
    if(csv.empty())
      csv = ament_index_cpp::get_package_share_directory("px4_ros_com")+"/data/aruco_markers.csv";
    if(!loadCSV(csv))
      throw std::runtime_error("marker csv load failed");

    /* ─ ROS I/O ─ */
    pitch_dbg_pub_ = create_publisher<std_msgs::msg::Float32>("/gimbal_pitch_degree",10);
    yaw_dbg_pub_   = create_publisher<std_msgs::msg::Float32>("/gimbal_yaw_degree",10);
    cmd_pub_       = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command",10);
    arrow_pub_     = create_publisher<visualization_msgs::msg::MarkerArray>("gimbal_arrow",
                      rclcpp::QoS(1).transient_local());

    pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
      [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr m){ last_pos_ = *m; });

    timer_ = create_wall_timer(100ms, [this]{ timerCb(); });

    RCLCPP_INFO(get_logger(),"gimbal_target_publisher – markers=%zu  cam_frame=%s",
                markers_.size(), cam_frame_.c_str());
  }

private:
  /* ─ CSV 파싱 ─ */
  bool loadCSV(const std::string& path)
  {
    std::ifstream fs(path);
    if(!fs.is_open()){ RCLCPP_ERROR(get_logger(),"open fail %s",path.c_str()); return false; }

    std::string line; bool first=true;
    while(std::getline(fs,line)){
      if(first){ first=false; continue; }
      std::stringstream ss(line); std::string tok;
      std::vector<std::string> col;
      while(std::getline(ss,tok,',')) col.push_back(tok);
      if(col.size()<4) continue;
      Marker m; m.name=col[0]; m.e=std::stod(col[1]); m.n=std::stod(col[2]); m.u=std::stod(col[3]);
      markers_.push_back(m);
    }
    return !markers_.empty();
  }

  /* ─ 메인 루프 ─ */
  void timerCb()
  {
    if(!last_pos_) return;

    geometry_msgs::msg::TransformStamped tf;
    try{
      tf = tf_buf_.lookupTransform(map_frame_, cam_frame_, tf2::TimePointZero, 50ms);
    }catch(const tf2::TransformException& ex){
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(),2000,"TF %s",ex.what());
      return;
    }

    double ce=tf.transform.translation.x;
    double cn=tf.transform.translation.y;
    double cu=tf.transform.translation.z;

    /* 최근접 마커 선택 */
    const Marker* best=nullptr; double best_d2=std::numeric_limits<double>::max();
    for(const auto& m:markers_){
      double dE=m.e-ce, dN=m.n-cn, dU=m.u-cu;
      double d2=dE*dE+dN*dN+dU*dU;
      if(d2<best_d2 && std::sqrt(d2)<=max_range_){ best_d2=d2; best=&m; }
    }
    if(!best) return;

    /* ENU 차이 */
    double dE = best->e - ce;
    double dN = best->n - cn;
    double dU = best->u - cu;

    /* Body-기준 변환 : heading(rad) CW + */
    double hd = last_pos_->heading;
    double cosH = std::cos(hd), sinH = std::sin(hd);
    double x_b =  dE*cosH + dN*sinH;
    double y_b = -dE*sinH + dN*cosH;

    /* 각도 계산 */
    double yaw_deg   = std::atan2(y_b, x_b) * 180./M_PI;                    // 0°=정면
    double pitch_deg = -std::atan2(dU, std::hypot(x_b, y_b)) * 180./M_PI;   // 아래 음
    pitch_deg = std::clamp(pitch_deg, pitch_min_, pitch_max_);

    { std_msgs::msg::Float32 m; m.data=pitch_deg; pitch_dbg_pub_->publish(m);}
    { std_msgs::msg::Float32 m; m.data=yaw_deg;   yaw_dbg_pub_->publish(m); }

    /* ─ PX4 짐벌 명령 ─ */
    px4_msgs::msg::VehicleCommand vc{};
    vc.timestamp        = nowUs();
    vc.target_system    = 1;
    vc.target_component = 1;
    vc.command          = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_MOUNT_CONTROL;
    vc.param1 = pitch_deg;          // pitch
    vc.param2 = 0.0;                // roll(사용안함)
    vc.param3 = yaw_deg;            // yaw
    vc.param7 = 2;                  // MAV_MOUNT_MODE_MAVLINK_TARGETING
    cmd_pub_->publish(vc);

    publishMarkers(ce,cn,cu,*best,yaw_deg,pitch_deg,hd);
  }

  /* ─ RViz 시각화 ─ */
  void publishMarkers(double ce,double cn,double cu,
                      const Marker& tgt,double yaw_deg,double pitch_deg,double hd)
  {
    visualization_msgs::msg::MarkerArray arr; auto st=now();

    /* ① 목표 방향(빨강) */
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id=map_frame_; arrow.header.stamp=st;
    arrow.ns="gimbal_goal"; arrow.id=0; arrow.type=arrow.ARROW; arrow.action=arrow.ADD;
    arrow.scale.x=0.05; arrow.scale.y=0.1; arrow.scale.z=0.1;
    arrow.color.r=1.0; arrow.color.a=1.0;
    geometry_msgs::msg::Point p0,p1;
    p0.x=ce; p0.y=cn; p0.z=cu;
    p1.x=tgt.e; p1.y=tgt.n; p1.z=tgt.u;
    arrow.points={p0,p1};
    arr.markers.push_back(arrow);

    /* ② 실제 시선(민트색) 길이 3 m */
    double yaw_b = yaw_deg*M_PI/180.;
    double pit_b = pitch_deg*M_PI/180.;
    double x_b =  std::cos(pit_b)*std::cos(yaw_b);
    double y_b =  std::cos(pit_b)*std::sin(yaw_b);
    double z_b = -std::sin(pit_b);

    double cosH=std::cos(hd), sinH=std::sin(hd);
    double dE =  x_b*cosH - y_b*sinH;
    double dN =  x_b*sinH + y_b*cosH;
    double dU =  z_b;

    visualization_msgs::msg::Marker cam;
    cam = arrow; cam.ns="gimbal_actual"; cam.id=1;
    cam.color.r=0.0; cam.color.g=1.0; cam.color.b=1.0;            // 민트색
    geometry_msgs::msg::Point q1;
    q1.x = ce + 3.0*dE;
    q1.y = cn + 3.0*dN;
    q1.z = cu + 3.0*dU;
    cam.points={p0,q1};
    arr.markers.push_back(cam);

    arrow_pub_->publish(arr);
  }

  uint64_t nowUs(){ return get_clock()->now().nanoseconds()/1000ULL; }

  /* ─ data ─ */
  std::vector<Marker> markers_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_dbg_pub_, yaw_dbg_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr arrow_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buf_; tf2_ros::TransformListener tf_lis_;
  std::optional<px4_msgs::msg::VehicleLocalPosition> last_pos_;

  std::string map_frame_, cam_frame_;
  double max_range_, pitch_min_, pitch_max_;
};

/* ─ main ─ */
int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<GimbalTargetPublisher>());
  rclcpp::shutdown();
  return 0;
}
