// offboard_waypoint_trigger.cpp  (PD Landing + Cooperative Yaw v3)
//   • /desired_body_yaw_enu (rad, ENU) 구독 → NED 변환 후 TrajectorySetpoint.yaw
//   • 여타 로직(미션·착륙·PD) 그대로, 모든 publishSet() 에 yaw 적용
//   • 콜백에 auto 사용 안 함 (Humble SFINAE 문제 회피)

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <optional>
#include <cmath>
#include <chrono>
using namespace std::chrono_literals;

/* ---------- helpers ---------- */
static inline double wrap_pi(double x){ return std::remainder(x,2.0*M_PI); }

class WPTriggerNode : public rclcpp::Node
{
  enum class State { WAIT_SIGNAL, INIT, TAKEOFF, MISSION,
                     LANDING_APPROACH, LANDING_DESCENT, TOUCHDOWN, LAND };
  struct ENU { float e,n,u; };          // simple holder

public:
  WPTriggerNode()
  : Node("offboard_waypoint_trigger"),
    tf_buf_(this->get_clock()), tf_(tf_buf_),
    wp_idx_(0), off_cnt_(0), state_(State::WAIT_SIGNAL),
    got_start_(false), takeoff_done_(false),
    marker_tick_(0), reached_(false), trig_next_(false),
    prev_err_e_(0.0), prev_err_n_(0.0)
  {
    /* ───── parameters ───── */
    declare_parameter("map_frame","map");
    declare_parameter("base_frame","x500_gimbal_0");
    declare_parameter("takeoff_alt",5.0);
    declare_parameter("wp_csv","");
    declare_parameter("approach_alt",1.5);
    declare_parameter("descent_speed",1.0);
    declare_parameter("touchdown_alt",0.15);
    declare_parameter("landing_h_thresh",0.2);
    declare_parameter("kp_horizontal",0.8);
    declare_parameter("kd_horizontal",0.4);

    map_frame_        = get_parameter("map_frame").as_string();
    base_frame_       = get_parameter("base_frame").as_string();
    takeoff_alt_      = get_parameter("takeoff_alt").as_double();
    approach_alt_     = get_parameter("approach_alt").as_double();
    descent_speed_    = get_parameter("descent_speed").as_double();
    touchdown_alt_    = get_parameter("touchdown_alt").as_double();
    landing_h_thresh_ = get_parameter("landing_h_thresh").as_double();
    kp_horizontal_    = get_parameter("kp_horizontal").as_double();
    kd_horizontal_    = get_parameter("kd_horizontal").as_double();

    std::string csv = get_parameter("wp_csv").as_string();
    if(csv.empty())
      csv = ament_index_cpp::get_package_share_directory("px4_ros_com")
          + "/data/offboard_waypoints.csv";
    if(!loadCSV(csv))
      throw std::runtime_error("Waypoint CSV load error");

    /* ───── publishers ───── */
    auto px4_qos   = rclcpp::SensorDataQoS();
    auto marker_qos= rclcpp::QoS(1).reliable().transient_local();

    off_ctrl_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", px4_qos);
    traj_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", px4_qos);
    cmd_pub_  = create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", px4_qos);
    two_fb_pub_ = create_publisher<std_msgs::msg::Int32>("two_feedback",10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "waypoint_markers", marker_qos);
    gimbal_mode_pub_ = create_publisher<std_msgs::msg::String>("/gimbal_mode",10);

    /* ───── subscribers ───── */
    pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", px4_qos,
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg){
        last_pos_ = *msg;
      });

    start_sub_ = create_subscription<std_msgs::msg::Int32>(
      "one_feedback",10,
      [this](const std_msgs::msg::Int32::SharedPtr msg){
        if(msg->data==1 && !got_start_){ got_start_=true; state_=State::INIT; }});

    trig_sub_ = create_subscription<std_msgs::msg::Bool>(
      "next_waypoint",10,
      [this](const std_msgs::msg::Bool::SharedPtr msg){
        if(msg->data) trig_next_=true; });

    marker_pos_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/marker_enu_point",10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg){
        last_marker_pos_ = *msg; });

    /* ★ yaw (ENU) subscriber */
    yaw_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/desired_body_yaw_enu",10,
      [this](const std_msgs::msg::Float32::SharedPtr msg){
        if(!std::isnan(msg->data)) desired_yaw_enu_ = msg->data;
        else desired_yaw_enu_.reset();
      });

    /* ───── timer ───── */
    timer_ = create_wall_timer(50ms,[this]{ onTimer(); });
    RCLCPP_INFO(get_logger(),"WPTriggerNode with cooperative yaw ready");
  }

private:
  /* ===== helper ===== */
  uint64_t usec(){ return now().nanoseconds()/1000ULL; }

  double yawNed() const
  {
    if(!desired_yaw_enu_) return NAN;
    // ENU→NED : ψ_ned = π/2 – ψ_enu
    return wrap_pi(M_PI/2.0 - static_cast<double>(*desired_yaw_enu_));
  }

  void publishSet(double x,double y,double z){
    px4_msgs::msg::OffboardControlMode m{}; m.timestamp=usec(); m.position=true;
    off_ctrl_pub_->publish(m);

    px4_msgs::msg::TrajectorySetpoint sp{}; sp.timestamp=m.timestamp;
    sp.position[0]=x; sp.position[1]=y; sp.position[2]=z;
    sp.yaw = yawNed();          // may be NaN
    traj_pub_->publish(sp);
  }
  void hover(double x,double y,double z){ publishSet(x,y,z); }

  void sendCmd(uint16_t cmd, float p1=0.f, float p2=0.f){
    px4_msgs::msg::VehicleCommand c{}; c.timestamp=usec();
    c.target_system=1; c.target_component=1; c.command=cmd;
    c.param1=p1; c.param2=p2; cmd_pub_->publish(c);
  }

  /* ===== tf helper ===== */
  std::optional<ENU> enuNow(){
    try{
      auto tf = tf_buf_.lookupTransform(map_frame_,base_frame_,tf2::TimePointZero,20ms);
      return ENU{(float)tf.transform.translation.x,
                 (float)tf.transform.translation.y,
                 (float)tf.transform.translation.z};
    }catch(...){ return std::nullopt; }
  }

  /* ===== CSV loader ===== */
  bool loadCSV(const std::string& p){
    std::ifstream f(p); if(!f.is_open()) return false;
    std::string l; bool first=true;
    while(std::getline(f,l)){
      if(first){first=false; continue;}
      std::stringstream ss(l); std::string tok; std::vector<float> v;
      while(std::getline(ss,tok,',')) if(!tok.empty()) v.push_back(std::stof(tok));
      if(v.size()>=3) wps_.push_back({v[0],v[1],v[2]});
    }
    return !wps_.empty();
  }

  /* ===== main loop ===== */
  void onTimer(){
    if(++marker_tick_==20){ pubMarkers(); marker_tick_=0; }
    if(!last_pos_) return;

    switch(state_){
      case State::WAIT_SIGNAL:        hover(last_pos_->x,last_pos_->y,last_pos_->z); break;
      case State::INIT:               init(); break;
      case State::TAKEOFF:            takeoff(); break;
      case State::MISSION:            mission(); break;
      case State::LANDING_APPROACH:   landing_approach(); break;
      case State::LANDING_DESCENT:    landing_descent(); break;
      case State::TOUCHDOWN:          touchdown(); break;
      case State::LAND:               break;
    }
  }

  /* ===== states (UNCHANGED except yaw now in publishSet) ===== */
  void init(){
    if(off_cnt_<20){ publishSet(last_pos_->x,last_pos_->y,last_pos_->z); ++off_cnt_; return; }
    sendCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,1,6);
    sendCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,1);
    start_pos_=last_pos_; state_=State::TAKEOFF;
  }

  void takeoff(){
    double tgt_z = start_pos_->z - takeoff_alt_;
    publishSet(start_pos_->x,start_pos_->y,tgt_z);
    if(std::fabs(last_pos_->z - tgt_z) < 0.5){
      if(!takeoff_done_){ std_msgs::msg::Int32 f; f.data=1; two_fb_pub_->publish(f); takeoff_done_=true; }
      state_=State::MISSION;
    }
  }

  void mission(){
    if(wp_idx_>=wps_.size()){ state_=State::LANDING_APPROACH; return; }

    if(trig_next_ && reached_){
      trig_next_=false; reached_=false; ++wp_idx_;
      if(wp_idx_>=wps_.size()){ state_=State::LANDING_APPROACH; return; }
      if(wp_idx_==wps_.size()-1){ std_msgs::msg::String s; s.data="LOOK_DOWN"; gimbal_mode_pub_->publish(s);}
    }

    const auto tgt = wps_[wp_idx_];
    auto enu = enuNow(); if(!enu) return;
    double dx=tgt.n-enu->n, dy=tgt.e-enu->e, dz=-(tgt.u-enu->u);

    if(!reached_){
      publishSet(last_pos_->x+dx, last_pos_->y+dy, last_pos_->z+dz);
      if(std::hypot(dx,dy)<1.0 && std::fabs(dz)<1.0){
        reached_=true; hold_x_=last_pos_->x+dx; hold_y_=last_pos_->y+dy; hold_z_=last_pos_->z+dz;
      }
    }else{
      publishSet(hold_x_,hold_y_,hold_z_);
    }
  }

  void landing_approach(){
    if(!last_marker_pos_){ hover(hold_x_,hold_y_,hold_z_); return; }
    auto enu=enuNow(); if(!enu) return;
    const auto m=last_marker_pos_->point;
    double err_e=m.x-enu->e, err_n=m.y-enu->n, err_u=(m.z+approach_alt_)-enu->u;
    if(std::hypot(err_e,err_n)<landing_h_thresh_ && std::fabs(err_u)<0.2){
      descent_target_u_ = enu->u; state_=State::LANDING_DESCENT; return;
    }
    publishSet(last_pos_->x+err_n,last_pos_->y+err_e,last_pos_->z-err_u);
  }

  void landing_descent(){
    if(!last_marker_pos_){ hover(last_pos_->x,last_pos_->y,last_pos_->z); return; }
    auto enu=enuNow(); if(!enu) return;
    const auto m=last_marker_pos_->point;
    double alt_now = enu->u - m.z;
    if(alt_now<touchdown_alt_){ state_=State::TOUCHDOWN; return; }

    double dt=0.05;
    double err_e=m.x-enu->e, err_n=m.y-enu->n;
    double ctrl_e=kp_horizontal_*err_e + kd_horizontal_*(err_e-prev_err_e_)/dt;
    double ctrl_n=kp_horizontal_*err_n + kd_horizontal_*(err_n-prev_err_n_)/dt;
    prev_err_e_=err_e; prev_err_n_=err_n;

    descent_target_u_ -= descent_speed_*dt;
    double err_u = descent_target_u_-enu->u;

    publishSet(last_pos_->x+ctrl_n,last_pos_->y+ctrl_e,last_pos_->z-err_u);
  }

  void touchdown(){
    sendCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,0);
    state_=State::LAND;
  }

  /* ===== rviz markers ===== */
  void pubMarkers(){
    visualization_msgs::msg::MarkerArray arr; auto st=now();
    for(size_t i=0;i<wps_.size();++i){
      visualization_msgs::msg::Marker m;
      m.header.frame_id=map_frame_; m.header.stamp=st;
      m.ns="wp"; m.id=i; m.type=m.SPHERE; m.action=m.ADD;
      m.pose.position.x=wps_[i].e; m.pose.position.y=wps_[i].n; m.pose.position.z=wps_[i].u;
      m.scale.x=m.scale.y=m.scale.z=1.0; m.color.r=m.color.b=1.0; m.color.a=1.0;
      arr.markers.push_back(m);
    }
    marker_pub_->publish(arr);
  }

  /* ===== members ===== */
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr  off_ctrl_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr   traj_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr       cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr                two_fb_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr               gimbal_mode_pub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr               start_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                trig_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr   marker_pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr             yaw_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buf_; tf2_ros::TransformListener tf_;

  struct ENUwp{ float e,n,u; };
  std::vector<ENUwp> wps_;
  size_t wp_idx_;
  uint8_t off_cnt_, marker_tick_;
  bool reached_, trig_next_, got_start_, takeoff_done_;
  std::optional<px4_msgs::msg::VehicleLocalPosition> last_pos_, start_pos_;
  std::optional<geometry_msgs::msg::PointStamped>    last_marker_pos_;
  double hold_x_, hold_y_, hold_z_;
  State state_;

  /* params & PD */
  double takeoff_alt_;
  double approach_alt_,descent_speed_,touchdown_alt_,landing_h_thresh_;
  double descent_target_u_;
  double kp_horizontal_, kd_horizontal_;
  double prev_err_e_, prev_err_n_;
  std::string map_frame_, base_frame_;

  /* yaw cooperation */
  std::optional<float> desired_yaw_enu_;
};

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<WPTriggerNode>());
  rclcpp::shutdown();
  return 0;
}
