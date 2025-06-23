// offboard_waypoint_trigger.cpp (PD Control & Gentle Landing Final Version)

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
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

class WPTriggerNode : public rclcpp::Node
{
  enum class State { WAIT_SIGNAL, INIT, TAKEOFF, MISSION, LANDING_APPROACH, LANDING_DESCENT, TOUCHDOWN, LAND };
  struct ENU { float e,n,u; };

public:
  WPTriggerNode()
  : Node("offboard_waypoint_trigger"),
    tf_buf_(this->get_clock()), tf_(tf_buf_),
    wp_idx_(0), off_cnt_(0), state_(State::WAIT_SIGNAL),
    got_start_(false), takeoff_done_(false),
    marker_tick_(0), reached_(false), trig_next_(false),
    prev_err_e_(0.0), prev_err_n_(0.0)
  {
    /* ─ parameters ─ */
    declare_parameter("map_frame","map");
    declare_parameter("base_frame","x500_gimbal_0");
    declare_parameter("takeoff_alt",5.0);
    declare_parameter("wp_csv","");
    declare_parameter("approach_alt", 1.5);
    declare_parameter("descent_speed", 0.3);
    declare_parameter("touchdown_alt", 0.15); // [수정] 지면 접촉 감지를 위해 매우 낮게 설정
    declare_parameter("landing_h_thresh", 0.2);
    // PD 제어 게인 파라미터 (튜닝 필요)
    declare_parameter("kp_horizontal", 0.8);
    declare_parameter("kd_horizontal", 0.4);

    map_frame_      = get_parameter("map_frame").as_string();
    base_frame_     = get_parameter("base_frame").as_string();
    takeoff_alt_    = get_parameter("takeoff_alt").as_double();
    approach_alt_     = get_parameter("approach_alt").as_double();
    descent_speed_    = get_parameter("descent_speed").as_double();
    touchdown_alt_    = get_parameter("touchdown_alt").as_double();
    landing_h_thresh_ = get_parameter("landing_h_thresh").as_double();
    kp_horizontal_    = get_parameter("kp_horizontal").as_double();
    kd_horizontal_    = get_parameter("kd_horizontal").as_double();

    std::string csv = get_parameter("wp_csv").as_string();
    if(csv.empty())
      csv = ament_index_cpp::get_package_share_directory("px4_ros_com") + "/data/offboard_waypoints.csv";
    if(!loadCSV(csv)){
      RCLCPP_FATAL(get_logger(),"CSV %s 로드 실패", csv.c_str());
      throw std::runtime_error("csv");
    }

    /* ─ pubs/subs ─ */
    auto px4_qos=rclcpp::SensorDataQoS();
    auto marker_qos=rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    off_ctrl_pub_=create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode",px4_qos);
    traj_pub_    =create_publisher<px4_msgs::msg::TrajectorySetpoint> ("/fmu/in/trajectory_setpoint" ,px4_qos);
    cmd_pub_     =create_publisher<px4_msgs::msg::VehicleCommand>     ("/fmu/in/vehicle_command"     ,px4_qos);
    two_fb_pub_  =create_publisher<std_msgs::msg::Int32>("two_feedback",10);
    marker_pub_  =create_publisher<visualization_msgs::msg::MarkerArray>("waypoint_markers",marker_qos);
    gimbal_mode_pub_ = create_publisher<std_msgs::msg::String>("/gimbal_mode", 10);
    pos_sub_=create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position",px4_qos,
      [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr m){ last_pos_=*m; });
    start_sub_=create_subscription<std_msgs::msg::Int32>(
      "one_feedback",10,
      [this](std_msgs::msg::Int32::SharedPtr m){ if(m->data==1 && !got_start_){ got_start_=true; state_=State::INIT; }});
    trig_sub_=create_subscription<std_msgs::msg::Bool>(
      "next_waypoint",10,
      [this](std_msgs::msg::Bool::SharedPtr m){ if(m->data) trig_next_=true; });
    marker_pos_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/marker_enu_point", 10,
      [this](geometry_msgs::msg::PointStamped::SharedPtr msg){
        last_marker_pos_ = *msg;
      });
    timer_=create_wall_timer(50ms,[this]{onTimer();});
    RCLCPP_INFO(get_logger(),"PD 제어 기반 정밀 착륙 제어기 활성화.");
  }

private:
  // onTimer, init, takeoff, mission 등은 이전과 동일
  void onTimer(){
    if(++marker_tick_>=20){ pubMarkers(); marker_tick_=0; }
    if(!last_pos_) return;

    switch(state_){
      case State::WAIT_SIGNAL: hover(last_pos_->x,last_pos_->y,last_pos_->z); break;
      case State::INIT: init(); break;
      case State::TAKEOFF: takeoff(); break;
      case State::MISSION: mission(); break;
      case State::LANDING_APPROACH: landing_approach(); break;
      case State::LANDING_DESCENT: landing_descent(); break;
      case State::TOUCHDOWN: touchdown(); break;
      case State::LAND: break;
      default: break;
    }
  }

  void init(){
    if(off_cnt_<20){ sendCtrl(); hover(last_pos_->x,last_pos_->y,last_pos_->z); ++off_cnt_; return; }
    sendCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,1,6);
    sendCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,1);
    start_pos_=last_pos_; state_=State::TAKEOFF;
    RCLCPP_INFO(get_logger(),"OFFBOARD+ARM → TAKEOFF");
  }

  void takeoff(){
    double tgt_z = start_pos_->z - takeoff_alt_;
    sendCtrl(); hover(start_pos_->x,start_pos_->y,tgt_z);
    if(std::fabs(last_pos_->z - tgt_z)<0.5){
      if(!takeoff_done_){ 
        std_msgs::msg::Int32 f; f.data=1; two_fb_pub_->publish(f); 
        takeoff_done_=true; 
      }
      state_=State::MISSION;
      RCLCPP_INFO(get_logger(),"이륙 완료 → WP 0");
    }
  }
  
  void mission(){
    if(wp_idx_ >= wps_.size()){
      state_ = State::LANDING_APPROACH;
      RCLCPP_INFO(get_logger(), "모든 WP 완료 → 착륙 접근 모드로 전환.");
      return;
    }

    if(trig_next_){
      trig_next_ = false; 
      reached_ = false;
      ++wp_idx_;
      
      if(wp_idx_ >= wps_.size()) {
          state_ = State::LANDING_APPROACH;
          RCLCPP_INFO(get_logger(), "Trigger 수신, 모든 WP 완료 → 착륙 접근 모드 전환.");
          return;
      }

      if(wp_idx_ == wps_.size() - 1) {
        RCLCPP_INFO(get_logger(),"Trigger 수신! → 마지막 WP(%zu)로 이동하며 아래를 주시합니다.", wp_idx_);
        std_msgs::msg::String gimbal_mode_msg;
        gimbal_mode_msg.data = "LOOK_DOWN";
        gimbal_mode_pub_->publish(gimbal_mode_msg);
      } else {
        RCLCPP_INFO(get_logger(),"Trigger 수신! → WP %zu 로 이동", wp_idx_);
      }
    }

    const auto tgt = wps_[wp_idx_];
    if(!reached_){
      auto enu = enuNow();
      if(!enu) return;
      double dx =  tgt.n - enu->n;
      double dy =  tgt.e - enu->e;
      double dz = -(tgt.u - enu->u);
      sendCtrl();
      publishSet(last_pos_->x + dx, last_pos_->y + dy, last_pos_->z + dz);
      if(std::hypot(dx,dy) < 1.0 && std::fabs(dz) < 1.0){
        reached_ = true;
        hold_x_ = last_pos_->x + dx;
        hold_y_ = last_pos_->y + dy;
        hold_z_ = last_pos_->z + dz;
        RCLCPP_INFO(get_logger(),"WP %zu 도달 → Hover. 다음 Trigger 대기 중...", wp_idx_);
      }
    } else {
      sendCtrl();
      publishSet(hold_x_, hold_y_, hold_z_);
    }
  }

  void landing_approach() {
      auto drone_enu_opt = enuNow();
      if (!last_marker_pos_ || !last_pos_ || !drone_enu_opt) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "착륙 접근 중... 마커/위치 정보 대기 중.");
          sendCtrl();
          hover(hold_x_, hold_y_, hold_z_);
          return;
      }
      auto drone_enu = drone_enu_opt.value();
      const auto marker_enu = last_marker_pos_->point;

      double err_e = marker_enu.x - drone_enu.e;
      double err_n = marker_enu.y - drone_enu.n;
      double err_u = (marker_enu.z + approach_alt_) - drone_enu.u;
      double dist_h = std::hypot(err_e, err_n);

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
          "착륙 접근... 수평 오차: %.2f m, 고도 오차: %.2f m", dist_h, err_u);

      if (dist_h < landing_h_thresh_ && std::abs(err_u) < 0.2) {
          RCLCPP_INFO(get_logger(), "목표 상공 접근 완료 → 정밀 하강 시작.");
          descent_target_u_ = drone_enu.u;
          prev_err_e_ = err_e;
          prev_err_n_ = err_n;
          state_ = State::LANDING_DESCENT;
          return;
      }
      sendCtrl();
      publishSet(last_pos_->x + err_n, last_pos_->y + err_e, last_pos_->z - err_u);
  }

  void landing_descent() {
      auto drone_enu_opt = enuNow();
      if (!last_marker_pos_ || !last_pos_ || !drone_enu_opt) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "정밀 하강 중 마커/위치 유실! 현재 위치에서 호버링합니다.");
          sendCtrl();
          hover(last_pos_->x, last_pos_->y, last_pos_->z);
          return;
      }
      auto drone_enu = drone_enu_opt.value();
      const auto marker_enu = last_marker_pos_->point;

      // [수정] 착륙 감지 로직
      double current_alt_from_marker = drone_enu.u - marker_enu.z;
      if (current_alt_from_marker < touchdown_alt_) {
          RCLCPP_INFO(get_logger(), "지면 접촉 고도 도달 → 터치다운 실행.");
          state_ = State::TOUCHDOWN;
          return;
      }

      // 수평 제어 로직 (PD Controller)
      double dt = 0.05;
      double err_e = marker_enu.x - drone_enu.e;
      double err_n = marker_enu.y - drone_enu.n;
      double deriv_e = (err_e - prev_err_e_) / dt;
      double deriv_n = (err_n - prev_err_n_) / dt;
      double control_e = kp_horizontal_ * err_e + kd_horizontal_ * deriv_e;
      double control_n = kp_horizontal_ * err_n + kd_horizontal_ * deriv_n;
      prev_err_e_ = err_e;
      prev_err_n_ = err_n;
      
      // 수직 제어 로직
      double descent_step = descent_speed_ * dt;
      descent_target_u_ -= descent_step;
      double err_u = descent_target_u_ - drone_enu.u;

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
          "정밀 하강(PD)... 고도: %.2f m, 수평 오차: %.2f m", current_alt_from_marker, std::hypot(err_e, err_n));

      sendCtrl();
      publishSet(last_pos_->x + control_n, last_pos_->y + control_e, last_pos_->z - err_u);
  }

  void touchdown() {
      // [수정] 부드러운 착지를 위해 Disarm 사용
      RCLCPP_INFO(get_logger(), "착지 명령(Disarm) 전송.");
      sendCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
      state_ = State::LAND;
  }

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

  std::optional<ENU> enuNow(){
    try{
      auto tf=tf_buf_.lookupTransform(map_frame_,base_frame_,tf2::TimePointZero,20ms);
      return ENU{(float)tf.transform.translation.x,(float)tf.transform.translation.y,(float)tf.transform.translation.z};
    }catch(...){return std::nullopt;}
  }

  void sendCtrl(){ px4_msgs::msg::OffboardControlMode m{}; m.timestamp=usec(); m.position=true; off_ctrl_pub_->publish(m);}
  void publishSet(double x,double y,double z, float yaw = NAN){ px4_msgs::msg::TrajectorySetpoint sp{}; sp.timestamp=usec(); sp.position[0]=x; sp.position[1]=y; sp.position[2]=z; sp.yaw=yaw; traj_pub_->publish(sp);}
  void hover(double x,double y,double z){ publishSet(x,y,z); }
  void sendCmd(uint16_t c,float p1=0,float p2=0){ px4_msgs::msg::VehicleCommand v{}; v.timestamp=usec(); v.target_system=1; v.target_component=1; v.command=c; v.param1=p1; v.param2=p2; cmd_pub_->publish(v);}
  uint64_t usec(){ return (uint64_t)(get_clock()->now().nanoseconds()/1000ULL); }

  void pubMarkers(){
    visualization_msgs::msg::MarkerArray arr; auto st=now();
    for(size_t i=0;i<wps_.size();++i){
      visualization_msgs::msg::Marker m; m.header.frame_id=map_frame_; m.header.stamp=st;
      m.ns="wp"; m.id=(int)i; m.type=m.SPHERE; m.action=m.ADD;
      m.pose.position.x=wps_[i].e; m.pose.position.y=wps_[i].n; m.pose.position.z=wps_[i].u;
      m.pose.orientation.w=1.0; m.scale.x=m.scale.y=m.scale.z=1.0;
      m.color.r=m.color.b=1.0; m.color.a=1.0; arr.markers.push_back(m);
      auto t=m; t.ns="wp_text"; t.id=i+wps_.size(); t.type=t.TEXT_VIEW_FACING; t.pose.position.z+=1; t.scale.z=0.7; t.text="WP "+std::to_string(i); arr.markers.push_back(t);
    }
    marker_pub_->publish(arr);
  }

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr off_ctrl_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr  traj_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr      cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr               two_fb_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gimbal_mode_pub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr  trig_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr marker_pos_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buf_; tf2_ros::TransformListener tf_;

  std::vector<ENU> wps_;
  size_t wp_idx_;
  uint8_t off_cnt_, marker_tick_;
  bool reached_, trig_next_, got_start_, takeoff_done_;
  std::optional<px4_msgs::msg::VehicleLocalPosition> last_pos_, start_pos_;
  std::optional<geometry_msgs::msg::PointStamped> last_marker_pos_;
  double hold_x_, hold_y_, hold_z_;
  State state_;
  
  double takeoff_alt_;
  double approach_alt_, descent_speed_, touchdown_alt_, landing_h_thresh_;
  double descent_target_u_;
  // PD 제어 관련 멤버 변수
  double kp_horizontal_, kd_horizontal_;
  double prev_err_e_, prev_err_n_;
  std::string map_frame_, base_frame_;
};

int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<WPTriggerNode>());
  rclcpp::shutdown();
  return 0;
}