// offboard_waypoint_trigger.cpp
// PX4 ROS 2 – /next_waypoint Bool(True) 트리거 & 정확 Hover
// 2025-06-24 ChatGPT – hover NED fix

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
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
  enum class State { WAIT_SIGNAL, INIT, TAKEOFF, MISSION, LAND };
  struct ENU { float e,n,u; };

public:
  WPTriggerNode()
  : Node("offboard_waypoint_trigger"),
    tf_buf_(this->get_clock()), tf_(tf_buf_),
    wp_idx_(0), off_cnt_(0), state_(State::WAIT_SIGNAL),
    got_start_(false), takeoff_done_(false),
    marker_tick_(0), reached_(false), trig_next_(false)
  {
    /* ─ parameters ─ */
    declare_parameter("map_frame","map");
    declare_parameter("base_frame","x500_gimbal_0");
    declare_parameter("takeoff_alt",5.0);
    declare_parameter("wp_csv","");
    map_frame_   = get_parameter("map_frame").as_string();
    base_frame_  = get_parameter("base_frame").as_string();
    takeoff_alt_ = get_parameter("takeoff_alt").as_double();

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

    pos_sub_=create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position",px4_qos,
      [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr m){ last_pos_=*m; });

    start_sub_=create_subscription<std_msgs::msg::Int32>(
      "one_feedback",10,
      [this](std_msgs::msg::Int32::SharedPtr m){ if(m->data==1 && !got_start_){ got_start_=true; state_=State::INIT; }});

    trig_sub_=create_subscription<std_msgs::msg::Bool>(
      "next_waypoint",10,
      [this](std_msgs::msg::Bool::SharedPtr m){ if(m->data) trig_next_=true; });

    timer_=create_wall_timer(50ms,[this]{onTimer();});

    RCLCPP_INFO(get_logger(),"웨이포인트 %zu 로드. /next_waypoint 사용", wps_.size());
  }

private:
  /* csv */
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

  /* timer */
  void onTimer(){
    if(++marker_tick_>=20){ pubMarkers(); marker_tick_=0; }
    if(!last_pos_) return;

    switch(state_){
      case State::WAIT_SIGNAL: hover(last_pos_->x,last_pos_->y,last_pos_->z); break;
      case State::INIT: init(); break;
      case State::TAKEOFF: takeoff(); break;
      case State::MISSION: mission(); break;
      default: break;
    }
  }

  /* states */
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
      if(!takeoff_done_){ std_msgs::msg::Int32 f; f.data=1; two_fb_pub_->publish(f); takeoff_done_=true; }
      state_=State::MISSION;
      RCLCPP_INFO(get_logger(),"이륙 완료 → WP 0");
    }
  }

  void mission(){
    if(wp_idx_>=wps_.size()){ sendCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND); state_=State::LAND; return; }

    const auto tgt = wps_[wp_idx_];

    /* ─ 아직 못 도달 ─ */
    if(!reached_){
      auto enu=enuNow(); if(!enu) return;
      double dx =  tgt.n - enu->n;
      double dy =  tgt.e - enu->e;
      double dz = -(tgt.u - enu->u);

      sendCtrl();
      publishSet(last_pos_->x + dx, last_pos_->y + dy, last_pos_->z + dz);

      if(std::hypot(dx,dy) < 1.0 && std::fabs(dz) < 1.0){
        reached_ = true;

        /* 도달 시 → 해당 위치 NED 좌표를 hold_*_ 에 저장 */
        hold_x_ = last_pos_->x + dx;
        hold_y_ = last_pos_->y + dy;
        hold_z_ = last_pos_->z + dz;

        RCLCPP_INFO(get_logger(),"WP %zu 도달 → Aruco DETECTING (Hover)", wp_idx_);
      }
      return;
    }

    /* ─ 도달 후 : Hover & 트리거 대기 ─ */
    sendCtrl();
    publishSet(hold_x_, hold_y_, hold_z_);   // 정확 고정

    if(trig_next_){
      trig_next_=false; reached_=false; ++wp_idx_;
      if(wp_idx_<wps_.size())
        RCLCPP_INFO(get_logger(),"Aruco DETECTED! → WP %zu", wp_idx_);
    }
  }

  /* helpers */
  void sendCtrl(){ px4_msgs::msg::OffboardControlMode m{}; m.timestamp=usec(); m.position=true; off_ctrl_pub_->publish(m);}
  void publishSet(double x,double y,double z){ px4_msgs::msg::TrajectorySetpoint sp{}; sp.timestamp=usec(); sp.position[0]=x; sp.position[1]=y; sp.position[2]=z; sp.yaw=0; traj_pub_->publish(sp);}
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

  /* members */
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr off_ctrl_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr  traj_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr      cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr               two_fb_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr  trig_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buf_; tf2_ros::TransformListener tf_;

  std::vector<ENU> wps_;
  size_t wp_idx_;
  uint8_t off_cnt_, marker_tick_;
  bool reached_, trig_next_, got_start_, takeoff_done_;
  std::optional<px4_msgs::msg::VehicleLocalPosition> last_pos_, start_pos_;
  double hold_x_, hold_y_, hold_z_;          // ← Hover NED 좌표
  State state_;
  double takeoff_alt_;
  std::string map_frame_, base_frame_;
};

/* main */
int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<WPTriggerNode>());
  rclcpp::shutdown();
  return 0;
}
