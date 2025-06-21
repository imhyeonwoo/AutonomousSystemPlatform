// offboard_waypoint_trigger.cpp (Strategy 2: Viewpoint Follower)
//
// 기능:
// 1. 웨이포인트까지 비행 (기존 기능)
// 2. 추가 기능: 웨이포인트 도달 후, /desired_view_position 토픽을 구독
// 3. 수신된 위치를 새로운 목표 지점으로 삼아 실시간으로 드론 위치를 미세 조정

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <geometry_msgs/msg/point_stamped.hpp> // 추가

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
    
    // =========================== 추가된 부분 1 ===========================
    view_pos_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/desired_view_position", rclcpp::SystemDefaultsQoS(),
        [this](geometry_msgs::msg::PointStamped::SharedPtr msg){
            if (msg->header.frame_id == this->map_frame_) {
                this->desired_view_pos_ = msg->point;
            } else {
                RCLCPP_WARN_ONCE(this->get_logger(), "Received /desired_view_position with mismatched frame_id. Expected '%s', got '%s'",
                    this->map_frame_.c_str(), msg->header.frame_id.c_str());
            }
        });
    // =====================================================================

    timer_=create_wall_timer(50ms,[this]{onTimer();});

    RCLCPP_INFO(get_logger(),"웨이포인트 %zu 로드. /next_waypoint 및 /desired_view_position 사용", wps_.size());
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

    /* ─ 아직 웨이포인트에 못 도달 ─ */
    if(!reached_){
      const auto tgt_wp = wps_[wp_idx_];
      auto enu=enuNow(); if(!enu) return;
      
      // NED 좌표계의 PX4 위치에 ENU 좌표계의 오프셋을 더하기 위한 계산
      // last_pos_ (NED): x=N, y=E, z=D
      // enu (ENU): e=E, n=N, u=U
      // tgt_wp (ENU): e=E, n=N, u=U
      double dx_wp = tgt_wp.n - enu->n; // North offset
      double dy_wp = tgt_wp.e - enu->e; // East offset
      double dz_wp = -(tgt_wp.u - enu->u); // Down offset

      sendCtrl();
      publishSet(last_pos_->x + dx_wp, last_pos_->y + dy_wp, last_pos_->z + dz_wp);

      if(std::hypot(dx_wp, dy_wp) < 1.0 && std::fabs(dz_wp) < 1.0){
        reached_ = true;
        hold_x_ = last_pos_->x + dx_wp;
        hold_y_ = last_pos_->y + dy_wp;
        hold_z_ = last_pos_->z + dz_wp;
        RCLCPP_INFO(get_logger(),"WP %zu 도달 → Viewpoint 조정 시작", wp_idx_);
      }
      return;
    }

    /* ─ 도달 후 : Viewpoint 추종 및 트리거 대기 ─ */
    // =========================== 수정된 부분 ===========================
    if (desired_view_pos_) {
        // 수신된 view position의 ENU 좌표를 가져온다.
        const auto tgt_view = *desired_view_pos_;
        auto enu = enuNow(); 
        if (!enu) {
            // enu를 못가져오면 안전하게 마지막 위치에서 호버
            sendCtrl();
            publishSet(hold_x_, hold_y_, hold_z_);
            return;
        }
        
        // 목표(view_pos)와 현재(enu)의 차이(offset)를 계산한다.
        // Point.x -> E, Point.y -> N, Point.z -> U
        double dx_view = (float)tgt_view.y - enu->n; // North offset
        double dy_view = (float)tgt_view.x - enu->e; // East offset
        double dz_view = -((float)tgt_view.z - enu->u); // Down offset

        sendCtrl();
        // 현재 PX4 로컬 위치에 offset을 더해 새로운 setpoint를 발행한다.
        publishSet(last_pos_->x + dx_view, last_pos_->y + dy_view, last_pos_->z + dz_view);
    } else {
        // 아직 view position을 받지 못했다면, 기존처럼 도달한 지점에서 호버링한다.
        sendCtrl();
        publishSet(hold_x_, hold_y_, hold_z_);
    }
    // =====================================================================

    if(trig_next_){
      trig_next_=false; 
      reached_=false; 
      desired_view_pos_.reset(); // 다음 WP를 위해 수신된 위치 초기화
      ++wp_idx_;
      if(wp_idx_<wps_.size())
        RCLCPP_INFO(get_logger(),"Trigger 수신! → WP %zu 로 이동", wp_idx_);
    }
  }

  /* helpers */
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

  /* members */
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr off_ctrl_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr  traj_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr      cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr               two_fb_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr  trig_sub_;
  // =========================== 추가된 부분 2 ===========================
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr view_pos_sub_;
  std::optional<geometry_msgs::msg::Point> desired_view_pos_;
  // =====================================================================
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buf_; tf2_ros::TransformListener tf_;

  std::vector<ENU> wps_;
  size_t wp_idx_;
  uint8_t off_cnt_, marker_tick_;
  bool reached_, trig_next_, got_start_, takeoff_done_;
  std::optional<px4_msgs::msg::VehicleLocalPosition> last_pos_, start_pos_;
  double hold_x_, hold_y_, hold_z_;
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