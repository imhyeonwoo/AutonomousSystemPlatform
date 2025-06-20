// offboard_waypoint_map.cpp
// PX4 ROS 2 – UAV 오프보드 미션 (UGV ↔ UAV hand-shake)
// 2025-06-22 ChatGPT – CSV 웨이포인트 + 전체 비행 로직

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <optional>
#include <chrono>
#include <cmath>
using namespace std::chrono_literals;

/************************************************************
 * 노드 정의
 ************************************************************/
class OffboardWaypointMap : public rclcpp::Node
{
  enum class State { WAIT_SIGNAL, INIT, TAKEOFF, MISSION, LAND };
  struct ENU { float e,n,u; };

public:
  OffboardWaypointMap()
  : Node("offboard_waypoint_map"),
    tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_),
    current_wp_idx_(0), offboard_counter_(0),
    state_(State::WAIT_SIGNAL),
    start_signal_received_(false), takeoff_done_pub_(false),
    marker_tick_(0)
  {
    /* ───── 파라미터 ───── */
    declare_parameter("map_frame",  "map");
    declare_parameter("base_frame", "x500_gimbal_0");
    declare_parameter("takeoff_alt", 5.0);
    declare_parameter("wp_csv",      "");                // CSV 경로 (옵션)

    map_frame_     = get_parameter("map_frame").as_string();
    base_frame_    = get_parameter("base_frame").as_string();
    takeoff_alt_m_ = get_parameter("takeoff_alt").as_double();

    std::string csv_path = get_parameter("wp_csv").as_string();
    if (csv_path.empty()) {
      csv_path = ament_index_cpp::get_package_share_directory("px4_ros_com")
               + "/data/offboard_waypoints.csv";
    }
    if (!loadWaypoints(csv_path)) {
      RCLCPP_FATAL(get_logger(), "웨이포인트 CSV 로드 실패: %s", csv_path.c_str());
      throw std::runtime_error("CSV load failed");
    }
    RCLCPP_INFO(get_logger(), "로드된 웨이포인트: %zu", enu_wps_.size());

    /* ───── QoS & I/O ───── */
    auto px4_qos    = rclcpp::SensorDataQoS();
    auto marker_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    offboard_ctrl_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
                                "/fmu/in/offboard_control_mode", px4_qos);
    traj_setpoint_pub_      = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
                                "/fmu/in/trajectory_setpoint",   px4_qos);
    vehicle_cmd_pub_        = create_publisher<px4_msgs::msg::VehicleCommand>(
                                "/fmu/in/vehicle_command",       px4_qos);
    two_fb_pub_             = create_publisher<std_msgs::msg::Int32>(
                                "two_feedback", 10);
    marker_pub_             = create_publisher<visualization_msgs::msg::MarkerArray>(
                                "waypoint_markers", marker_qos);

    local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", px4_qos,
      std::bind(&OffboardWaypointMap::localPosCb, this, std::placeholders::_1));
    one_fb_sub_    = create_subscription<std_msgs::msg::Int32>(
      "one_feedback", 10,
      std::bind(&OffboardWaypointMap::oneFeedbackCb, this, std::placeholders::_1));

    /* 20 Hz 주기 */
    timer_ = create_wall_timer(50ms, std::bind(&OffboardWaypointMap::timerCb, this));
  }

private:
  /**************** CSV 로드 ****************/
  bool loadWaypoints(const std::string& path)
  {
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;

    std::string line; bool first = true;
    while (std::getline(ifs, line)) {
      if (first) { first = false; continue; }   // header skip
      std::stringstream ss(line);
      std::vector<float> v; std::string tok;
      while (std::getline(ss, tok, ',')) {
        if (!tok.empty()) v.push_back(std::stof(tok));
      }
      if (v.size() >= 3) enu_wps_.push_back({v[0], v[1], v[2]});
    }
    return !enu_wps_.empty();
  }

  /**************** 위치·신호 콜백 ****************/
  void localPosCb(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
  { last_local_pos_ = *msg; }

  void oneFeedbackCb(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (msg->data == 1 && !start_signal_received_) {
      start_signal_received_ = true;
      state_ = State::INIT;
      RCLCPP_INFO(get_logger(), "[UAV] one_feedback=1 수신 → INIT 진입");
    }
  }

  /**************** ENU 현재 위치 ****************/
  std::optional<ENU> currentENU()
  {
    try {
      auto tf = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero, 20ms);
      return ENU{static_cast<float>(tf.transform.translation.x),
                 static_cast<float>(tf.transform.translation.y),
                 static_cast<float>(tf.transform.translation.z)};
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF 실패: %s", ex.what());
      return std::nullopt;
    }
  }

  /**************** 메인 타이머 ****************/
  void timerCb()
  {
    if (++marker_tick_ >= 20) { publishWaypointMarkers(); marker_tick_ = 0; }

    if (!last_local_pos_) return;

    switch (state_) {
      case State::WAIT_SIGNAL: handleWaitSignal(); break;
      case State::INIT:        handleInit();       break;
      case State::TAKEOFF:     handleTakeoff();    break;
      case State::MISSION:     handleMission();    break;
      default: break;
    }
  }

  /**************** 상태 처리 ****************/
  void handleWaitSignal()
  {
    publishHoldSetpoint();  // OFFBOARD 전에도 위치 고정
  }

  void handleInit()
  {
    if (offboard_counter_ < 20) {
      publishOffboardCtrlMode();
      publishHoldSetpoint();
      ++offboard_counter_;
      return;
    }

    // OFFBOARD 모드 & ARM
    sendVehicleCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    sendVehicleCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);

    start_local_pos_ = last_local_pos_;
    state_ = State::TAKEOFF;
    RCLCPP_INFO(get_logger(), "OFFBOARD/ARM 완료 → TAKEOFF");
  }

  void handleTakeoff()
  {
    float target_z = start_local_pos_->z - takeoff_alt_m_;   // NED Down (+)
    publishOffboardCtrlMode();
    publishSetpointNED(start_local_pos_->x, start_local_pos_->y, target_z);

    if (std::fabs(last_local_pos_->z - target_z) < 0.5f) {
      RCLCPP_INFO(get_logger(), "이륙 고도 %.1f m 도달", takeoff_alt_m_);
      if (!takeoff_done_pub_) {
        std_msgs::msg::Int32 flag; flag.data = 1;
        two_fb_pub_->publish(flag);           // UAV → UGV
        takeoff_done_pub_ = true;
      }
      state_ = State::MISSION;
    }
  }

  void handleMission()
  {
    auto enu_now_opt = currentENU();
    if (!enu_now_opt) return;
    const auto enu_now = *enu_now_opt;

    if (current_wp_idx_ >= enu_wps_.size()) {
      sendVehicleCmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
      state_ = State::LAND;
      RCLCPP_INFO(get_logger(), "모든 웨이포인트 완료 → LAND");
      return;
    }

    const auto tgt = enu_wps_[current_wp_idx_];
    float dE = tgt.e - enu_now.e,
          dN = tgt.n - enu_now.n,
          dU = tgt.u - enu_now.u;

    // 1 m 안팎이면 다음 웨이포인트
    if (std::hypot(dE, dN) < 1.0f && std::fabs(dU) < 1.0f) {
      ++current_wp_idx_;
      return;
    }

    // ENU → PX4 NED 좌표 변환
    float dX =  dN;
    float dY =  dE;
    float dZ = -dU;

    publishOffboardCtrlMode();
    publishSetpointNED(last_local_pos_->x + dX,
                       last_local_pos_->y + dY,
                       last_local_pos_->z + dZ);
  }

  /**************** Marker 퍼블리시 ****************/
  void publishWaypointMarkers()
  {
    visualization_msgs::msg::MarkerArray arr;
    rclcpp::Time stamp = now();

    for (size_t i = 0; i < enu_wps_.size(); ++i) {
      const auto& wp = enu_wps_[i];

      // 구체
      visualization_msgs::msg::Marker m;
      m.header.frame_id = map_frame_; m.header.stamp = stamp;
      m.ns = "wp"; m.id = static_cast<int>(i);
      m.type = m.SPHERE; m.action = m.ADD;
      m.pose.position.x = wp.e; m.pose.position.y = wp.n; m.pose.position.z = wp.u;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = m.scale.z = 1.0;
      m.color.r = 1.0; m.color.b = 1.0; m.color.a = 1.0;
      arr.markers.emplace_back(m);

      // 텍스트
      visualization_msgs::msg::Marker t = m;
      t.ns = "wp_text"; t.id = static_cast<int>(i + enu_wps_.size());
      t.type = t.TEXT_VIEW_FACING;
      t.pose.position.z += 1.0;
      t.scale.z = 0.7;
      t.text = "WayPoint " + std::to_string(i);
      arr.markers.emplace_back(t);
    }
    marker_pub_->publish(arr);
  }

  /**************** 공통 퍼블리시 ****************/
  void publishOffboardCtrlMode()
  {
    px4_msgs::msg::OffboardControlMode m{};
    m.timestamp = nowUSec();
    m.position  = true;
    offboard_ctrl_mode_pub_->publish(m);
  }

  void publishHoldSetpoint()
  {
    if (!last_local_pos_) return;
    publishSetpointNED(last_local_pos_->x,
                       last_local_pos_->y,
                       last_local_pos_->z);
  }

  void publishSetpointNED(float x, float y, float z)
  {
    px4_msgs::msg::TrajectorySetpoint sp{};
    sp.timestamp   = nowUSec();
    sp.position[0] = x;
    sp.position[1] = y;
    sp.position[2] = z;
    sp.yaw         = 0.0f;
    traj_setpoint_pub_->publish(sp);
  }

  void sendVehicleCmd(uint16_t cmd, float p1=0, float p2=0)
  {
    px4_msgs::msg::VehicleCommand vc{};
    vc.timestamp        = nowUSec();
    vc.target_system    = 1;
    vc.target_component = 1;
    vc.command          = cmd;
    vc.param1           = p1;
    vc.param2           = p2;
    vehicle_cmd_pub_->publish(vc);
  }

  uint64_t nowUSec()
  { return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000ULL); }

  /**************** 멤버 ****************/
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_ctrl_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr  traj_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr      vehicle_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr               two_fb_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr            one_fb_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_; tf2_ros::TransformListener tf_listener_;

  std::vector<ENU> enu_wps_;
  size_t current_wp_idx_;
  uint8_t offboard_counter_, marker_tick_;
  bool takeoff_done_pub_;
  std::optional<px4_msgs::msg::VehicleLocalPosition> last_local_pos_, start_local_pos_;
  bool start_signal_received_;
  State state_;
  double takeoff_alt_m_;
  std::string map_frame_, base_frame_;
};

/************************************************************
 * main
 ************************************************************/
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardWaypointMap>());
  rclcpp::shutdown();
  return 0;
}
