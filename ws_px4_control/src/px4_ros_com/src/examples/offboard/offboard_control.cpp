// /****************************************************************************
//  *
//  * Copyright 2020 PX4 Development Team. All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions are met:
//  *
//  * 1. Redistributions of source code must retain the above copyright notice, this
//  * list of conditions and the following disclaimer.
//  *
//  * 2. Redistributions in binary form must reproduce the above copyright notice,
//  * this list of conditions and the following disclaimer in the documentation
//  * and/or other materials provided with the distribution.
//  *
//  * 3. Neither the name of the copyright holder nor the names of its contributors
//  * may be used to endorse or promote products derived from this software without
//  * specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  * POSSIBILITY OF SUCH DAMAGE.
//  *
//  ****************************************************************************/

// /**
//  * @brief Offboard control example
//  * @file offboard_control.cpp
//  * @addtogroup examples
//  * @author Mickey Cowden <info@cowden.tech>
//  * @author Nuno Marques <nuno.marques@dronesolutions.io>
//  */

// #include <px4_msgs/msg/offboard_control_mode.hpp>
// #include <px4_msgs/msg/trajectory_setpoint.hpp>
// #include <px4_msgs/msg/vehicle_command.hpp>
// #include <px4_msgs/msg/vehicle_control_mode.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <stdint.h>

// #include <chrono>
// #include <iostream>

// using namespace std::chrono;
// using namespace std::chrono_literals;
// using namespace px4_msgs::msg;

// class OffboardControl : public rclcpp::Node
// {
// public:
// 	OffboardControl() : Node("offboard_control")
// 	{

// 		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
// 		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
// 		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

// 		offboard_setpoint_counter_ = 0;

// 		auto timer_callback = [this]() -> void {

// 			if (offboard_setpoint_counter_ == 10) {
// 				// Change to Offboard mode after 10 setpoints
// 				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

// 				// Arm the vehicle
// 				this->arm();
// 			}

// 			// offboard_control_mode needs to be paired with trajectory_setpoint
// 			publish_offboard_control_mode();
// 			publish_trajectory_setpoint();

// 			// stop the counter after reaching 11
// 			if (offboard_setpoint_counter_ < 11) {
// 				offboard_setpoint_counter_++;
// 			}
// 		};
// 		timer_ = this->create_wall_timer(100ms, timer_callback);
// 	}

// 	void arm();
// 	void disarm();

// private:
// 	rclcpp::TimerBase::SharedPtr timer_;

// 	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
// 	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
// 	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

// 	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

// 	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

// 	void publish_offboard_control_mode();
// 	void publish_trajectory_setpoint();
// 	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
// };

// /**
//  * @brief Send a command to Arm the vehicle
//  */
// void OffboardControl::arm()
// {
// 	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

// 	RCLCPP_INFO(this->get_logger(), "Arm command send");
// }

// /**
//  * @brief Send a command to Disarm the vehicle
//  */
// void OffboardControl::disarm()
// {
// 	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

// 	RCLCPP_INFO(this->get_logger(), "Disarm command send");
// }

// /**
//  * @brief Publish the offboard control mode.
//  *        For this example, only position and altitude controls are active.
//  */
// void OffboardControl::publish_offboard_control_mode()
// {
// 	OffboardControlMode msg{};
// 	msg.position = true;
// 	msg.velocity = false;
// 	msg.acceleration = false;
// 	msg.attitude = false;
// 	msg.body_rate = false;
// 	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
// 	offboard_control_mode_publisher_->publish(msg);
// }

// /**
//  * @brief Publish a trajectory setpoint
//  *        For this example, it sends a trajectory setpoint to make the
//  *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
//  */
// void OffboardControl::publish_trajectory_setpoint()
// {
// 	TrajectorySetpoint msg{};
// 	msg.position = {0.0, 0.0, -5.0};
// 	msg.yaw = -3.14; // [-PI:PI]
// 	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
// 	trajectory_setpoint_publisher_->publish(msg);
// }

// /**
//  * @brief Publish vehicle commands
//  * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
//  * @param param1    Command parameter 1
//  * @param param2    Command parameter 2
//  */
// void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
// {
// 	VehicleCommand msg{};
// 	msg.param1 = param1;
// 	msg.param2 = param2;
// 	msg.command = command;
// 	msg.target_system = 1;
// 	msg.target_component = 1;
// 	msg.source_system = 1;
// 	msg.source_component = 1;
// 	msg.from_external = true;
// 	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
// 	vehicle_command_publisher_->publish(msg);
// }

// int main(int argc, char *argv[])
// {
// 	std::cout << "Starting offboard control node..." << std::endl;
// 	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
// 	rclcpp::init(argc, argv);
// 	rclcpp::spin(std::make_shared<OffboardControl>());

// 	rclcpp::shutdown();
// 	return 0;
// }
// // ✅ [1] offboard_control_node.cpp
// // 위치 제어나 속도 제어를 외부 명령에 따라 전환하며 동작


#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
// TF2와 관련된 헤더 파일
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <rclcpp/time.hpp>
#include <chrono>
#include <cmath>
#include <limits>
#include <fstream>
#include <sstream>
#include <vector>
#include <tuple>
#include <algorithm>
#include <numeric>
#include <iomanip> // std::setprecision 사용을 위해 추가
#include <cstring> // for std::memcpy
// 기존 헤더 아래에 추가
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
// 기존 헤더 아래에 추가
#include <geometry_msgs/msg/point_stamped.hpp>
// 기존 헤더 아래에 추가
#include <sstream>
#include <iomanip>
#include <cstdlib>     // for getenv
#include <filesystem>  // for std::filesystem::path (경로를 쉽게 합치기 위함)
// 기존 헤더 파일들 아래에 추가
#include <tf2/LinearMath/Quaternion.h>

// ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼ 이 부분 추가 ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // ★★★ 이 줄을 추가하세요 ★★★
// ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲ 여기까지 추가 ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

#include <geometry_msgs/msg/transform.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;

// PX4 메시지 타입 별칭
using OffboardControlMode = px4_msgs::msg::OffboardControlMode;
using TrajectorySetpoint  = px4_msgs::msg::TrajectorySetpoint;
using VehicleCommand      = px4_msgs::msg::VehicleCommand;

// ─── 트럭 하차 위치(맵 좌표) ─────────────────────────────
constexpr double ACTIVATE_X       = -135.027;
constexpr double ACTIVATE_Y       =   62.169;
constexpr double ACTIVATE_RADIUS  =    1.0;   // [m] 허용 오차
// ───────────────────────────────────────────────────────

/* -----  take-off 준비 상태 플래그  ------------------------------------ */
bool camera_tilt_sent_{false};   ///< 짐벌 -90° 명령을 한 번만 보내기 위한 플래그
bool wp_loaded_{false};          ///< CSV 읽기 + ENU→NED 변환이 끝났는지
bool route_planned_{false};      ///< compute_tsp_route() 완료 여부
bool first_wp_sent_{false};        // ★ PATCH-BEGIN : 첫 WP setpoint 발행 여부
/* --------------------------------------------------------------------- */


/* --- 첫 번째 WP 로컬 좌표 캐시 --- */
float first_wp_x_{0.0f};      // ★ NEW
float first_wp_y_{0.0f};      // ★ NEW


enum MissionState {
    WAIT_FOR_POSE = 0,
    HOLD_ON_TRUCK,
    INIT,
    TAKEOFF,
    GOTO_WAYPOINT,
    SEARCH_FOR_MARKER,      // ✨ 이름 변경
    DONE
};

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control"), step_(0), mission_state_(WAIT_FOR_POSE), current_wp_idx_(0)
    {
        // 퍼블리셔 설정
        offb_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        traj_pub_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        cmd_pub_  = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        // TF2 버퍼와 리스너, 브로드캐스터 초기화
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // ArUco 및 이미지 관련 초기화
        aruco_dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        aruco_detector_params_ = cv::aruco::DetectorParameters::create();
        auto sub_qos = rclcpp::QoS(1).best_effort();
        auto pub_qos = rclcpp::QoS(10).reliable();
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image", sub_qos,
            std::bind(&OffboardControl::image_callback, this, std::placeholders::_1));
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info", sub_qos,
            std::bind(&OffboardControl::camera_info_callback, this, std::placeholders::_1));
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/offboard_control/image_proc", pub_qos);

        // 타이머 로직을 생성자 내부의 람다 함수로 통합
        auto timer_callback = [this]() -> void {

            // 새로운 로직: 최초 1회 px4_origin 설정
            if (!origin_offset_set_) {
                try {
                    // 드론의 시작 위치와 방향을 map 기준으로 조회
                    auto t_map_drone_initial = tf_buffer_->lookupTransform("map", "x500_gimbal_0", tf2::TimePointZero);
                    
                    // 1. px4_origin의 위치는 드론의 시작 위치로 고정
                    origin_offset_.translation = t_map_drone_initial.transform.translation;

                    // 2. px4_origin의 방향은 map(ENU) -> NED로 변환하는 고정된 회전값을 사용
                    tf2::Quaternion q_enu_to_ned;
                    q_enu_to_ned.setRPY(3.14159, 0, 1.57079); // Roll=PI, Pitch=0, Yaw=PI/2
                    origin_offset_.rotation = tf2::toMsg(q_enu_to_ned);

                    origin_offset_set_ = true;
                    RCLCPP_INFO(get_logger(), "px4_origin이 드론 시작점에 고정되었습니다.");

                } catch (const tf2::TransformException & ex) {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "초기 TF 변환 대기 중... %s", ex.what());
                    return; // 아직 TF가 준비 안됐으면 콜백 종료
                }
            }

            // 설정된 origin_offset_을 계속해서 브로드캐스트
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.frame_id  = "map";
            tf_msg.child_frame_id   = "px4_origin";
            tf_msg.header.stamp     = this->get_clock()->now();
            tf_msg.transform        = origin_offset_;
            tf_broadcaster_->sendTransform(tf_msg);


            // 기존 미션 로직
            update_drone_position();

            switch (mission_state_)
            {
                case WAIT_FOR_POSE: // HOLD_ON_TRUCK과 동일하게 동작
                case HOLD_ON_TRUCK:
                    // update_drone_position() 내부에서 도착 여부를 체크
                    break;
                case INIT:
                    if (step_ <= 20) {
                        publish_offboard_mode();
                        traj_pub_->publish(make_sp(cur_x_, cur_y_, cur_z_, NANF));
                        if (step_ == 10) {
                            send_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
                            send_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
                            RCLCPP_INFO(get_logger(), "[INIT] Offboard + Arm 명령 전송");
                        }
                        ++step_;
                    } else {
                        publish_offboard_mode();
                        traj_pub_->publish(make_sp(cur_x_, cur_y_, cur_z_, NANF));

                        // 이륙 전 경로 계획을 수행
                        if (!wp_loaded_ && load_waypoints()) {
                            transform_and_adjust_waypoints();
                            wp_loaded_ = true;
                            RCLCPP_INFO(get_logger(), "[INIT] 웨이포인트 로드 및 변환 완료 (%zu개)", waypoints_.size());
                        }
                        if (wp_loaded_ && !route_planned_) {
                            planned_route_  = compute_tsp_route();
                            current_wp_idx_ = 0; // 첫 번째 웨이포인트부터 시작
                            route_planned_  = true;
                        }

                        // 경로 계획이 완료되었으면 이륙 상태로 전환
                        if (route_planned_) {
                            mission_state_ = TAKEOFF;
                            step_ = 0;
                            RCLCPP_INFO(get_logger(), "[INIT] 경로 계획 완료 → TAKEOFF");
                        }
                    }
            break;
                
                case TAKEOFF:
                {
                    publish_offboard_mode();
                    
                    // 현재 위치에서 수직으로만 이륙하도록 명령
                    traj_pub_->publish(make_sp(cur_x_, cur_y_, TAKEOFF_ALTITUDE, NANF));
                    
                   // ★★★ 수정된 짐벌 틸트 로직 ★★★
                    // TAKEOFF 상태에 진입한 직후, 2초 동안(step_ < 20) 틸트 명령을 반복 전송합니다.
                    if (!camera_tilt_sent_) {
                        if (step_ < 10) {
                            send_gimbal_tilt(-90.0f);
                            step_++;
                        } else {
                            // 2초가 지나면 틸팅이 완료되었다고 간주하고 플래그를 설정합니다.
                            camera_tilt_sent_ = true;
                            RCLCPP_INFO(get_logger(), "[TAKEOFF] 짐벌 틸팅 완료");
                        }
                    }
        

                    // 목표 고도에 도달하면 GOTO_WAYPOINT 상태로 전환
                    if (std::abs(cur_z_ - TAKEOFF_ALTITUDE) < 1.0) {
                        RCLCPP_INFO(get_logger(), "[TAKEOFF] 목표 고도 도달 → GOTO_WAYPOINT");
                        mission_state_ = GOTO_WAYPOINT;
                    }
                    break;
                }
                
                case GOTO_WAYPOINT:
                {
                    publish_offboard_mode();
                    if (current_wp_idx_ >= planned_route_.size()) {
                        mission_state_ = DONE;
                        RCLCPP_INFO(get_logger(), "모든 웨이포인트 완료 → DONE");
                        break;
                    }
                    size_t target_idx = planned_route_[current_wp_idx_];
                    const auto [gx, gy, gz] = waypoints_[target_idx];
                    geometry_msgs::msg::PointStamped wp_map, wp_local;
                    wp_map.header.frame_id = "map";
                    //wp_map.header.stamp = this->now();
                    wp_map.header.stamp = rclcpp::Time(0);
                    wp_map.point.x = gy;
                    wp_map.point.y = gx;
                    wp_map.point.z = -(gz + 10.0f);
                    try {
                        wp_local = tf_buffer_->transform(wp_map, "px4_origin");
                    } catch (const tf2::TransformException& e) {
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "WP 변환 실패: %s", e.what());
                        break;
                    }
                    traj_pub_->publish(make_sp(wp_local.point.x, wp_local.point.y, wp_local.point.z, M_PI_2));
                    should_process_image_ = false;
                    const double dist_xy  = std::hypot(cur_x_ - wp_local.point.x, cur_y_ - wp_local.point.y);
                    if (dist_xy < 1.5) {
                        RCLCPP_INFO(get_logger(), "WP #%zu 도착 → SEARCH_FOR_MARKER", target_idx);
                        mission_state_ = SEARCH_FOR_MARKER;
                    }
                    break;
                }

                case SEARCH_FOR_MARKER:
                {
                    publish_offboard_mode();
                    // 현재 위치에서 호버링
                    traj_pub_->publish(make_sp(cur_x_, cur_y_, cur_z_, NANF));
                    
                    // 이미지 처리 활성화하여 마커 탐색
                    should_process_image_ = true;

                    // 마커가 발견되면 다음 WP로 이동
                    if (marker_detected_) {
                        RCLCPP_INFO(get_logger(), "마커 확인! → 다음 웨이포인트로 이동");
                        marker_detected_ = false;
                        should_process_image_ = false;
                        current_wp_idx_++; // 다음 웨이포인트로 인덱스 증가
                        mission_state_ = GOTO_WAYPOINT;
                    }
                    break;
                }

                case DONE:
                    send_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0f);
                    break;
                default:
                    break;
            }
        };
        // Wall Timer 대신 rclcpp::Timer 사용을 시도했으나 컴파일 오류 발생, Wall Timer로 되돌림
        // 이는 환경 문제일 수 있으며, Wall Timer로도 내부 로직의 시간은 sim_time을 따르므로 동작에 문제 없음
        timer_ = this->create_wall_timer(100ms, timer_callback);

        RCLCPP_INFO(get_logger(), "[OffboardControl] 노드 시작. 초기 위치 대기 중...");
    }

private:
    // 멤버 함수 선언
    void update_drone_position();
    void publish_offboard_mode();
    TrajectorySetpoint make_sp(float x, float y, float z, float yaw);
    void send_vehicle_command(uint16_t command, float param1, float param2 = 0.0f);
    void send_gimbal_tilt(float pitch_deg);
    bool load_waypoints();
    std::vector<size_t> compute_tsp_route();
    void transform_and_adjust_waypoints();
    void set_px4_param_via_command(const std::string& param_id, float value);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void annotate_image(cv_bridge::CvImagePtr image);

    // ROS 관련 멤버
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offb_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr  traj_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr      cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    // 상태 및 데이터 멤버
    int step_;
    const float NANF = std::numeric_limits<float>::quiet_NaN();
    const float TAKEOFF_ALTITUDE = -15.0f;
    MissionState mission_state_;
    std::vector<std::tuple<float, float, float>> waypoints_;
    std::vector<size_t> planned_route_;
    size_t current_wp_idx_;
    float cur_x_{0}, cur_y_{0}, cur_z_{0};
    geometry_msgs::msg::Transform origin_offset_;
    bool origin_offset_set_{false};
    bool should_process_image_{false};
    cv::Ptr<cv::aruco::Dictionary> aruco_dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_detector_params_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    double marker_size_{0.5};
    cv::Vec3d marker_relative_position_;
    bool marker_detected_{false};
    bool camera_tilt_sent_{false};
};

void OffboardControl::update_drone_position()
{
    // origin이 설정된 후에만 좌표를 조회하도록 보장
    if (!origin_offset_set_) {
        return;
    }
    
    try {
        auto t_map_drone = tf_buffer_->lookupTransform("map", "x500_gimbal_0", tf2::TimePointZero);
        auto t_local_drone = tf_buffer_->lookupTransform("px4_origin", "x500_gimbal_0", tf2::TimePointZero);

        cur_x_ = t_local_drone.transform.translation.x;
        cur_y_ = t_local_drone.transform.translation.y;
        cur_z_ = t_local_drone.transform.translation.z;

        RCLCPP_INFO(this->get_logger(),
                    "Coords | Global(map): [x: %7.2f, y: %7.2f, z: %7.2f] | Local(ned): [x: %7.2f, y: %7.2f, z: %7.2f]",
                    t_map_drone.transform.translation.x,
                    t_map_drone.transform.translation.y,
                    t_map_drone.transform.translation.z,
                    cur_x_,
                    cur_y_,
                    cur_z_);

        if (mission_state_ == WAIT_FOR_POSE) {
            mission_state_ = HOLD_ON_TRUCK;           // 첫 호출 시 단-한-번 전이
            RCLCPP_INFO(get_logger(), "→ HOLD_ON_TRUCK (대기 시작)");
        }

        if (mission_state_ == HOLD_ON_TRUCK) {
            const double dist_xy =
                std::hypot(t_map_drone.transform.translation.x - ACTIVATE_X,
                           t_map_drone.transform.translation.y - ACTIVATE_Y);

            if (dist_xy < ACTIVATE_RADIUS) {
                RCLCPP_INFO(get_logger(), "트럭 하차 지점 도착! (%.2f m)  →  INIT 단계 진입", dist_xy);
                mission_state_ = INIT;
                step_ = 0;
            }
        }
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF 변환 대기 중... %s", ex.what());
    }
}

void OffboardControl::publish_offboard_mode()
{
    OffboardControlMode msg;
    msg.timestamp = now().nanoseconds() / 1000;
    msg.position = true;
    offb_pub_->publish(msg);
}



/**
 * @brief 100ms 주기로 호출되는 메인 콜백 함수. 미션 상태에 따라 적절한 동작을 수행합니다.
 */
/**
 * @brief 100ms 주기로 호출되는 메인 콜백 함수. 미션 상태에 따라 적절한 동작을 수행합니다.
 */

/**
 * @brief OffboardControlMode 메시지를 퍼블리시합니다. 위치 제어를 활성화합니다.
 */

/**
 * @brief TrajectorySetpoint 메시지를 생성합니다.
 */
TrajectorySetpoint OffboardControl::make_sp(float x, float y, float z, float yaw)
{
    TrajectorySetpoint sp;
    sp.timestamp = now().nanoseconds() / 1000;
    sp.position[0] = x;
    sp.position[1] = y;
    sp.position[2] = z;
    sp.yaw = yaw;
    sp.yawspeed = NANF;
    for (int i = 0; i < 3; ++i) {
      sp.velocity[i] = NANF;
      sp.acceleration[i] = NANF;
      sp.jerk[i] = NANF;
    }
    return sp;
}

/**
 * @brief VehicleCommand 메시지를 퍼블리시합니다. (ARM, Disarm, Land 등)
 */
void OffboardControl::send_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg;
    msg.timestamp = now().nanoseconds() / 1000;
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    cmd_pub_->publish(msg);
}

void OffboardControl::send_gimbal_tilt(float pitch_deg)
{
    VehicleCommand msg{};
    msg.timestamp = now().nanoseconds() / 1000;
    msg.command   = VehicleCommand::VEHICLE_CMD_DO_MOUNT_CONTROL;  // 205
    msg.param1    = pitch_deg;   // Pitch
    msg.param2    = 0.0f;        // Roll
    msg.param3    = 0.0f;        // Yaw
    msg.param7    = 2.0f;        // MAV_MOUNT_MODE_MAVLINK_TARGETING
    msg.target_system    = 1;
    msg.target_component = 1;
    msg.source_system    = 1;
    msg.source_component = 1;
    msg.from_external    = true;
    cmd_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "짐벌을 %.1f°로 틸트 명령", pitch_deg);
}


// ✨ PX4 파라미터를 VehicleCommand를 통해 설정하는 새로운 함수
void OffboardControl::set_px4_param_via_command(const std::string& param_id, float value)
{
    VehicleCommand msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.command = VehicleCommand::VEHICLE_CMD_DO_SET_PARAMETER;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.from_external = true;

    // param1: 설정할 값
    msg.param1 = value;
    // param2: 비워둠 (0)
    msg.param2 = 0.0f;
    // param3, param4: 파라미터 ID (이름)을 4바이트씩 나누어 전송
    // 이 방식은 PX4에서만 동작할 수 있습니다.
    if (param_id.length() <= 16) { // PX4 파라미터 ID는 최대 16바이트
        char param_id_c[17];
        std::strncpy(param_id_c, param_id.c_str(), sizeof(param_id_c) - 1);
        param_id_c[sizeof(param_id_c) - 1] = '\0'; // Null-terminate

        // param_id를 정수처럼 다루어 param3, param4에 복사
        std::memcpy(&msg.param3, param_id_c, sizeof(msg.param3));
        std::memcpy(&msg.param4, param_id_c + sizeof(msg.param3), sizeof(msg.param4));

    } else {
        RCLCPP_ERROR(this->get_logger(), "파라미터 ID '%s'가 너무 깁니다 (최대 16자).", param_id.c_str());
        return;
    }
    
    cmd_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "파라미터 '%s'를 %.2f로 변경하는 명령을 전송했습니다.", param_id.c_str(), value);
}


/**
 * @brief 카메라 정보를 수신하여 내부 변수(camera_matrix_, dist_coeffs_)를 설정합니다.
 */
void OffboardControl::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (!camera_matrix_.empty()) return;
    camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
    dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();
    RCLCPP_INFO(this->get_logger(), "카메라 정보 수신 완료.");
}

/**
 * @brief 이미지를 수신하여 ArUco 마커를 탐지하고, 상대 위치를 계산하여 저장하며,
 * 결과 이미지를 다시 퍼블리시합니다.
 */
// OffboardControl::image_callback() 함수 전체를 이 내용으로 교체하세요.

void OffboardControl::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (!should_process_image_ || camera_matrix_.empty() || dist_coeffs_.empty()) {
        marker_detected_ = false;
        return;
    }

    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(cv_ptr->image, aruco_dictionary_, corners, ids, aruco_detector_params_);

        marker_detected_ = false; 

        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

            for (size_t i = 0; i < ids.size(); ++i) {
                cv::drawFrameAxes(cv_ptr->image, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], marker_size_ / 2.0f);
                
                marker_relative_position_ = tvecs[i];
                marker_detected_ = true;

                // ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
                // ★★★              수정된 로직: 변환 후 저장                     ★★★
                // ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
                
                geometry_msgs::msg::PoseStamped pose_in_map; // 변환 결과를 저장할 변수

                // 1. TF를 이용해 먼저 글로벌 'map' 좌표계로 변환
                try {
                    geometry_msgs::msg::PoseStamped pose_in_camera;
                    //pose_in_camera.header.stamp = this->get_clock()->now();
                    pose_in_camera.header.stamp = rclcpp::Time(0);
                    pose_in_camera.header.frame_id = "x500_gimbal_0/camera_link";
                    
                    pose_in_camera.pose.position.x = tvecs[i][0];
                    pose_in_camera.pose.position.y = tvecs[i][1];
                    pose_in_camera.pose.position.z = tvecs[i][2];
                    
                    const auto& rvec = rvecs[i];
                    double angle = cv::norm(rvec);
                    cv::Vec3d axis = (angle > 1e-6) ? rvec / angle : cv::Vec3d(0, 0, 1);
                    tf2::Quaternion q;
                    q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);
                    pose_in_camera.pose.orientation = tf2::toMsg(q);

                    pose_in_map = tf_buffer_->transform(pose_in_camera, "map");

                    RCLCPP_INFO(this->get_logger(),
                                "[ARUCO GLOBAL] Marker ID: %d, Global Pos (map): [x: %.3f, y: %.3f, z: %.3f]",
                                ids[i],
                                pose_in_map.pose.position.x,
                                pose_in_map.pose.position.y,
                                pose_in_map.pose.position.z);

                } catch(const tf2::TransformException &ex) {
                    RCLCPP_WARN(this->get_logger(), "ArUco 마커 좌표 변환 실패: %s", ex.what());
                    continue; // 변환에 실패하면 이 마커는 건너뛰고 다음 마커를 처리
                }
                
                // 2. 변환된 글로벌 좌표를 CSV 파일에 저장
                try {
                    const char* home_dir = getenv("HOME");
                    if (home_dir != nullptr) {
                        std::filesystem::path dir_path = std::filesystem::path(home_dir) / "workspace";
                        if (!std::filesystem::exists(dir_path)) {
                            std::filesystem::create_directory(dir_path);
                        }
                        std::filesystem::path file_path = dir_path / "test2.csv";
                        std::ofstream outfile;
                        outfile.open(file_path, std::ios_base::app);

                        if (outfile.is_open()) {
                            // tvecs, q 대신 변환된 pose_in_map의 값을 사용
                            outfile << std::fixed << std::setprecision(6)
                                    << ids[i] << ","
                                    << pose_in_map.pose.position.x << ","
                                    << pose_in_map.pose.position.y << ","
                                    << pose_in_map.pose.position.z << ","
                                    << pose_in_map.pose.orientation.x << ","
                                    << pose_in_map.pose.orientation.y << ","
                                    << pose_in_map.pose.orientation.z << ","
                                    << pose_in_map.pose.orientation.w << "\n";
                            outfile.close();
                            
                            RCLCPP_INFO(this->get_logger(), 
                                "[CSV SAVED] Marker ID: %d, Global Pos: (%.3f, %.3f, %.3f)",
                                ids[i], 
                                pose_in_map.pose.position.x,
                                pose_in_map.pose.position.y,
                                pose_in_map.pose.position.z);
                        }
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "CSV 파일 쓰기 중 예외 발생: %s", e.what());
                }
                
                // 첫 번째 마커만 처리하고 반복 중단
                break; 
            }
        }
        
        annotate_image(cv_ptr);
        image_pub_->publish(*cv_ptr->toImageMsg());

    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}
/**
 * @brief 이미지에 마커의 상대 좌표 텍스트를 추가합니다.
 */
void OffboardControl::annotate_image(cv_bridge::CvImagePtr image)
{
    // 마커가 탐지되었을 때만 텍스트를 표시합니다.
    if (!marker_detected_) {
        return;
    }

    // marker_relative_position_ 변수의 값을 문자열로 변환합니다.
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2);
    stream << "X:" << marker_relative_position_[0] 
           << " Y:" << marker_relative_position_[1] 
           << " Z:" << marker_relative_position_[2];
    std::string text_xyz = stream.str();

    // 이미지 좌상단에 텍스트를 씁니다.
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.8;
    int thickness = 2;
    cv::Point textOrg(10, 30);
    cv::putText(image->image, text_xyz, textOrg, fontFace, fontScale, cv::Scalar(0, 255, 255), thickness);
}

/**
 * @brief CSV 파일들로부터 웨이포인트를 읽어옵니다.
 */




bool OffboardControl::load_waypoints()
{
    // 1. HOME 환경 변수로부터 홈 디렉토리 경로를 가져옵니다.
    const char* home_dir = getenv("HOME");
    if (home_dir == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "HOME 환경 변수를 찾을 수 없습니다.");
        return false;
    }

    // 2. std::filesystem::path를 사0:12: warning: enumeration value ‘SET_ORIGIN’ not handled in switch [-Wswitch]
    std::filesystem::path base_path(home_dir);
    std::filesystem::path wp_path = base_path / "worksapce/uav_wp.csv";
    std::filesystem::path goal_path = base_path / "worksapce/rendezvous.csv";

    RCLCPP_INFO(this->get_logger(), "웨이포인트 파일 경로: %s", wp_path.c_str());
    RCLCPP_INFO(this->get_logger(), "최종 목적지 파일 경로: %s", goal_path.c_str());

    waypoints_.clear();
    std::string line;

    // 3. 수정된 경로로 경유지 파일을 엽니다.
    std::ifstream fin(wp_path);
    if (!fin) {
        RCLCPP_ERROR(get_logger(), "'%s' 파일을 열 수 없습니다.", wp_path.c_str());
        return false;
    }
    while (std::getline(fin, line)) {
        std::stringstream ss(line);
        float x, y, z;
        char c;
        if ((ss >> x >> c >> y >> c >> z)) {
            waypoints_.emplace_back(x, y, z);
        }
    }
    fin.close();

    // 4. 수정된 경로로 최종 목적지 파일을 엽니다.
    std::ifstream goalfin(goal_path);
    if (!goalfin) {
        RCLCPP_ERROR(get_logger(), "'%s' 파일을 열 수 없습니다.", goal_path.c_str());
        return false;
    }
    if (std::getline(goalfin, line)) {
        std::stringstream ss(line);
        float x, y, z;
        char c;
        if ((ss >> x >> c >> y >> c >> z)) {
            waypoints_.emplace_back(x, y, z);
        }
    } else {
        RCLCPP_ERROR(get_logger(), "rendezvous.csv 파일이 비어있습니다.");
        goalfin.close();
        return false;
    }
    goalfin.close();

    return !waypoints_.empty();
}
/**
 * @brief Brute-force 방식으로 모든 경로 순열을 탐색하여 최단 거리를 찾습니다.
 * 마지막 웨이포인트는 최종 목적지로 고정됩니다.
 */
std::vector<size_t> OffboardControl::compute_tsp_route()
{

    RCLCPP_INFO(this->get_logger(), "사용자 지정 경로를 생성합니다.");

    // ✨ 1. 방문하고 싶은 경유지 순서(인덱스 0~5)를 직접 지정합니다.
    //    예시: 0 -> 2 -> 1 -> 4 -> 3 -> 5 순서로 방문
    std::vector<size_t> intermediate_route = {0, 5, 1, 4, 3, 2};

    // 2. 경로 마지막에 최종 목적지 인덱스를 추가합니다.
    //    (최종 목적지는 항상 waypoints_ 벡터의 마지막에 있음)
    if (!waypoints_.empty()) {
        size_t final_destination_idx = waypoints_.size() - 1;
        intermediate_route.push_back(final_destination_idx);
    }

    return intermediate_route;

}

  /**
   * @brief waypoints_ 멤버의 좌표계를 ENU에서 NED로 변환하고, 고도를 10m씩 높입니다.
   */
  void OffboardControl::transform_and_adjust_waypoints() {
      std::vector<std::tuple<float, float, float>> transformed_wps;
      const float altitude_increase = -10.0f; // NED에서 위로 10m

      for (const auto& wp : waypoints_) {
          // ENU 좌표 추출
          float enu_x = std::get<0>(wp);
          float enu_y = std::get<1>(wp);
          float enu_z = std::get<2>(wp);

          // 1. 표준 ENU -> NED 회전 변환
          // NED_X = ENU_Y, NED_Y = ENU_X, NED_Z = -ENU_Z
          float ned_x = enu_y;
          float ned_y = enu_x;
          float ned_z = -enu_z;
          
            // ★★★ 수정된 부분 ★★★
            // 2. 이륙 고도(TAKEOFF_ALTITUDE)만큼 고도를 추가합니다.
          ned_z += altitude_increase;

          transformed_wps.emplace_back(ned_x, ned_y, ned_z);
      }
      // 변환된 웨이포인트로 기존 목록을 교체
      waypoints_ = transformed_wps;
  }



int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}