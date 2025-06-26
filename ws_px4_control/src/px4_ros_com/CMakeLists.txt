cmake_minimum_required(VERSION 3.5)
project(px4_ros_com)

# ────────── LANG STD ──────────
set(CMAKE_C_STANDARD  99)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ────────── DEPENDENCIES ──────────
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy  REQUIRED)

find_package(px4_msgs          REQUIRED)
find_package(geometry_msgs     REQUIRED)
find_package(sensor_msgs       REQUIRED)
find_package(nav_msgs          REQUIRED)
find_package(std_msgs          REQUIRED)          # ★ 추가
find_package(tf2_ros           REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge         REQUIRED)
find_package(visualization_msgs REQUIRED)


# OpenCV(aruco)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco)

include_directories(include)

############################
# ▒▒  Libraries / Targets ▒▒
############################
# ── (1) lib: frame_transforms ──────────────────────────────
add_library(frame_transforms SHARED src/lib/frame_transforms.cpp)
ament_target_dependencies(frame_transforms
  Eigen3 geometry_msgs sensor_msgs)
target_include_directories(frame_transforms PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# ── (2) 예제 리스너 3종 ───────────────────────────────────
add_executable(sensor_combined_listener src/examples/listeners/sensor_combined_listener.cpp)
ament_target_dependencies(sensor_combined_listener rclcpp px4_msgs)

add_executable(vehicle_gps_position_listener src/examples/listeners/vehicle_gps_position_listener.cpp)
ament_target_dependencies(vehicle_gps_position_listener rclcpp px4_msgs)

add_executable(debug_vect_advertiser src/examples/advertisers/debug_vect_advertiser.cpp)
ament_target_dependencies(debug_vect_advertiser rclcpp px4_msgs)

# ── (3) offboard_control (이미지·aruco) ───────────────────
add_executable(offboard_control src/examples/offboard/offboard_control.cpp)
ament_target_dependencies(offboard_control
  rclcpp px4_msgs
  tf2_ros tf2_geometry_msgs
  geometry_msgs sensor_msgs std_msgs
  cv_bridge)
target_include_directories(offboard_control PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(offboard_control ${OpenCV_LIBRARIES})

# ── (4) offboard_control_srv ──────────────────────────────
add_executable(offboard_control_srv src/examples/offboard/offboard_control_srv.cpp)
ament_target_dependencies(offboard_control_srv
  rclcpp px4_msgs
  tf2_ros tf2_geometry_msgs
  geometry_msgs sensor_msgs std_msgs
  cv_bridge)
target_include_directories(offboard_control_srv PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(offboard_control_srv ${OpenCV_LIBRARIES})

# ── (5) offboard_waypoint (기존 단순 WP 버전) ─────────────
# add_executable(offboard_waypoint src/examples/offboard/offboard_waypoint.cpp)
# ament_target_dependencies(offboard_waypoint rclcpp px4_msgs geometry_msgs)

# ── (6) offboard_waypoint_map (ENU→NED 변환) ──────
add_executable(offboard_waypoint_map src/examples/offboard/offboard_waypoint_map.cpp)
ament_target_dependencies(offboard_waypoint_map
  rclcpp px4_msgs tf2_ros tf2_geometry_msgs geometry_msgs std_msgs nav_msgs visualization_msgs)

# ── (7) offboard_waypoint_trigger (이동 명령) ──────
add_executable(offboard_waypoint_trigger src/examples/offboard/offboard_waypoint_trigger.cpp)
ament_target_dependencies(offboard_waypoint_trigger
  rclcpp px4_msgs tf2_ros tf2_geometry_msgs geometry_msgs
  std_msgs nav_msgs visualization_msgs ament_index_cpp)

# ── (8) gimbal_target_publisher (gimbal 타겟 발행) ──────
add_executable(gimbal_target_publisher src/examples/offboard/gimbal_target_publisher.cpp)
ament_target_dependencies(gimbal_target_publisher
  rclcpp px4_msgs visualization_msgs std_msgs
  tf2_ros tf2_geometry_msgs px4_msgs)

# ── (9) offboard_waypoint_map_landing (정밀 착륙 버전) ── ★ 추가
add_executable(offboard_waypoint_map_landing
src/examples/offboard/offboard_waypoint_map_landing.cpp)
ament_target_dependencies(offboard_waypoint_map_landing
rclcpp px4_msgs tf2_ros tf2_geometry_msgs geometry_msgs
std_msgs nav_msgs visualization_msgs)


# ── (10) offboard_waypoint_map_speed  ── ★ 추가
add_executable(offboard_waypoint_map_speed
src/examples/offboard/offboard_waypoint_map_speed.cpp)
ament_target_dependencies(offboard_waypoint_map_speed
rclcpp px4_msgs tf2_ros tf2_geometry_msgs geometry_msgs
std_msgs nav_msgs visualization_msgs)

#################################
# ▒▒  install / export section ▒▒
#################################
install(TARGETS
  frame_transforms
  sensor_combined_listener
  vehicle_gps_position_listener
  debug_vect_advertiser
  offboard_control
  offboard_control_srv
  # offboard_waypoint
  offboard_waypoint_map
  offboard_waypoint_trigger
  gimbal_target_publisher
  offboard_waypoint_map_landing
  offboard_waypoint_map_speed
  EXPORT export_frame_transforms
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY test   DESTINATION share/${PROJECT_NAME}/)
install(FILES test1.csv test2.csv DESTINATION share/${PROJECT_NAME})

# CSV · 기타 정적 자원 설치
install(DIRECTORY data DESTINATION share/${PROJECT_NAME})   # ★ 추가

# ──────────────────── Python 노드 모음 ────────────────────
install(PROGRAMS 
  src/examples/offboard/path_planner_node.py
  src/examples/map_visualization/publish_map_info.py
  DESTINATION lib/${PROJECT_NAME})


ament_export_dependencies(
  ament_cmake rclcpp rosidl_default_runtime
  eigen3_cmake_module Eigen3
  px4_msgs geometry_msgs sensor_msgs std_msgs
  tf2_ros tf2_geometry_msgs cv_bridge OpenCV)

ament_export_include_directories(include)
ament_export_targets(export_frame_transforms HAS_LIBRARY_TARGET)

ament_package()
