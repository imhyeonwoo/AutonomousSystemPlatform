cmake_minimum_required(VERSION 3.8)
project(multi_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 설치된 경로로 설정
set(HOME_DIR $ENV{HOME})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# include 디렉토리 설정
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 실행 파일 추가
# -------------------(1) multi_tracker_node-------------------
add_executable(multi_tracker_node src/multi_tracker_node.cpp)
ament_target_dependencies(multi_tracker_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  OpenCV
  cv_bridge
  Eigen3
  image_transport
  tf2
  tf2_geometry_msgs
)



# 런치 설정
install(TARGETS multi_tracker_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
