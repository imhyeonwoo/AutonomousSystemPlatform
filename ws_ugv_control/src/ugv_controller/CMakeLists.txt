cmake_minimum_required(VERSION 3.8)
project(ugv_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(path_follower_node src/path_follower_node.cpp)
ament_target_dependencies(path_follower_node
  rclcpp sensor_msgs geometry_msgs tf2 tf2_geometry_msgs visualization_msgs nav_msgs)

install(TARGETS path_follower_node
  DESTINATION lib/${PROJECT_NAME})

# ★ CSV 등이 들어있는 data 폴더 설치
install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME})

# (launch 파일이 있으면 유지)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
