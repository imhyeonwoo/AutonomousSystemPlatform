cmake_minimum_required(VERSION 3.8)
project(gazebo_env_setup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


include_directories(
  include
)

add_executable(pose_tf_broadcaster src/pose_tf_broadcaster.cpp)
ament_target_dependencies(pose_tf_broadcaster rclcpp geometry_msgs tf2_ros)

install(TARGETS
  pose_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


ament_package()
