#!/bin/bash

# 1. NavSat, cmd_vel, odometry 브리지 실행
gnome-terminal -- bash -c "ros2 run ros_gz_bridge parameter_bridge \
/world/default/model/X1_asp/link/base_link/sensor/navsat_sensor/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat \
/model/X1_asp/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
/model/X1/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry; exec bash"

# 2. topic_bridge.launch.py 실행
# 공용 컴퓨터에서는 경로 바꿔줘야 함
gnome-terminal -- bash -c "cd ~/workspace/AutonomousVehiclePlatform/ws_gazebo && \
source install/setup.bash && \
ros2 launch gazebo_env_setup topic_bridge.launch.py; exec bash"

# 3. 카메라 브리지 실행
gnome-terminal -- bash -c "ros2 run ros_gz_bridge parameter_bridge \
/world/default/model/X1_asp/link/base_link/sensor/camera_front/image@sensor_msgs/msg/Image@gz.msgs.Image \
/world/default/model/X1_asp/link/base_link/sensor/camera_front/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
/world/default/model/X1_asp/link/base_link/sensor/fisheye/image@sensor_msgs/msg/Image@gz.msgs.Image \
/world/default/model/X1_asp/link/base_link/sensor/fisheye/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image \
/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo; exec bash"

# 4. Micro XRCE Agent 실행
gnome-terminal -- bash -c "MicroXRCEAgent udp4 -p 8888; exec bash"

