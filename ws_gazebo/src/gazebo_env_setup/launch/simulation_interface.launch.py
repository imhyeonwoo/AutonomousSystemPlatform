# ~/workspace/AutonomousVehiclePlatform/ws_gazebo/src/gazebo_env_setup/launch/simulation_bringup.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # 1. 기존 topic_bridge.launch.py 포함하기
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_env_setup'), 'launch', 'topic_bridge.launch.py')
        )
    )

    # 2. ros_gz_bridge 노드들
    nav_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/default/model/X1_asp/link/base_link/sensor/navsat_sensor/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
            '/model/X1_asp/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/X1/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ],
        output='screen'
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/default/model/X1_asp/link/base_link/sensor/camera_front/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/world/default/model/X1_asp/link/base_link/sensor/camera_front/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/world/default/model/X1_asp/link/base_link/sensor/fisheye/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/world/default/model/X1_asp/link/base_link/sensor/fisheye/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    # 3. MicroXRCEAgent 실행 (ROS 노드가 아니므로 ExecuteProcess 사용)
    microxrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )

    return LaunchDescription([
        included_launch,
        nav_bridge,
        camera_bridge,
        microxrce_agent
    ])