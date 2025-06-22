from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_ros_com',
            executable='publish_map_info.py',
            name='map_info_publisher',
            output='screen'
        ),
        Node(
            package='px4_ros_com',
            executable='path_planner_node.py',
            name='path_planner',
            output='screen'
        ),
        Node(
            package='px4_ros_com',
            executable='gimbal_target_publisher',
            name='gimbal_publisher',
            output='screen'
        ),
    ])