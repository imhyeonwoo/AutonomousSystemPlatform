from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    path_file_arg = DeclareLaunchArgument(
        'path_file',
        default_value='path_s_curve.csv',
        description='Full path to CSV file with waypoints')
    longitudinal_speed_arg = DeclareLaunchArgument(
        'longitudinal_speed',
        default_value='2.0',
        description='Max longitudinal speed for the vehicle')

    # Path follower node
    path_follower_node = Node(
        package='ugv_controller',
        executable='path_follower_node',
        name='path_follower',
        output='screen',
        parameters=[{
            'path_file': LaunchConfiguration('path_file'),
            'longitudinal_speed': LaunchConfiguration('longitudinal_speed')
        }]
    )

    return LaunchDescription([
        path_file_arg,
        longitudinal_speed_arg,
        path_follower_node
    ])