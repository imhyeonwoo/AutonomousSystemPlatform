from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vehicle_type', default_value='X1'),
        DeclareLaunchArgument('image_topic', default_value='/world/default/model/X1_asp/link/base_link/sensor/camera_front/image'),
        DeclareLaunchArgument('camera_info_topic', default_value='/world/default/model/X1_asp/link/base_link/sensor/camera_front/camera_info'),
        DeclareLaunchArgument('target_id_topic', default_value='/X1/target_id'),
        DeclareLaunchArgument('image_proc_topic', default_value='/X1/image_proc'),
        DeclareLaunchArgument('target_pose_topic', default_value='/X1/target_pose'),

        Node(
            package='multi_tracker',
            executable='multi_tracker_node',
            name='x1_aruco_detector',
            output='screen',
            parameters=[{
                'vehicle_type': LaunchConfiguration('vehicle_type'),
                'image_topic': LaunchConfiguration('image_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'target_id_topic': LaunchConfiguration('target_id_topic'),
                'image_proc_topic': LaunchConfiguration('image_proc_topic'),
                'target_pose_topic': LaunchConfiguration('target_pose_topic')
            }]
        )
    ])
