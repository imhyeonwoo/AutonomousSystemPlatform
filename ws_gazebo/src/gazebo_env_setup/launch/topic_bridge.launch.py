# ~/workspace/AutonomousVehiclePlatform/ws_gazebo/src/gazebo_env_setup/launch/topic_bridge.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # TF 및 clock bridge ----------------------------------------------
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/model/X1_asp/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/model/X1_asp/pose_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/model/x500_gimbal_0/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/model/x500_gimbal_0/pose_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
            ],
            output='screen'
        ),

        # X1 LiDAR (point cloud) -----------------------------------------
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/world/default/model/X1_asp/link/base_link/sensor/gpu_lidar/scan/points'
                '@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
            ],
            output='screen'
        ),
    ])
