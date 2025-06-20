from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # TF 및 clock bridge
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

        # X1 카메라 (image + camera_info)
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/world/default/model/X1_asp/link/base_link/sensor/camera_front/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/world/default/model/X1_asp/link/base_link/sensor/camera_front/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
            ],
            output='screen'
        ),

        # X1 LiDAR (point cloud)
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/world/default/model/X1_asp/link/base_link/sensor/gpu_lidar/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
            ],
            output='screen'
        ),

        # x500 카메라 (image + camera_info)
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
            ],
            output='screen'
        )
    ])
