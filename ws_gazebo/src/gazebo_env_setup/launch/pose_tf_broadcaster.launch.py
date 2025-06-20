from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # -------------- Gazebo → ROS TF 브로드캐스터 --------------
        Node(
            package='gazebo_env_setup',
            executable='pose_tf_broadcaster',
            name='pose_tf_broadcaster',
            output='screen'
        ),

        # -------------- Static TF (센서) --------------
        # base_link → camera_front
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '0.43', '0.0', '0.26',           # xyz
                '0', '0', '0',                   # rpy (deg)
                'X1_asp/base_link', 'X1_asp/base_link/camera_front'
            ],
            output='screen'
        ),
        # base_link → gpu_lidar
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '0.60', '0.0', '0.13',
                '0', '0', '0',
                'X1_asp/base_link', 'X1_asp/base_link/gpu_lidar'
            ],
            output='screen'
        ),

        # -------------- **새로운** Static TF --------------
        # map(ENU) → local_ned(NED)  :  x↔y 스왑, z 부호 반전
        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
        #         # translation
        #         '0', '0', '0',
        #         # quaternion  (x, y, z, w) = (0.7071, 0.7071, 0, 0)
        #         '0.7071068', '0.7071068', '0', '0',
        #         'map', 'local_ned'
        #     ],
        #     output='screen'
        # ),
    ])
