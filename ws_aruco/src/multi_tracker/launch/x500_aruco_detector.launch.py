# ws_aruco/src/multi_tracker/launch/x500_aruco_detector.launch.py (Corrected)

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 'use_sim_time'을 런치 파일의 인자로 선언하여 유연성을 높입니다.
    # 이렇게 하면 나중에 실제 로봇에서 실행할 때 'use_sim_time:=false'로 쉽게 전환할 수 있습니다.
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # LaunchConfiguration을 사용하여 런치 인자 값을 가져옵니다.
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # 런치 인자들을 선언합니다.
        use_sim_time_arg,
        DeclareLaunchArgument('vehicle_type', default_value='x500'),
        DeclareLaunchArgument('image_topic', default_value='/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image'),
        DeclareLaunchArgument('camera_info_topic', default_value='/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info'),
        DeclareLaunchArgument('target_id_topic', default_value='/x500/target_id'),
        DeclareLaunchArgument('image_proc_topic', default_value='/x500/image_proc'),
        DeclareLaunchArgument('target_pose_topic', default_value='/x500/target_pose'),

        Node(
            package='multi_tracker',
            executable='multi_tracker_node',
            name='x500_aruco_detector',
            output='screen',
            parameters=[
                # =========================== FIX ===========================
                # 이 노드가 시뮬레이션 시간을 사용하도록 설정합니다.
                {'use_sim_time': use_sim_time},
                # ===========================================================
                {'vehicle_type': LaunchConfiguration('vehicle_type')},
                {'image_topic': LaunchConfiguration('image_topic')},
                {'camera_info_topic': LaunchConfiguration('camera_info_topic')},
                {'target_id_topic': LaunchConfiguration('target_id_topic')},
                {'image_proc_topic': LaunchConfiguration('image_proc_topic')},
                {'target_pose_topic': LaunchConfiguration('target_pose_topic')}
            ]
        ),
    ])