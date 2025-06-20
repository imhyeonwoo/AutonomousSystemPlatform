from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    offboard_control_node = Node(
        package='px4_ros_com',
        executable='offboard_control',
        name='offboard_control',
        output='screen',
        # use_sim_time 파라미터를 True로 설정합니다.
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        offboard_control_node
    ])
