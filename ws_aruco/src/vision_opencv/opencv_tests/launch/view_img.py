from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # image_tools 패키지의 showimage 실행
        ExecuteProcess(
            cmd=['ros2', 'run', 'image_tools', 'showimage', '-t', '/opencv_tests/images'],
            name='showimage',
            output='screen'
        ),

        # opencv_tests 패키지의 source.py 실행
        ExecuteProcess(
            cmd=['ros2', 'run', 'opencv_tests', 'source.py'],
            name='source',
            output='screen'
        )
    ])
