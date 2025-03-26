from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='crawling_robot_controller',
            executable='position_control',  # C++ 노드 실행 파일 이름
            name='position_control_node',
            output='screen'
        ),
        Node(
            package='crawling_sunghyuk',
            executable='gripper_control',   # Python으로 작성한 그리퍼 제어 노드 실행 파일 이름
            name='gripper_control_node',
            output='screen'
        )
    ])
