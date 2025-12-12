import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    prefix = DeclareLaunchArgument("prefix", default_value="")

    # my_rover.urdf.xacro는 prefix 인자를 지원하지 않으므로 xacro 명령에 전달하지 않음
    # frame_prefix는 robot_state_publisher 파라미터로만 사용 (빈 문자열이면 무시됨)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'ignore_timestamp': False,
            'frame_prefix': LaunchConfiguration('prefix'),
            'robot_description':
                Command([
                    'xacro ',
                    PathJoinSubstitution([
                        FindPackageShare('simulation'),
                        'urdf',
                        'my_rover.urdf.xacro',
                    ]),
                ]),
        }]
    )

    ld.add_action(prefix)
    ld.add_action(rsp_node)

    return ld

