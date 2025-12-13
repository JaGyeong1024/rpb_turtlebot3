from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    # 1. TurtleBot3 로봇 구동 launch
    turtlebot3_bringup_dir = get_package_share_directory('turtlebot3_bringup')
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_bringup_dir, 'launch', 'robot.launch.py')
        )
    )
    
    # 2. 카메라 이미지 발행 launch는 비활성화 (백 파일에서 카메라 사용)
    # image_pub_launch는 제외
    
    # 3. 카메라 뷰어 노드 (이미지 뒤집기)
    # 백 파일의 /image_raw를 구독하여 /camera/image_flipped 발행
    camera_viewer_node = Node(
        package='rover_line_follower',
        executable='camera_viewer',
        output='screen',
        remappings=[
            ('/image_raw', '/image_raw'),  # 백 파일의 /image_raw 구독
        ]
    )
    
    # 4. 라인 추종 노드 (20초 지연 후 시작)
    line_follower_node = Node(
        package='rover_line_follower',
        executable='line_follower_node',
        output='screen',
        parameters=[
            {
                'camera_topic': 'camera/image_flipped',
                'cmd_vel_topic': 'cmd_vel',
            }
        ],
    )
    
    delayed_line_follower = TimerAction(period=20.0, actions=[line_follower_node])
    
    return LaunchDescription([
        robot_launch,           # TurtleBot3 하드웨어 구동
        # image_pub_launch 제외 (백 파일에서 카메라 사용)
        camera_viewer_node,    # 카메라 이미지 뒤집기 (백 파일의 /image_raw 사용)
        delayed_line_follower  # 라인 추종 (20초 후 시작)
    ])



