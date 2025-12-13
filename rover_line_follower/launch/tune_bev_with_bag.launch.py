from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    # 1. 카메라 이미지 발행 launch는 비활성화 (백 파일에서 카메라 사용)
    # image_pub_launch는 제외
    
    # 2. 카메라 뷰어 노드 (이미지 뒤집기)
    # 백 파일의 /image_raw를 구독하여 /camera/image_flipped 발행
    camera_viewer_node = Node(
        package='rover_line_follower',
        executable='camera_viewer',
        output='screen',
        remappings=[
            ('/image_raw', '/image_raw'),  # 백 파일의 /image_raw 구독
        ]
    )
    
    # 3. BEV 튜닝 노드
    tune_bev_node = Node(
        package='rover_line_follower',
        executable='tune_bev',
        output='screen',
        parameters=[
            {
                'camera_topic': 'camera/image_flipped',  # camera_viewer가 발행하는 토픽 사용
            }
        ],
    )
    
    return LaunchDescription([
        # image_pub_launch 제외 (백 파일에서 카메라 사용)
        camera_viewer_node,    # 카메라 이미지 뒤집기 (백 파일의 /image_raw 사용)
        tune_bev_node,         # BEV 튜닝
    ])



