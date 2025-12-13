from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    # 1. 카메라 이미지 발행 launch
    image_pub_dir = get_package_share_directory('image_pub')
    image_pub_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(image_pub_dir, 'launch', 'raw_image_pub.launch.py')
        )
    )
    
    # 2. 카메라 뷰어 노드 (이미지 뒤집기)
    camera_viewer_node = Node(
        package='rover_line_follower',
        executable='camera_viewer',
        output='screen',
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
        image_pub_launch,      # 카메라 이미지 발행 (/image_raw)
        camera_viewer_node,    # 카메라 이미지 뒤집기 (/camera/image_flipped)
        tune_bev_node,         # BEV 튜닝
    ])



