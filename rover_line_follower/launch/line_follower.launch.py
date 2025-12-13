from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # camera_viewer 노드: 카메라 이미지 180도 뒤집기
    camera_viewer_node = Node(
        package='rover_line_follower',
        executable='camera_viewer',
        output='screen',
    )
    
    # line_follower_node: 라인 추종 제어 노드
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
    
    delayed_line_follower = TimerAction(period=5.0, actions=[line_follower_node])
    
    return LaunchDescription([
        camera_viewer_node,
        delayed_line_follower
    ])
