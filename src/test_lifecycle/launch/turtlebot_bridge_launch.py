#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('test_lifecycle')
    
    # Domain bridge node for turtlebot
    # 터틀봇에서 domain 8로 실행되어 domain 18로 배터리 토픽 브리징
    bridge_node = Node(
        package='domain_bridge',
        executable='domain_bridge',
        name='turtlebot_bridge',
        output='screen',
        additional_env={'ROS_DOMAIN_ID': '8'},
        arguments=[
            os.path.join(pkg_share, 'config', 'domain_bridge_turtlebot.yaml')
        ]
    )
    
    return LaunchDescription([bridge_node])

