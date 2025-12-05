#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # GCS node in DOMAIN 0
        Node(
            package='test_lifecycle',
            executable='gcs',
            name='gcs',
            output='screen',
            additional_env={'ROS_DOMAIN_ID': '0'}
        ),
    ])
