#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nodes = []

    # Robot 1 - DOMAIN 1
    for i in range(1, 5):
        nodes.append(Node(
            package='test_lifecycle',
            executable=f'node{i}',
            name=f'robot1_node{i}',
            output='screen',
            additional_env={'ROS_DOMAIN_ID': '1'}
        ))

    # Robot 2 - DOMAIN 2
    for i in range(1, 5):
        nodes.append(Node(
            package='test_lifecycle',
            executable=f'node{i}',
            name=f'robot2_node{i}',
            output='screen',
            additional_env={'ROS_DOMAIN_ID': '2'}
        ))

    # Robot 3 - DOMAIN 3
    for i in range(1, 5):
        nodes.append(Node(
            package='test_lifecycle',
            executable=f'node{i}',
            name=f'robot3_node{i}',
            output='screen',
            additional_env={'ROS_DOMAIN_ID': '3'}
        ))

    # GCS - DOMAIN 1 (receives from Robot 1 only)
    nodes.append(Node(
        package='test_lifecycle',
        executable='gcs',
        name='gcs',
        output='screen',
        additional_env={'ROS_DOMAIN_ID': '0'}
    ))

    return LaunchDescription(nodes)
