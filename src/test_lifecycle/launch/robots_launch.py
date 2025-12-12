#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def nodes_in_robot(namespace, domain_id):
    nodes = []

    # Get package share directory
    pkg_share = get_package_share_directory('test_lifecycle')

    # Robot nodes (lifecycle nodes)
    for i in range(1, 5):
        nodes.append(Node(
            package='test_lifecycle',
            executable=f'node{i}',
            name=f'node{i}',
            namespace=namespace,
            output='screen',
            additional_env={'ROS_DOMAIN_ID': f'{domain_id}'}
        ))

    # Lifecycle Manager for this robot
    nodes.append(Node(
        package='test_lifecycle',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        namespace=namespace,
        output='screen',
        additional_env={'ROS_DOMAIN_ID': f'{domain_id}'},
        parameters=[{
            'managed_nodes': ['node1', 'node2', 'node3', 'node4']
        }]
    ))

    # Domain bridge
    nodes.append(Node(
        package='domain_bridge',
        executable='domain_bridge',
        name='bridge_node',
        namespace=namespace,
        output='screen',
        additional_env={'ROS_DOMAIN_ID': f'{domain_id}'},
        arguments=[
                   os.path.join(pkg_share, 'config', f'domain_bridge_robot{domain_id}.yaml')
                ]
    ))

    return nodes

def generate_launch_description():
    nodes = []

    # Get package share directory
    pkg_share = get_package_share_directory('test_lifecycle')

    nodes += nodes_in_robot('robot1', 1)
    nodes += nodes_in_robot('robot2', 2)
    nodes += nodes_in_robot('robot3', 3)

    return LaunchDescription(nodes)
