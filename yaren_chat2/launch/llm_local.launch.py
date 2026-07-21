#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(
            package='yaren_chat2',
            executable='llm_local_lifecycle_node.py',
            name='llm_lifecycle_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
