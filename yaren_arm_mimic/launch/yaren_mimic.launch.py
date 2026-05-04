#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        Node(
            package='yaren_face_display',
            executable='face_screen',
            name='face_screen',
            output='screen'
        ),
        Node(
            package='yaren_arm_mimic',
            executable='csi_cam_pub.py',
            name='csi_cam_pub',
            output='screen'
        ),
        Node(
            package='yaren_arm_mimic',
            executable='yaren_controller',
            name='yaren_controller',
            output='screen'
        ),
        Node(
            package='yaren_arm_mimic',
            executable='body_points_detector.py',
            name='body_points_detector',
            output='screen'
        ),
        Node(
            package='yaren_arm_mimic',
            executable='body_tracker_node',
            name='body_tracker_node',
            output='screen'
        ),      
    ])
