#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():

    csi_cam = Node(
    package='yaren_dice',
    executable='csi_cam_pub.py',
    name='csi_cam_pub',
    output='screen'
    )
    
    
    pose_detector_node = Node(
        package='yaren_dice',
        executable='pose_detector',
        name='pose_detector',
        output='screen'
    )

    body_landmarks = Node(
        package='yaren_dice',
        executable='body_landmarks.py',
        name='body_landmarks',
        output='screen'
    )

    speaker_node = Node(
        package='yaren_dice',
        executable='speaker_node.py',
        name='speaker_node',
        output='screen'
    )

    game_manager_node = Node(
        package='yaren_dice',
        executable='game_manager',
        name='game_manager',
        output='screen'
    )

    face_screen = Node(
        package="yaren_face_display",
        executable="face_screen",
        name="face_screen",
        output="screen",
    )
    '''
    face_screen = Node(
    	package="yaren_face_display",
    	executable="face_screen",
    	name="face_screen",
    	output="screen",
    )
'''
    # return LaunchDescription([
    #     # Arranca la cámara y el speaker apenas inicie el launch
    #     csi_cam,
    #     speaker_node,

    #     # Cuando speaker esté arriba → arranca game_manager
    #     RegisterEventHandler(
    #         OnProcessStart(
    #             target_action=speaker_node,
    #             on_start=[TimerAction(period=20.0, actions=[game_manager_node])]
    #         )
    #     ),

    #     # Cuando cámara esté arriba → arranca body_landmarks
    #     RegisterEventHandler(
    #         OnProcessStart(
    #             target_action=csi_cam,
    #             on_start=[TimerAction(period=3.0, actions=[body_landmarks])]
    #         )
    #     ),

    #     # Cuando body_landmarks esté arriba → arranca pose_detector
    #     RegisterEventHandler(
    #         OnProcessStart(
    #             target_action=body_landmarks,
    #             on_start=[TimerAction(period=3.0, actions=[pose_detector_node])]
    #         )
    #     ),
    # ])

    return LaunchDescription([
        csi_cam,
        face_screen,
        body_landmarks,
        RegisterEventHandler(
            OnProcessStart(
                target_action=body_landmarks,
                on_start=[
                    TimerAction(
                        period=3.0,
                        actions=[pose_detector_node]
                    )
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=pose_detector_node,
                on_start=[
                    TimerAction(
                        period=3.0,
                        actions=[speaker_node]
                    )
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=speaker_node,
                on_start=[
                    TimerAction(
                        period=3.0,
                        actions=[game_manager_node]
                    )
                ]
            )
        ),
    ])

