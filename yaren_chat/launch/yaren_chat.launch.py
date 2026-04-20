from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart

def generate_launch_description():

    stt_node = LifecycleNode(
        name='stt_lifecycle_node',
        namespace='',
        package='yaren_chat',
        executable='stt_lifecycle_node.py',
        output='screen',
    )

    llm_node = LifecycleNode(
        name='llm_lifecycle_node',
        namespace='',
        package='yaren_chat',
        executable='llm_lifecycle_node.py',
        output='screen',
    )

    tts_node = LifecycleNode(
        name='tts_lifecycle_node',
        namespace='',
        package='yaren_chat',
        executable='tts_lifecycle_node.py',
        output='screen',
    )

    control_manager_node = Node(
        name='lifecycle_control_manager',
        package='yaren_chat',
        executable='control_manager_node',
        output='screen'
    )
    
    face_screen = Node(
        package='yaren_face_display',
        executable='face_screen',
        name='face_screen',
        output='screen'
    )

    return LaunchDescription([
        llm_node,
        face_screen,
        RegisterEventHandler(
            OnProcessStart(
                target_action=llm_node,
                on_start=[
                    TimerAction(
                        period=1.0,
                        actions=[stt_node]
                    )
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=llm_node,
                on_start=[
                    TimerAction(
                        period=1.0,
                        actions=[tts_node]
                    )
                ]
            ),
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=tts_node,
                on_start=[
                    TimerAction(
                        period=1.0,
                        actions=[control_manager_node]
                    )
                ]
            )
        ),
    ])
