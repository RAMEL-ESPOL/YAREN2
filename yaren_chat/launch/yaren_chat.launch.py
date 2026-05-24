from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart

def generate_launch_description():

    # 🔹 GESTOR DE IDIOMA — arranca primero, publica /yaren/current_language
    gestor_idioma_node = Node(
        name='language_manager',
        package='yaren_idioma',
        executable='gestor_idioma',
        output='screen',
    )

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

    return LaunchDescription([
        gestor_idioma_node,

        RegisterEventHandler(
            OnProcessStart(
                target_action=gestor_idioma_node,
                on_start=[TimerAction(period=0.5, actions=[llm_node])]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=llm_node,
                on_start=[TimerAction(period=1.0, actions=[tts_node])]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=llm_node,
                on_start=[TimerAction(period=3.0, actions=[stt_node])]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=stt_node,
                on_start=[TimerAction(period=5.0, actions=[control_manager_node])]
            )
        ),
    ])