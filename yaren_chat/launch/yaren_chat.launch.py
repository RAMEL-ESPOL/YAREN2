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
        llm_node,    # 1. El cerebro arranca primero
        face_screen, # La cara arranca de inmediato
        
        # 2. El TTS arranca 2s después del LLM
        RegisterEventHandler(
            OnProcessStart(
                target_action=llm_node,
                on_start=[TimerAction(period=2.0, actions=[tts_node])]
            )
        ),
        
        # 3. El STT arranca 5s después del LLM (tiempo suficiente para inicializar Vosk)
        RegisterEventHandler(
            OnProcessStart(
                target_action=llm_node,
                on_start=[TimerAction(period=10.0, actions=[stt_node])]
            )
        ),
        
        # 4. El controlador arranca SOLO cuando el STT (el más lento) ha iniciado
        RegisterEventHandler(
            OnProcessStart(
                target_action=stt_node,
                on_start=[TimerAction(period=15.0, actions=[control_manager_node])]
            )
        ),
    ])
