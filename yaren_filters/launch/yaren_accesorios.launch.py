from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, SetEnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        # 1. Evitar que Python se ahogue con los mensajes de texto
        SetEnvironmentVariable('PYTHONUNBUFFERED', '1'),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        
        # 2. Nodo de la cámara
        Node(
            package='yaren_filters', 
            executable='csi_cam_pub.py', 
            name='camara',
            output='screen',
            emulate_tty=True # Mágico para evitar bloqueos
        ),
        
        # 3. Nodo de IA (espera 2 segundos)
        TimerAction(period=2.0, actions=[
            Node(
                package='yaren_filters', 
                executable='face_landmark_detector.py', 
                name='detector',
                output='screen',
                emulate_tty=True
            )
        ]),
        
        # 4. Nodo de Filtros (espera 4 segundos)
        TimerAction(period=4.0, actions=[
            Node(
                package='yaren_filters', 
                executable='face_filter_node', 
                name='filtro_accesorios',
                output='screen',
                emulate_tty=True
            )
        ])
    ])