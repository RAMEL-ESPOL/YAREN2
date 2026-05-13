from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Abre la cámara en segundo plano
        Node(
            package='yaren_filters', 
            executable='csi_cam_pub.py', 
            name='camara',
            output='screen',
            emulate_tty=True
        ),
        
        # 2. Abre el detector de IA en segundo plano
        Node(
            package='yaren_filters', 
            executable='fondo_virtual.py', 
            name='fondo',
            output='screen',
            emulate_tty=True
        ),
    ])
