from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Abre la cámara en su propia ventana de Terminator
        Node(
            package='yaren_filters', 
            executable='csi_cam_pub.py', 
            name='camara',
            
            prefix='terminator -u -x ' # <-- por terminator -x 
        ),
        
        # 2. Abre el detector de IA
        Node(
            package='yaren_filters', 
            executable='face_landmark_detector.py', 
            name='detector',
            prefix='terminator -u  -x '
        ),
        
        # 3. Abre el filtro 
        Node(
            package='yaren_filters', 
            executable='animal_filter_mask', 
            name='filtro_animales',
            prefix='terminator -u -x '
        )
    ])