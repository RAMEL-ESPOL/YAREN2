from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Abre la cámara en su propia ventana de Terminator
        Node(
            package='yaren_emotions', 
            executable='csi_cam_pub.py', 
            name='camara',
            
            #prefix='terminator -u -x ' # <-- por terminator -x 
        ),
        
        # 2. Abre el detector de IA
        Node(
            package='yaren_emotions', 
            executable='detect_emotion.py', 
            name='detector',
            #prefix='terminator -u  -x '
        ),
    
    ])