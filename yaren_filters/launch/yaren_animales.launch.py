from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 2. Abre el detector de IA en segundo plano
        Node(
            package='yaren_filters', 
            executable='face_landmark_detector.py', 
            name='detector',
            output='screen',
            emulate_tty=True
        ),
        
        # 3. Abre el filtro en segundo plano
        Node(
            package='yaren_filters', 
            executable='animal_filter_mask', 
            name='filtro_animales',
            output='screen',
            emulate_tty=True
        )
    ])
