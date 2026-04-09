from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='Nivel de logging (debug, info, warn, error)'
        ),
        Node(
            package='ai_face_software',
            executable='final_face',
            name='final_face',
            output='screen',
            arguments=['--ros-args', '--log-level',
                       LaunchConfiguration('log_level')],
        ),
    ])