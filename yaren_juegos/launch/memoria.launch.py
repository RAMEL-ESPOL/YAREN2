from launch import LaunchDescription
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(
            package='yaren_juegos',
            executable='memoria_node',
            name='memoria_node',
            namespace='',
            output='screen',
        ),
    ])
