import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'yaren_description'
    urdf_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'yaren_description.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
        # Este nodo crea una ventana con barras deslizantes para mover los joints
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        )
    ])