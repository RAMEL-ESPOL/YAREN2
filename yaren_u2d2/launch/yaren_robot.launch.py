#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # 1. Configuración de rutas
    # Asegúrate de que estos nombres de paquetes coincidan con tus carpetas en /src
    pkg_u2d2 = get_package_share_directory('yaren_u2d2')
    pkg_description = get_package_share_directory('yaren_description')
    
    urdf_file = os.path.join(pkg_description, 'urdf', 'yaren_description.urdf')
    controller_config = os.path.join(pkg_u2d2, 'config', 'yaren_controllers.yaml')

    with open(urdf_file, "r") as infp:
        robot_description_content = infp.read()

    # 2. Robot State Publisher (Siempre arranca primero)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # 3. Nodo principal de Control
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description_content}, controller_config],
        output='screen',
    )
    
    # 4. Spawners (Retrasados 2 segundos para dar tiempo al controller_manager)
    spawner_joint_state = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    
    spawner_trajectory = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen",
    )

    # 5. Nodo de la Cara (Paquete corregido)
    face_screen_node = Node(
        package='yaren_face_display', # Nombre corregido
        executable='face_hablar_cambiar',
        name='face_screen_node',
        output='screen'
    )
    movements_node = Node(
        package='yaren_movements',
        executable='yaren_fullmovement', # Asegúrate de que tenga permisos de ejecución
        name='yaren_movements_node',
        output='screen'
    )


    # Creamos el temporizador solo para los spawners
    delayed_spawners = TimerAction(
        period=2.0,
        actions=[spawner_joint_state, spawner_trajectory]
    )

    return LaunchDescription([
        rsp,
        ros2_control_node,
        delayed_spawners,
        face_screen_node,
        movements_node
    ])