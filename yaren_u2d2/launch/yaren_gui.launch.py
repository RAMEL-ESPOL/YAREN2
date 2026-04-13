from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Argumentos para el puerto y baudrate
    usb_port_arg = DeclareLaunchArgument('usb_port', default_value='/dev/ttyUSB0')
    dxl_baud_rate_arg = DeclareLaunchArgument('dxl_baud_rate', default_value='1000000')

    # Nodo de la interfaz gráfica (w_datos_ros2.py)
    gui_node = Node(
        package='yaren_u2d2',
        executable='w_datos_ros2.py',
        name='yaren_motor_gui',
        output='screen',
        parameters=[{
            'usb_port': LaunchConfiguration('usb_port'),
            'dxl_baud_rate': LaunchConfiguration('dxl_baud_rate'),
        }]
    )

    return LaunchDescription([
        usb_port_arg,
        dxl_baud_rate_arg,
        gui_node
    ])