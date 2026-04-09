from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    usb_port_arg = DeclareLaunchArgument('usb_port', default_value='/dev/ttyUSB0')
    dxl_baud_rate_arg = DeclareLaunchArgument('dxl_baud_rate', default_value='1000000')

    # Nodo de comunicación (u2d2_communication.py migrado)
    communication_node = Node(
        package='yaren_u2d2',
        executable='u2d2_communication.py',
        name='yaren_motor_communication',
        output='screen',
        parameters=[{
            'usb_port': LaunchConfiguration('usb_port'),
            'dxl_baud_rate': LaunchConfiguration('dxl_baud_rate'),
        }]
    )

    return LaunchDescription([
        usb_port_arg,
        dxl_baud_rate_arg,
        communication_node
    ])