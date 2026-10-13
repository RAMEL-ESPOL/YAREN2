from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("threshold", default_value="0.45"),
        DeclareLaunchArgument("use_online", default_value="true"),
        DeclareLaunchArgument("in_topic", default_value="stt_text"),
        DeclareLaunchArgument("out_topic", default_value="tts_text"),
        DeclareLaunchArgument("online_topic", default_value="chat_online_in"),
        DeclareLaunchArgument("start_topic", default_value="chat_start"),
        DeclareLaunchArgument("modo_conversacion", default_value="true"),
        DeclareLaunchArgument("session_timeout", default_value="180.0"),
        Node(
            package="yaren_chatbot",
            executable="chat_node.py",
            name="chat_node",
            output="screen",
            parameters=[{
                "threshold": ParameterValue(LaunchConfiguration("threshold"), value_type=float),
                "use_online": ParameterValue(LaunchConfiguration("use_online"), value_type=bool),
                "in_topic": LaunchConfiguration("in_topic"),
                "out_topic": LaunchConfiguration("out_topic"),
                "online_topic": LaunchConfiguration("online_topic"),
                "start_topic": LaunchConfiguration("start_topic"),
                "modo_conversacion": ParameterValue(LaunchConfiguration("modo_conversacion"), value_type=bool),
                "session_timeout": ParameterValue(LaunchConfiguration("session_timeout"), value_type=float),
            }],
        ),
    ])
