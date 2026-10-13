from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("threshold", default_value="0.45"),
        DeclareLaunchArgument("use_online", default_value="true"),
        DeclareLaunchArgument("in_topic", default_value="stt_text"),
        DeclareLaunchArgument("out_topic", default_value="tts_text"),
        DeclareLaunchArgument("online_topic", default_value="chat_online_in"),
        Node(
            package="yaren_chatbot",
            executable="chat_node.py",
            name="chat_node",
            output="screen",
            parameters=[{
                "threshold": LaunchConfiguration("threshold"),
                "use_online": LaunchConfiguration("use_online"),
                "in_topic": LaunchConfiguration("in_topic"),
                "out_topic": LaunchConfiguration("out_topic"),
                "online_topic": LaunchConfiguration("online_topic"),
            }],
        ),
    ])
