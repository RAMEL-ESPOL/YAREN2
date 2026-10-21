"""Todo el modo conversación: STT (Vosk) + chatbot + TTS (Piper)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue as PV


def generate_launch_description():
    args = [
        DeclareLaunchArgument("vosk_model", default_value="/home/roberto/robotis_ws/src/YAREN2/yaren_chat/models/STT/vosk-model-small-es-0.42"),
        DeclareLaunchArgument("piper_model", default_value="/home/roberto/robotis_ws/src/YAREN2/yaren_chat/models/TTS/es_MX-claude-high.onnx"),
        DeclareLaunchArgument("mic_device", default_value=""),
        DeclareLaunchArgument("speaker_device", default_value=""),
        DeclareLaunchArgument("gain", default_value="1.05"),
        DeclareLaunchArgument("length_scale", default_value="1"),
        DeclareLaunchArgument("backend", default_value="piper"),
        DeclareLaunchArgument("use_online", default_value="true"),
        DeclareLaunchArgument("online_topic", default_value="chat_online_in"),
    ]
    stt = Node(package="yaren_chatbot", executable="stt_node.py", name="stt_node", output="screen",
               parameters=[{"model_path": LC("vosk_model"), "device": LC("mic_device"),
                            "gain": PV(LC("gain"), value_type=float)}])
    tts = Node(package="yaren_chatbot", executable="tts_node.py", name="tts_node", output="screen",
               parameters=[{"backend": LC("backend"), "model_path": LC("piper_model"), "device": LC("speaker_device"),
                            "length_scale": PV(LC("length_scale"), value_type=float)}])
    chat = Node(package="yaren_chatbot", executable="chat_node.py", name="chat_node", output="screen",
                parameters=[{"use_online": PV(LC("use_online"), value_type=bool),
                             "online_topic": LC("online_topic")}])
    return LaunchDescription(args + [stt, chat, tts])
