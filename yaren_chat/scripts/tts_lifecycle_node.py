#!/usr/bin/env python3
import sys
import os
_node_dir = os.path.dirname(os.path.realpath(__file__))
if _node_dir not in sys.path:
    sys.path.insert(0, _node_dir)

import rclpy
from rclpy.action import ActionClient
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn 
from std_msgs.msg import Bool
from yaren_interfaces.msg import PersonResponse
from yaren_interfaces.action import ProcessResponse
from ament_index_python.packages import get_package_share_directory
from piper import PiperVoice
from piper.config import SynthesisConfig
from playsound import playsound
import os
import threading
import tempfile
import wave
import time 
from action_msgs.msg import GoalStatus
class TTSLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('tts_lifecycle_node')
        
        pkg_share_dir = get_package_share_directory('yaren_chat')
        self.tts_model_path = os.path.join(pkg_share_dir, 'models', 'TTS', 'es_MX-claude-high.onnx')
        self.tts_config_path = os.path.join(pkg_share_dir, 'models', 'TTS', 'es_MX-claude-high.onnx.json')
        
        self.voice = None

        self.voice = PiperVoice.load(
                model_path=self.tts_model_path,
                config_path=self.tts_config_path,
                use_cuda=True
            )
        
        self.stt_status_publisher = self.create_publisher(Bool, '/stt_terminado', 10)
        self.audio_playing_publisher = self.create_publisher(Bool, '/audio_playing', 10)

        self.create_subscription(PersonResponse, '/response_person', self.process_input_person, 10)
        self.text_person = None
        
        self._action_client = None
    
    def process_input_person(self, msg):
        """Process input from person response topic"""
        self.text_person = msg.text
    
    def on_configure(self, state):
        self.get_logger().info('Configuring TTS Node')
        
        try:
            # self.voice = PiperVoice.load(
            #     model_path=self.tts_model_path,
            #     config_path=self.tts_config_path,
            #     use_cuda=True
            # )
            
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            return TransitionCallbackReturn.FAILURE
    
    def on_activate(self, state):
        self.get_logger().info('Activating TTS Node')
        
        self._action_client = ActionClient(self, ProcessResponse, '/response_llama')
        
        goal_thread = threading.Thread(target=self._listen_and_speak)
        goal_thread.start()
        
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state):
        self.get_logger().info('Deactivating TTS Node')
        return TransitionCallbackReturn.SUCCESS
    
    def _listen_and_speak(self):
        """Listen for LLAMA responses and convert to speech"""
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            return
        
        if self.text_person is None:
            return
        
        goal_msg = ProcessResponse.Goal()
        goal_msg.input_text = self.text_person

        self._action_client.wait_for_server()
        
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )

        self.get_logger().info("Waiting for TTS goal to complete...")
        while rclpy.ok() and not future.done():
            time.sleep(0.1) # Espera activa sin bloquear el ejecutor global

        if future.done():
            result = future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("TTS finished successfully.")
            else:
                self.get_logger().warn(f"TTS failed with status: {result.status}")
    def _play_audio(self, text_to_speak):
        audio_status_msg = Bool()
        audio_status_msg.data = True
        self.audio_playing_publisher.publish(audio_status_msg)

        syn_config = SynthesisConfig(
            length_scale=1.2,
            noise_scale=0.5,
            noise_w_scale=0.8
        )

        with tempfile.NamedTemporaryFile(suffix=".wav", delete=True) as fp:
            with wave.open(fp.name, 'wb') as wav_file:
                wav_file.setnchannels(1)
                wav_file.setsampwidth(2)
                wav_file.setframerate(self.voice.config.sample_rate)
                self.voice.synthesize_wav(text_to_speak, wav_file, syn_config=syn_config)
            
            playsound(fp.name)

            audio_status_msg.data = False
            self.audio_playing_publisher.publish(audio_status_msg)
    
    def _feedback_callback(self, feedback_msg):
        """Process feedback and convert text to speech"""
        chunk = feedback_msg.feedback.current_chunk
        is_last_chunk = feedback_msg.feedback.is_last_chunk
        
        if chunk and not is_last_chunk:
            self._play_audio(chunk)

        if is_last_chunk:
            stt_status_msg = Bool()
            stt_status_msg.data = False
            self.stt_status_publisher.publish(stt_status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TTSLifecycleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()