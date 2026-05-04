#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int16, String
import os
from piper import PiperVoice
from piper.config import SynthesisConfig
from ament_index_python.packages import get_package_share_directory
import tempfile
import wave
from playsound import playsound
import threading
import time

class YarenSpeakerNode(Node):
    def __init__(self):
        super().__init__('yaren_speaker_node')

        self.audio_playing_publisher = self.create_publisher(
            Bool, '/audio_playing', 10)
        
        self.create_subscription(
            String, '/game_feedback', self.speak_game_feedback, 10)
        
        pkg_share_dir_tts = get_package_share_directory('yaren_dice')
        self.tts_model_path = os.path.join(pkg_share_dir_tts, 'models', 'es_MX-claude-high.onnx')
        self.tts_config_path = os.path.join(pkg_share_dir_tts, 'models', 'es_MX-claude-high.onnx.json')
        
        self.voice = None

        self.init_tts()
        
        self.speaking_lock = threading.Lock()
        
        self.get_logger().info('Yaren Speaker Node started successfully')
    
    def init_tts(self):
        try:
            self.voice = PiperVoice.load(
                model_path=self.tts_model_path,
                config_path=self.tts_config_path,
                use_cuda=True
            )
            self.get_logger().info("TTS engine initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize TTS engine: {str(e)}")
    
    def speak_game_feedback(self, msg):
        if msg.data:
            info_data = msg.data
            threading.Thread(target=self.speak_text, args=(info_data,)).start()
    
    def speak_text(self, text):
        if self.voice is None:
            self.get_logger().error("TTS engine not initialized")
            return
        
        syn_config = SynthesisConfig(
            length_scale=1.2,
            noise_scale=0.5,
            noise_w_scale=0.8
        )
            
        with self.speaking_lock:
            audio_status_msg = Bool()
            audio_status_msg.data = True
            self.audio_playing_publisher.publish(audio_status_msg)
            
            try:
                with tempfile.NamedTemporaryFile(suffix=".wav", delete=True) as fp:
                    with wave.open(fp.name, 'wb') as wav_file:
                        wav_file.setnchannels(1)
                        wav_file.setsampwidth(2)
                        wav_file.setframerate(self.voice.config.sample_rate)
                        self.voice.synthesize_wav(text, wav_file, syn_config=syn_config)
                    
                    playsound(fp.name)
            except Exception as e:
                self.get_logger().error(f"Error generating or playing speech: {str(e)}")
            finally:
                audio_status_msg.data = False
                self.audio_playing_publisher.publish(audio_status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YarenSpeakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
