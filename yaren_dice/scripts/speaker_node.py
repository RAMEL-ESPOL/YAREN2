#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import os
from piper import PiperVoice
from piper.config import SynthesisConfig
from ament_index_python.packages import get_package_share_directory
import tempfile
import wave
from playsound import playsound
import threading
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class YarenSpeakerNode(Node):
    def __init__(self):
        super().__init__('yaren_speaker_node')

        self.audio_playing_publisher = self.create_publisher(
            Bool, '/audio_playing', 10)
        
        self.create_subscription(
            String, '/game_feedback', self.speak_game_feedback, 10)
        
        # 1. Variable de estado inicial
        self.is_english = False
        
        # 2. Configurar QoS Transient Local para ESCUCHAR a la pantalla
        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # 3. Suscribirse al tópico de idioma que publica face_screen.cpp
        self.create_subscription(
            Bool, '/yaren/is_english', self.language_callback, qos_profile)
        
        pkg_share_dir_tts = get_package_share_directory('yaren_dice')
        
        # Rutas del modelo en Español
        self.tts_model_path_es = os.path.join(pkg_share_dir_tts, 'models', 'es_MX-claude-high.onnx')
        self.tts_config_path_es = os.path.join(pkg_share_dir_tts, 'models', 'es_MX-claude-high.onnx.json')
        
        # Rutas del modelo en Inglés
        self.tts_model_path_en = os.path.join(pkg_share_dir_tts, 'models', 'en_US-lessac-medium.onnx')
        self.tts_config_path_en = os.path.join(pkg_share_dir_tts, 'models', 'en_US-lessac-medium.onnx.json')
        
        # Diccionario para almacenar ambas voces cargadas
        self.voices = {}

        self.init_tts()
        
        self.speaking_lock = threading.Lock()
        
        self.get_logger().info('Yaren Speaker Node started successfully (Bilingual Mode)')
    
    def init_tts(self):
        # Cargar modelo en Español
        try:
            self.voices['es'] = PiperVoice.load(
                model_path=self.tts_model_path_es,
                config_path=self.tts_config_path_es,
                use_cuda=False
            )
            self.get_logger().info("Motor TTS (Español) inicializado correctamente")
        except Exception as e:
            self.get_logger().error(f"Fallo al inicializar TTS (Español): {str(e)}")

        # Cargar modelo en Inglés
        try:
            self.voices['en'] = PiperVoice.load(
                model_path=self.tts_model_path_en,
                config_path=self.tts_config_path_en,
                use_cuda=False
            )
            self.get_logger().info("Motor TTS (Inglés) inicializado correctamente")
        except Exception as e:
            self.get_logger().error(f"Fallo al inicializar TTS (Inglés): Error: {str(e)}")

    def language_callback(self, msg):
        """Callback que se ejecuta cuando la pantalla (C++) publica un cambio de idioma"""
        self.is_english = msg.data
        lang_str = "English" if self.is_english else "Español"
        
        self.get_logger().info(f"Speaker actualizado al idioma: {lang_str}")
    
    def speak_game_feedback(self, msg):
        if msg.data:
            info_data = msg.data
            threading.Thread(target=self.speak_text, args=(info_data,)).start()
    
    def speak_text(self, text):
        # Seleccionar la voz dependiendo del idioma activo
        current_lang = 'en' if self.is_english else 'es'
        active_voice = self.voices.get(current_lang)

        if active_voice is None:
            self.get_logger().error(f"El motor TTS para el idioma '{current_lang}' no está cargado.")
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
                        wav_file.setframerate(active_voice.config.sample_rate)
                        
                        # Usar la voz activa (Español o Inglés) para sintetizar
                        active_voice.synthesize_wav(text, wav_file, syn_config=syn_config)
                    
                    playsound(fp.name)
            except Exception as e:
                self.get_logger().error(f"Error generando o reproduciendo audio: {str(e)}")
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