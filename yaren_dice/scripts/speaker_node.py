#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
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


class YarenSpeakerNode(LifecycleNode):

    def __init__(self):
        super().__init__('yaren_speaker_node')
        self.voices        = {}
        self.is_english    = False
        self.speaking_lock = threading.Lock()
        self._active       = False
        self.audio_playing_publisher = None
        self.feedback_subscription   = None
        self.language_subscription   = None
        # Rutas de modelos (solo strings, sin cargar nada)
        pkg = get_package_share_directory('yaren_dice')
        self.tts_model_path_es  = os.path.join(pkg, 'models', 'es_MX-claude-high.onnx')
        self.tts_config_path_es = os.path.join(pkg, 'models', 'es_MX-claude-high.onnx.json')
        self.tts_model_path_en  = os.path.join(pkg, 'models', 'en_US-lessac-medium.onnx')
        self.tts_config_path_en = os.path.join(pkg, 'models', 'en_US-lessac-medium.onnx.json')

    # ──────────────────────────────────────────────
    # Lifecycle callbacks
    # ──────────────────────────────────────────────

    def on_configure(self, state):
        """Carga ambos modelos TTS una sola vez."""
        self.get_logger().info('Cargando modelos TTS...')
        try:
            self._load_tts()
            self.audio_playing_publisher = self.create_publisher(Bool, '/audio_playing', 10)
            # Idioma: escuchar siempre aunque el nodo esté en pausa
            qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self.language_subscription = self.create_subscription(
                Bool, '/yaren/is_english', self.language_callback, qos)
            self.get_logger().info('Modelos TTS listos ✓')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Error en configure: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        """Empieza a escuchar feedback del juego."""
        self.get_logger().info('YarenSpeaker ACTIVO')
        self._active = True
        self.feedback_subscription = self.create_subscription(
            String, '/game_feedback', self.speak_game_feedback, 10)
        return super().on_activate(state)

    def on_deactivate(self, state):
        """Deja de hablar — modelos siguen en RAM."""
        self.get_logger().info('YarenSpeaker en PAUSA')
        self._active = False
        if self.feedback_subscription is not None:
            self.destroy_subscription(self.feedback_subscription)
            self.feedback_subscription = None
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self.voices = {}
        if self.audio_playing_publisher is not None:
            self.destroy_publisher(self.audio_playing_publisher)
            self.audio_playing_publisher = None
        if self.language_subscription is not None:
            self.destroy_subscription(self.language_subscription)
            self.language_subscription = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self._active = False
        self.voices  = {}
        return TransitionCallbackReturn.SUCCESS

    # ──────────────────────────────────────────────
    # Helpers privados
    # ──────────────────────────────────────────────

    def _load_tts(self):
        for lang, model, config in [
            ('es', self.tts_model_path_es, self.tts_config_path_es),
            ('en', self.tts_model_path_en, self.tts_config_path_en),
        ]:
            try:
                self.voices[lang] = PiperVoice.load(
                    model_path=model, config_path=config, use_cuda=False)
                self.get_logger().info(f'TTS ({lang}) cargado ✓')
            except Exception as e:
                self.get_logger().error(f'Fallo TTS ({lang}): {e}')

    # ──────────────────────────────────────────────
    # Callbacks (lógica idéntica al original)
    # ──────────────────────────────────────────────

    def language_callback(self, msg):
        self.is_english = msg.data
        self.get_logger().info(f"Idioma: {'English' if self.is_english else 'Español'}")

    def speak_game_feedback(self, msg):
        if not self._active or not msg.data:
            return
        threading.Thread(target=self.speak_text, args=(msg.data,), daemon=True).start()

    def speak_text(self, text):
        if not self._active:
            return
        lang  = 'en' if self.is_english else 'es'
        voice = self.voices.get(lang)
        if voice is None:
            self.get_logger().error(f"TTS '{lang}' no está cargado.")
            return
        syn_config = SynthesisConfig(
            length_scale=1.2, noise_scale=0.5, noise_w_scale=0.8)
        with self.speaking_lock:
            self._publish_audio_status(True)
            try:
                with tempfile.NamedTemporaryFile(suffix='.wav', delete=True) as fp:
                    with wave.open(fp.name, 'wb') as wav_file:
                        wav_file.setnchannels(1)
                        wav_file.setsampwidth(2)
                        wav_file.setframerate(voice.config.sample_rate)
                        voice.synthesize_wav(text, wav_file, syn_config=syn_config)
                    playsound(fp.name)
            except Exception as e:
                self.get_logger().error(f'Error en audio: {e}')
            finally:
                self._publish_audio_status(False)

    def _publish_audio_status(self, playing: bool):
        if self.audio_playing_publisher is not None:
            msg = Bool()
            msg.data = playing
            self.audio_playing_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = YarenSpeakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()