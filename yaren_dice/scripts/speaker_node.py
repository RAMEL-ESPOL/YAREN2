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
import numpy as np
import threading
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


class YarenSpeakerNode(LifecycleNode):

    def __init__(self):
        super().__init__('yaren_speaker_node')
        self.voices        = {}
        self.is_english    = False
        self.speaking_lock = threading.Lock()
        self._active       = False
        self.audio_playing_publisher  = None
        self.lipsync_publisher        = None
        self.feedback_subscription    = None
        self.language_subscription    = None
        pkg = get_package_share_directory('yaren_dice')
        self.tts_model_path_es  = os.path.join(pkg, 'models', 'es_MX-claude-high.onnx')
        self.tts_config_path_es = os.path.join(pkg, 'models', 'es_MX-claude-high.onnx.json')
        self.tts_model_path_en  = os.path.join(pkg, 'models', 'en_US-lessac-medium.onnx')
        self.tts_config_path_en = os.path.join(pkg, 'models', 'en_US-lessac-medium.onnx.json')

    # ──────────────────────────────────────────────
    # Lifecycle callbacks
    # ──────────────────────────────────────────────

    def on_configure(self, state):
        self.get_logger().info('Cargando modelos TTS...')
        try:
            self._load_tts()
            self.audio_playing_publisher = self.create_publisher(Bool, '/audio_playing', 10)
            self.lipsync_publisher       = self.create_publisher(String, '/yaren/tts_text', 10)
            qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self.language_subscription = self.create_subscription(
                Bool, '/yaren/is_english', self.language_callback, qos)
            self.get_logger().info('Modelos TTS listos ✓')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Error en configure: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        self.get_logger().info('YarenSpeaker ACTIVO')
        self._active = True
        self.feedback_subscription = self.create_subscription(
            String, '/game_feedback', self.speak_game_feedback, 10)
        return super().on_activate(state)

    def on_deactivate(self, state):
        self.get_logger().info('YarenSpeaker en PAUSA')
        self._active = False
        if self.feedback_subscription is not None:
            self.destroy_subscription(self.feedback_subscription)
            self.feedback_subscription = None
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self.voices = {}
        for pub in [self.audio_playing_publisher, self.lipsync_publisher]:
            if pub is not None:
                self.destroy_publisher(pub)
        self.audio_playing_publisher = None
        self.lipsync_publisher       = None
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

    def _compute_lipsync(self, wav_path: str, sample_rate: int) -> str:
        """
        Analiza el WAV por amplitud RMS y genera una secuencia de índices
        de sprite (0-8) con duración fija por frame, igual que tts_lifecycle_node.
        Retorna el string en formato 'dur_ms:idx0,idx1,idx2,...'
        """
        try:
            with wave.open(wav_path, 'rb') as wf:
                n_frames    = wf.getnframes()
                n_channels  = wf.getnchannels()
                sampwidth   = wf.getsampwidth()
                raw         = wf.readframes(n_frames)

            # Decodificar samples
            if sampwidth == 2:
                samples = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
            elif sampwidth == 4:
                samples = np.frombuffer(raw, dtype=np.int32).astype(np.float32) / 2147483648.0
            else:
                samples = np.frombuffer(raw, dtype=np.uint8).astype(np.float32) / 128.0 - 1.0

            # Mezclar a mono si es estéreo
            if n_channels > 1:
                samples = samples.reshape(-1, n_channels).mean(axis=1)

            # Tamaño de ventana: 80ms por frame
            FRAME_MS   = 80
            frame_size = int(sample_rate * FRAME_MS / 1000)
            if frame_size == 0:
                return ""

            indices = []
            for i in range(0, len(samples), frame_size):
                chunk = samples[i:i + frame_size]
                if len(chunk) == 0:
                    break
                rms = float(np.sqrt(np.mean(chunk ** 2)))

                # Mapear RMS → sprite (0 = cerrada, 8 = muy abierta)
                if rms < 0.01:
                    idx = 0
                elif rms < 0.03:
                    idx = 1
                elif rms < 0.06:
                    idx = 2
                elif rms < 0.10:
                    idx = 3
                elif rms < 0.15:
                    idx = 4
                elif rms < 0.20:
                    idx = 5
                elif rms < 0.27:
                    idx = 6
                elif rms < 0.35:
                    idx = 7
                else:
                    idx = 8

                indices.append(str(idx))

            if not indices:
                return ""

            return f"{FRAME_MS}:{','.join(indices)}"

        except Exception as e:
            self.get_logger().error(f'[LipSync] Error analizando WAV: {e}')
            return ""

    # ──────────────────────────────────────────────
    # Callbacks
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
                # Usar archivo temporal que no se borra inmediatamente
                # para poder analizarlo antes de reproducirlo
                with tempfile.NamedTemporaryFile(
                        suffix='.wav', delete=False) as fp:
                    wav_path = fp.name

                # 1. Sintetizar WAV
                with wave.open(wav_path, 'wb') as wav_file:
                    wav_file.setnchannels(1)
                    wav_file.setsampwidth(2)
                    wav_file.setframerate(voice.config.sample_rate)
                    voice.synthesize_wav(text, wav_file, syn_config=syn_config)

                # 2. Calcular lip sync y publicar ANTES de reproducir
                lipsync_str = self._compute_lipsync(wav_path, voice.config.sample_rate)
                if lipsync_str and self.lipsync_publisher is not None:
                    msg_out      = String()
                    msg_out.data = lipsync_str
                    self.lipsync_publisher.publish(msg_out)
                    self.get_logger().debug(f'[LipSync] Publicado: {lipsync_str[:60]}...')

                # 3. Reproducir audio
                import subprocess
                subprocess.run(
                    ['aplay', wav_path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )

            except Exception as e:
                self.get_logger().error(f'Error en audio: {e}')
            finally:
                # Limpiar archivo temporal
                try:
                    os.remove(wav_path)
                except Exception:
                    pass
                self._publish_audio_status(False)

    def _publish_audio_status(self, playing: bool):
        if self.audio_playing_publisher is not None:
            msg      = Bool()
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