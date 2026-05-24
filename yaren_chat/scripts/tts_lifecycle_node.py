#!/usr/bin/env python3
import sys
import os
import gc

_node_dir = os.path.dirname(os.path.realpath(__file__))
if _node_dir not in sys.path:
    sys.path.insert(0, _node_dir)

import rclpy
from rclpy.action import ActionClient
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from yaren_interfaces.msg import PersonResponse
from yaren_interfaces.action import ProcessResponse
from ament_index_python.packages import get_package_share_directory
from piper import PiperVoice
from piper.config import SynthesisConfig
from playsound import playsound
import threading
import tempfile
import wave
import time
import queue
from action_msgs.msg import GoalStatus


class TTSLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('tts_lifecycle_node')

        pkg_share_dir = get_package_share_directory('yaren_chat')
        self.tts_model_path_es = os.path.join(pkg_share_dir, 'models', 'TTS', 'es_MX-claude-high.onnx')
        self.tts_config_path_es = os.path.join(pkg_share_dir, 'models', 'TTS', 'es_MX-claude-high.onnx.json')
        self.tts_model_path_en = os.path.join(pkg_share_dir, 'models', 'TTS', 'en_US-lessac-medium.onnx')
        self.tts_config_path_en = os.path.join(pkg_share_dir, 'models', 'TTS', 'en_US-lessac-medium.onnx.json')

        self.voice = None
        self.idioma_actual = "es"
        self.voice_lock = threading.Lock()

        # Cola de chunks de audio: el LLM produce, el TTS consume en paralelo
        self.audio_queue = queue.Queue()
        self.tts_done = threading.Event()

        # Cargar voz inicial
        self.voice = PiperVoice.load(
            model_path=self.tts_model_path_es,
            config_path=self.tts_config_path_es,
            use_cuda=True
        )

        self.stt_status_publisher  = self.create_publisher(Bool, '/stt_terminado', 10)
        self.audio_playing_publisher = self.create_publisher(Bool, '/audio_playing', 10)
        self.create_subscription(PersonResponse, '/response_person', self.process_input_person, 10)

        qos_profile = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.lang_sub = self.create_subscription(
            String, '/yaren/current_language', self.cb_cambio_idioma, qos_profile
        )

        self.text_person = None
        self._action_client = None

    # ── Idioma ────────────────────────────────────────────────────────
    def cb_cambio_idioma(self, msg):
        nuevo = msg.data
        if nuevo == self.idioma_actual:
            return
        self.get_logger().info(f"🔄 Cambiando voz TTS a: {nuevo}")
        self.idioma_actual = nuevo
        self._switch_voice_in_memory()

    def _switch_voice_in_memory(self):
        with self.voice_lock:
            self.get_logger().info("🧹 Liberando RAM de la voz anterior...")
            del self.voice
            self.voice = None
            gc.collect()
            ruta_m = self.tts_model_path_en if self.idioma_actual == "en" else self.tts_model_path_es
            ruta_c = self.tts_config_path_en if self.idioma_actual == "en" else self.tts_config_path_es
            try:
                self.voice = PiperVoice.load(model_path=ruta_m, config_path=ruta_c, use_cuda=True)
                self.get_logger().info("✅ Nueva voz TTS cargada.")
            except Exception as e:
                self.get_logger().error(f"💥 Error al cambiar voz: {e}")

    # ── Lifecycle ─────────────────────────────────────────────────────
    def process_input_person(self, msg):
        self.text_person = msg.text

    def on_configure(self, state):
        self.get_logger().info('Configuring TTS Node')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating TTS Node')
        self._action_client = ActionClient(self, ProcessResponse, '/response_llama')

        # Limpiar estado anterior
        self.audio_queue = queue.Queue()
        self.tts_done.clear()

        # Hilo productor: manda el goal al LLM y recibe chunks por feedback
        threading.Thread(target=self._send_goal_and_receive_chunks, daemon=True).start()

        # Hilo consumidor: sintetiza y reproduce en cuanto llega cada chunk
        threading.Thread(target=self._audio_worker, daemon=True).start()

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info('Deactivating TTS Node')
        return TransitionCallbackReturn.SUCCESS

    # ── Pipeline productor / consumidor ──────────────────────────────
    def _send_goal_and_receive_chunks(self):
        """Envía el goal al LLM. Cada frase llega por feedback → cola de audio."""
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("❌ LLM action server no disponible")
            self.tts_done.set()
            return

        if self.text_person is None:
            self.get_logger().warn("⚠️ No hay texto para procesar")
            self.tts_done.set()
            return

        goal_msg = ProcessResponse.Goal()
        goal_msg.input_text = self.text_person

        # send_goal_async → future de ACEPTACIÓN
        send_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )

        # Esperar aceptación
        while rclpy.ok() and not send_future.done():
            time.sleep(0.05)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rechazado por el LLM")
            self.tts_done.set()
            return

        self.get_logger().info("✅ Goal aceptado, esperando respuesta del LLM...")

        # Ahora esperar el RESULTADO real (cuando el LLM termina de generar)
        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.05)

        result = result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("✅ LLM terminó de generar respuesta.")
        else:
            self.get_logger().warn(f"⚠️ LLM terminó con status: {result.status}")

        # Señal de fin al worker de audio
        self.audio_queue.put(None)

    def _feedback_callback(self, feedback_msg):
        """Recibe cada frase del LLM y la encola para síntesis inmediata."""
        chunk = feedback_msg.feedback.current_chunk
        is_last = feedback_msg.feedback.is_last_chunk

        if chunk and not is_last:
            self.audio_queue.put(chunk)

        if is_last:
            # El is_last_chunk ya no lleva chunk de audio, solo señal de fin
            # El None se pone en _send_goal_and_receive_chunks al terminar el result
            pass

    def _audio_worker(self):
        """Consume la cola y sintetiza+reproduce cada frase en orden."""
        self.get_logger().info("🔊 Audio worker iniciado")

        while True:
            chunk = self.audio_queue.get()  # bloquea hasta que haya algo

            if chunk is None:  # señal de fin
                break

            self._play_audio(chunk)

        # Todo el audio terminó → avisar al control_manager
        self.get_logger().info("🔊 Audio worker terminó")
        stt_msg = Bool()
        stt_msg.data = False
        self.stt_status_publisher.publish(stt_msg)
        self.tts_done.set()

    def _play_audio(self, text_to_speak):
        audio_msg = Bool()
        audio_msg.data = True
        self.audio_playing_publisher.publish(audio_msg)

        syn_config = SynthesisConfig(length_scale=1.2, noise_scale=0.5, noise_w_scale=0.8)

        try:
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as fp:
                tmp_path = fp.name

            with self.voice_lock:
                if self.voice is None:
                    return
                with wave.open(tmp_path, 'wb') as wav_file:
                    wav_file.setnchannels(1)
                    wav_file.setsampwidth(2)
                    wav_file.setframerate(self.voice.config.sample_rate)
                    self.voice.synthesize_wav(text_to_speak, wav_file, syn_config=syn_config)

            playsound(tmp_path)
            os.unlink(tmp_path)  # limpiar archivo temporal

        except Exception as e:
            self.get_logger().error(f"💥 Error en síntesis/reproducción: {e}")
        finally:
            audio_msg.data = False
            self.audio_playing_publisher.publish(audio_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TTSLifecycleNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()