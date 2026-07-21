#!/usr/bin/env python3
import os
import sys
import re
import queue
import threading
import tempfile
import wave
import time

_node_dir = os.path.dirname(os.path.realpath(__file__))
if _node_dir not in sys.path:
    sys.path.insert(0, _node_dir)

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import Bool, String
from yaren_interfaces.msg import PersonResponse
from yaren_interfaces.action import ProcessResponse
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus

WS = os.path.expanduser("~/robotis_ws/src/YAREN2")

TTS_MODEL_ES = os.path.join(
    os.path.expanduser("~/robotis_ws"),
    "src/YAREN2/yaren_chat/models/TTS/es_MX-claude-high.onnx"
)
TTS_CONFIG_ES = TTS_MODEL_ES + ".json"
TTS_MODEL_EN  = os.path.join(
    os.path.expanduser("~/robotis_ws"),
    "src/YAREN2/yaren_chat/models/TTS/en_US-lessac-medium.onnx"
)
TTS_CONFIG_EN = TTS_MODEL_EN + ".json"


class TTSLocalLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('tts_local_lifecycle_node')

        self._idioma = "es"
        self._voice = None
        self._voice_lock = threading.Lock()
        self._action_client = None
        self._audio_queue = queue.Queue()
        self._tts_done = threading.Event()
        self._text_person = None

        # Publishers
        self.stt_status_publisher    = self.create_publisher(Bool, '/stt_terminado_local', 10)
        self.audio_playing_publisher = self.create_publisher(Bool, '/audio_playing', 10)
        self.tts_text_pub            = self.create_publisher(String, '/yaren/tts_text', 10)
        # ── NUEVO: para indicar turno ──
        self.stt_listening_publisher = self.create_publisher(Bool, '/stt_listening', 10)

        # Subscriptions
        self.create_subscription(
            PersonResponse, '/response_person_local',
            self._process_input, 10)
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.lang_sub = self.create_subscription(
            String, '/yaren/current_language', self._cb_idioma, qos)

        # Cargar voz inicial
        self._load_voice(self._idioma)

    # ── Idioma ─────────────────────────────────────────────────────────────
    def _cb_idioma(self, msg: String):
        if msg.data == self._idioma:
            return
        self._idioma = msg.data
        self.get_logger().info(f"🔄 Cambiando voz TTS local a: {self._idioma}")
        self._load_voice(self._idioma)

    def _load_voice(self, idioma: str):
        from piper import PiperVoice
        model  = TTS_MODEL_EN  if idioma == "en" else TTS_MODEL_ES
        config = TTS_CONFIG_EN if idioma == "en" else TTS_CONFIG_ES
        try:
            with self._voice_lock:
                self._voice = PiperVoice.load(
                    model_path=model, config_path=config, use_cuda=True)
            self.get_logger().info(f"✅ Voz TTS local cargada: {model}")
        except Exception as e:
            self.get_logger().error(f"💥 Error cargando voz: {e}")

    # ── Lifecycle ──────────────────────────────────────────────────────────
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Activating TTS Local Node')
        self._action_client = ActionClient(self, ProcessResponse, '/response_llama_local')
        self._audio_queue = queue.Queue()
        self._tts_done.clear()
        threading.Thread(target=self._audio_worker, daemon=True).start()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating TTS Local Node')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    # ── Entrada de texto ───────────────────────────────────────────────────
    def _process_input(self, msg: PersonResponse):
        self._text_person = msg.text
        self.get_logger().info(f"🎤 Texto recibido (Local): {self._text_person}")
        self._tts_done.clear()
        self._audio_queue = queue.Queue()

        # ── Avisar que Yaren va a hablar (turno de Yaren) ──
        listen_msg = Bool()
        listen_msg.data = False
        self.stt_listening_publisher.publish(listen_msg)

        threading.Thread(target=self._audio_worker, daemon=True).start()
        threading.Thread(target=self._send_goal, daemon=True).start()

    # ── Pipeline LLM → TTS ────────────────────────────────────────────────
    def _send_goal(self):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ LLM local action server no disponible")
            self._audio_queue.put(None)
            return

        if self._text_person is None:
            self._audio_queue.put(None)
            return

        goal = ProcessResponse.Goal()
        goal.input_text = self._text_person

        self.get_logger().info("✅ Enviando texto al LLM Local...")
        future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)

        while rclpy.ok():
            if future.done():
                break
            time.sleep(0.05)

        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error("❌ Goal rechazado")
            self._audio_queue.put(None)
            return

        result_future = handle.get_result_async()
        while rclpy.ok():
            if result_future.done():
                break
            time.sleep(0.05)

        self.get_logger().info("✅ LLM Local terminó de generar toda la respuesta.")
        self._audio_queue.put(None)

        self.get_logger().info("⏳ Esperando a que el worker de TTS Local termine...")
        self._tts_done.wait()

    def _feedback_cb(self, feedback_msg):
        chunk   = feedback_msg.feedback.current_chunk
        is_last = feedback_msg.feedback.is_last_chunk
        if chunk and not is_last:
            self._audio_queue.put(chunk)

    # ── Worker de audio ────────────────────────────────────────────────────
    def _audio_worker(self):
        self.get_logger().info("🔊 Audio worker Local iniciado")
        while True:
            chunk = self._audio_queue.get()
            if chunk is None:
                break
            self._play_audio(chunk)

        self._tts_done.set()

        # Avisar al STT que el TTS terminó (stt_terminado_local = False)
        stt_msg = Bool()
        stt_msg.data = False
        self.stt_status_publisher.publish(stt_msg)

        self.get_logger().info("🔊 Audio worker Local terminó y liberó el bloqueo")

    def _play_audio(self, text: str):
        from piper.config import SynthesisConfig
        from playsound import playsound

        audio_msg = Bool()
        audio_msg.data = True
        self.audio_playing_publisher.publish(audio_msg)

        syn_config = SynthesisConfig(
            length_scale=1.2, noise_scale=0.5, noise_w_scale=0.8)

        try:
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as fp:
                tmp_path = fp.name

            with self._voice_lock:
                if self._voice is None:
                    return
                with wave.open(tmp_path, 'wb') as wf:
                    wf.setnchannels(1)
                    wf.setsampwidth(2)
                    wf.setframerate(self._voice.config.sample_rate)
                    self._voice.synthesize_wav(text, wf, syn_config=syn_config)

            self._publish_visemes(tmp_path)
            playsound(tmp_path)
            os.unlink(tmp_path)

        except Exception as e:
            self.get_logger().error(f"💥 Error en síntesis/reproducción: {e}")
        finally:
            audio_msg.data = False
            self.audio_playing_publisher.publish(audio_msg)

    def _publish_visemes(self, wav_path: str):
        import struct
        try:
            with wave.open(wav_path, 'rb') as wf:
                framerate  = wf.getframerate()
                n_frames   = wf.getnframes()
                chunk_ms   = 50
                chunk_size = int(framerate * chunk_ms / 1000)
                visemes    = []
                for _ in range(n_frames // chunk_size):
                    raw = wf.readframes(chunk_size)
                    if len(raw) < chunk_size * 2:
                        break
                    samples = struct.unpack(f'{len(raw)//2}h', raw)
                    rms = (sum(s*s for s in samples) / len(samples)) ** 0.5
                    if   rms < 200:   idx = 0
                    elif rms < 800:   idx = 1
                    elif rms < 2000:  idx = 3
                    elif rms < 5000:  idx = 5
                    elif rms < 10000: idx = 6
                    else:             idx = 7
                    visemes.append(idx)
            msg = String()
            msg.data = f"{chunk_ms}:" + ",".join(str(v) for v in visemes)
            self.tts_text_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Viseme extraction failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TTSLocalLifecycleNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()