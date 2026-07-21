#!/usr/bin/env python3
import sys
import os
import time
import gc

_node_dir = os.path.dirname(os.path.realpath(__file__))
if _node_dir not in sys.path:
    sys.path.insert(0, _node_dir)

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from yaren_interfaces.msg import PersonResponse
from ament_index_python.packages import get_package_share_directory
from vosk import Model, KaldiRecognizer, SetLogLevel
import pyaudio
import json
import threading
from contextlib import contextmanager
from ctypes import CFUNCTYPE, c_char_p, c_int, cdll

ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)

def py_error_handler(filename, line, function, err, fmt):
    pass

c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

@contextmanager
def noalsaerr():
    asound = cdll.LoadLibrary('libasound.so')
    asound.snd_lib_error_set_handler(c_error_handler)
    yield
    asound.snd_lib_error_set_handler(None)

SetLogLevel(-1)


class STTLocalLifecycleNode(LifecycleNode):

    def __init__(self):
        super().__init__('stt_local_lifecycle_node')

        pkg_share_dir = get_package_share_directory('yaren_chat')
        self.ruta_es = os.path.join(pkg_share_dir, 'models', 'STT', 'vosk-model-es-0.42')
        self.ruta_en = os.path.join(pkg_share_dir, 'models', 'STT', 'vosk-model-en-us-0.22-lgraph')

        self.stt_listening_publisher = self.create_publisher(Bool, '/stt_listening', 10)
        self.response_publisher      = self.create_publisher(PersonResponse, '/response_person_local', 10)
        self.stt_status_publisher    = self.create_publisher(Bool, '/stt_terminado_local', 10)
        qos_mic = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.mic_owner_pub = self.create_publisher(String, '/yaren/mic_owner', qos_mic)
        self.mic_owner = "none"
        self.mic_owner_sub = self.create_subscription(String, '/yaren/mic_owner', self._cb_mic_owner, qos_mic)
        self.mic_owner_event = threading.Event()
        self.mic_owner_event.set()

        self.vosk_model       = None
        self.recognition_thread = None
        self.is_recognizing   = False
        self.idioma_actual    = "es"

        self.tts_finished_event = threading.Event()
        self.tts_finished_event.set()

        qos_profile = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.lang_sub = self.create_subscription(
            String, '/yaren/current_language', self.cb_cambio_idioma, qos_profile)

        self.stt_terminado_sub = self.create_subscription(
            Bool, '/stt_terminado_local', self._on_stt_terminado, 10)

    def _cb_mic_owner(self, msg):
        self.mic_owner = msg.data
        if msg.data == "chat_stt_local":
            self.mic_owner_event.set()
        else:
            self.mic_owner_event.clear()

    def cb_cambio_idioma(self, msg):
        nuevo_idioma = msg.data
        if nuevo_idioma == self.idioma_actual:
            return
        self.get_logger().info(f"🔄 Solicitud de cambio de idioma STT Local a: {nuevo_idioma}")
        self.idioma_actual = nuevo_idioma
        if self.vosk_model is not None:
            self._switch_model_in_memory()

    def _switch_model_in_memory(self):
        reiniciar_hilo = False
        if self.is_recognizing:
            self.is_recognizing = False
            if self.recognition_thread:
                self.recognition_thread.join()
            reiniciar_hilo = True

        self.get_logger().info("🧹 Liberando RAM del modelo STT Local anterior...")
        del self.vosk_model
        self.vosk_model = None
        gc.collect()

        ruta = self.ruta_en if self.idioma_actual == "en" else self.ruta_es
        self.get_logger().info(f"⏳ Cargando nuevo modelo desde: {ruta}")
        try:
            self.vosk_model = Model(ruta)
            self.get_logger().info("✅ Nuevo modelo STT Local cargado exitosamente.")
        except Exception as e:
            self.get_logger().error(f"💥 Error al cambiar modelo: {e}")
            return

        if reiniciar_hilo:
            self.is_recognizing = True
            self.recognition_thread = threading.Thread(target=self._recognize_speech)
            self.recognition_thread.start()

    def _on_stt_terminado(self, msg):
        if not msg.data:
            self.tts_finished_event.set()

    def on_configure(self, state):
        self.get_logger().info('Configuring STT Local Node')

        self.tts_finished_event.set()

        ruta = self.ruta_en if self.idioma_actual == "en" else self.ruta_es
        self.get_logger().info(f'📍 Resolved model path: {ruta}')

        if not os.path.exists(ruta):
            self.get_logger().error(f'❌ Model directory NOT found at: {ruta}')
            return TransitionCallbackReturn.FAILURE
        try:
            self.get_logger().info('⏳ Loading Vosk model (this may take 10-30s)...')
            self.vosk_model = Model(ruta)
            self.get_logger().info('✅ Vosk model loaded successfully')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'💥 Failed to load Vosk model: {type(e).__name__}: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        self.get_logger().info('Activating STT Local Node')
        self.tts_finished_event.set()
        self.is_recognizing = True
        owner_msg = String()
        owner_msg.data = "chat_stt_local"
        self.mic_owner_pub.publish(owner_msg)
        self.mic_owner_event.set()
        self.recognition_thread = threading.Thread(target=self._recognize_speech)
        self.recognition_thread.start()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info('Deactivating STT Local Node')
        self.is_recognizing = False
        self.tts_finished_event.set()
        owner_msg = String()
        owner_msg.data = "none"
        self.mic_owner_pub.publish(owner_msg)
        if self.recognition_thread:
            self.recognition_thread.join()
        return TransitionCallbackReturn.SUCCESS

    def _publish_listening(self, is_listening: bool):
        """Publica el estado de escucha en /stt_listening."""
        msg = Bool()
        msg.data = is_listening
        self.stt_listening_publisher.publish(msg)

    def _recognize_speech(self):
        with noalsaerr():
            p = pyaudio.PyAudio()
            stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000,
                            input=True, frames_per_buffer=8000)

        recognizer = KaldiRecognizer(self.vosk_model, 16000)

        try:
            self.get_logger().info("🎤 Listening (Local)...")
            # ── Primer aviso: turno del niño ──
            self._publish_listening(True)

            while self.is_recognizing:
                if not self.mic_owner_event.wait(timeout=0.1):
                    continue
                data = stream.read(4000, exception_on_overflow=False)

                if not self.tts_finished_event.is_set():
                    recognizer = KaldiRecognizer(self.vosk_model, 16000)
                    continue

                if recognizer.AcceptWaveform(data):
                    result_json = recognizer.Result()
                    result     = json.loads(result_json)
                    text       = result.get("text", "").lower().strip()

                    if text:
                        self.tts_finished_event.clear()

                        # Avisar que el niño dejó de tener el turno (Yaren va a hablar)
                        self._publish_listening(False)

                        response_msg           = PersonResponse()
                        response_msg.text      = text
                        response_msg.timestamp = self.get_clock().now().to_msg()
                        self.response_publisher.publish(response_msg)
                        self.get_logger().info(f"🗣️ Person (Local): {text}")

                        status_msg      = Bool()
                        status_msg.data = True
                        self.stt_status_publisher.publish(status_msg)

                        self.get_logger().info("⏸️ Esperando que Yaren Local termine de hablar...")
                        self.tts_finished_event.wait()

                        # Vaciar buffer acumulado mientras Yaren hablaba
                        time.sleep(0.5)
                        try:
                            while stream.get_read_available() > 0:
                                stream.read(
                                    min(stream.get_read_available(), 4000),
                                    exception_on_overflow=False
                                )
                        except Exception:
                            pass

                        recognizer = KaldiRecognizer(self.vosk_model, 16000)

                        # ── Volver a avisar: ahora es el turno del niño ──
                        self.get_logger().info("🎤 Listening (Local)...")
                        self._publish_listening(True)

        finally:
            self._publish_listening(False)
            stream.stop_stream()
            stream.close()
            p.terminate()


def main(args=None):
    rclpy.init(args=args)
    node = STTLocalLifecycleNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()