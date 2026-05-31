#!/usr/bin/env python3
"""
wake_word_node.py
=================
Detecta la wake word por voz y publica el evento + idioma detectado.

Wake words:
  Español : "hola yaren", "hola amigo", "oye yaren"
  Inglés  : "hey yaren", "hello yaren", "wake up"

Topics ROS2:
  SUB  /yaren/face_idle      (std_msgs/Bool)   - pantalla libre o en menú
  PUB  /yaren/wake_event     (std_msgs/Bool)   - wake word detectada
  PUB  /yaren/is_english     (std_msgs/Bool)   - True=inglés, False=español
"""

import os
import json
import pyaudio
from vosk import Model, KaldiRecognizer
import time


import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy


# Wake words por idioma
WAKE_WORDS_ES = [
    # Frases exactas
    "hola yaren", "hola amigo", "oye yaren",
    # Variantes reales confirmadas por logs
    "hola hallaren",   # "hola yaren" ✅ confirmado
    "o llegaren",      # "oye yaren" ✅ confirmado
    "o llegaron",      # "oye yaren" ✅ confirmado
    "ya harén",        # "yaren" solo ✅ confirmado antes
    "y ya harén",      # "hey yaren" ✅ confirmado antes
    "oye ya haré",     # "oye yaren" ✅ confirmado antes
    "hola jardín",     # "hola yaren" ✅ confirmado antes
    "hola clarín",     # "hola yaren" ✅ confirmado antes
    "hola harén",      # variante probable
    "oye harén",       # variante probable
    "hola llaren", "oye llaren",
    "hola jaren",  "oye jaren",
    "hola laren",  "oye laren",
    "hola robot",  "oye robot",
]


# Para inglés: Vosk español transcribe "hey" como "ey", "el", "hay"
# y "hello" como "elo", "ello", "helo" — cubrimos todas
WAKE_WORDS_EN = [

    # Confirmadas por logs de sesión inglés}
    "hola inglés",    # "hey yaren" ✅
    "jay llevaren",    # "hey yaren" ✅
    "y yo harén",      # "hey yaren" ✅
    "jay robots",      # "hey robot" ✅
    "llevaren",        # "yaren" solo ✅
    "jay harén",       # "hey yaren" probable
    "jay laren",       # "hey yaren" probable
    "jay jardín",      # "hey yaren" probable
    # Por si Vosk transcribe bien
    "hey yaren", "hello yaren", "hey friend", "hey robot",
    # Variantes fonéticas anteriores
    "ey yaren",  "ey harén",  "hay yaren", "hay harén",
]
class YarenWakeWordNode(Node):
    def __init__(self):
        super().__init__('yaren_wake_word_node')

        self.is_face_idle = False

        # QoS Transient Local para leer el último estado al conectarse
        qos_tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.idle_sub = self.create_subscription(
            Bool, '/yaren/face_idle', self.idle_callback, qos_tl)

        self.wake_pub = self.create_publisher(Bool, '/yaren/wake_event', 10)
        self.lang_pub = self.create_publisher(Bool, '/yaren/is_english',  qos_tl)

        # Modelo Vosk
        workspace_dir = os.getcwd()
        model_path = os.path.join(
            workspace_dir, 'src', 'YAREN2', 'yaren_chat', 'models', 'STT',
            'vosk-model-small-es-0.42'
        )

        if not os.path.exists(model_path):
            self.get_logger().error(f"Modelo no encontrado: {model_path}")
            self.get_logger().error(f"Directorio actual: {workspace_dir}")
            return

        self.model      = Model(model_path)
        self.recognizer = KaldiRecognizer(self.model, 16000)

        # Micrófono
        self.mic    = pyaudio.PyAudio()
        self.stream = self.mic.open(
            format=pyaudio.paInt16, channels=1, rate=16000,
            input=True, frames_per_buffer=8000
        )
        self.stream.start_stream()

        self.create_timer(0.1, self.audio_loop_callback)
        self.get_logger().info("🤖 wake_word_node listo. Esperando wake word...")

    def idle_callback(self, msg: Bool):
        self.is_face_idle = msg.data
        if self.is_face_idle:
            self.get_logger().info("✅ Pantalla libre. Escuchando wake word...")
            self.recognizer = KaldiRecognizer(self.model, 16000)
        else:
            self.get_logger().info("🛑 Menú activo. Micrófono pausado.")

    def audio_loop_callback(self):
        if not self.is_face_idle:
            try:
                available = self.stream.get_read_available()
                if available > 0:
                    self.stream.read(available, exception_on_overflow=False)
            except Exception:
                pass
            return

        try:
            data = self.stream.read(4000, exception_on_overflow=False)
            if not data:
                return

            if self.recognizer.AcceptWaveform(data):
                result = json.loads(self.recognizer.Result())
                text   = result.get("text", "").lower().strip()
                if not text:
                    return

                self.get_logger().info(f"🎤 Transcripción: '{text}'")  # ← log

                # ← matching aquí dentro del if, no en el else
                matched_es = max((w for w in WAKE_WORDS_ES if w in text), key=len, default=None)
                matched_en = max((w for w in WAKE_WORDS_EN if w in text), key=len, default=None)

                if matched_es or matched_en:
                    is_english = matched_en is not None and matched_es is None
                    matched    = matched_en if is_english else matched_es

                    self.get_logger().info(f"✨ Wake word: '{matched}' → {'EN' if is_english else 'ES'}")

                    lang_msg      = Bool()
                    lang_msg.data = is_english
                    self.lang_pub.publish(lang_msg)
                    time.sleep(0.05)

                    wake_msg      = Bool()
                    wake_msg.data = True
                    self.wake_pub.publish(wake_msg)

                    self.is_face_idle = False

            else:
                # ← parciales solo para debug, sin matching
                partial      = json.loads(self.recognizer.PartialResult())
                partial_text = partial.get("partial", "").strip()
                if partial_text:
                    self.get_logger().info(f"🔄 Parcial: '{partial_text}'")

        except Exception:   
            pass

    def destroy_node(self):
        try:
            self.stream.stop_stream()
            self.stream.close()
            self.mic.terminate()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YarenWakeWordNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()