#!/usr/bin/env python3
"""
mimic_gate_node.py
==================
Escucha comandos de voz para activar/desactivar la réplica de brazos.

Flujo:
  1. /yaren_mode recibe "yaren_mimic"  → pide mic, espera "activar modo"
  2. "activar modo"  → /mimic/enabled = True
  3. "apagar modo"   → /mimic/enabled = False
  4. /yaren_mode recibe otro modo      → libera mic, para el gate

Topics:
  SUB  /yaren_mode        (std_msgs/String)  - modo activo global
  SUB  /yaren/mic_owner   (std_msgs/String)  - mutex de micrófono
  PUB  /yaren/mic_owner   (std_msgs/String)
  PUB  /mimic/enabled     (std_msgs/Bool)
"""

import os
import json
import queue
import threading
import pyaudio
from vosk import Model, KaldiRecognizer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String

ENABLE_PHRASES  = ["activar modo", "activar", "enable", "start", "comenzar", "listo", "iniciar"]
DISABLE_PHRASES = ["apagar modo", "a pagar", "disable", "stop", "parar", "detener","fin"]

def _matches(text: str, phrases: list) -> bool:
    t = text.lower().strip()
    return any(p in t for p in phrases)


class MimicGateNode(Node):

    def __init__(self):
        super().__init__("mimic_gate_node")

        # ── Estado ────────────────────────────────────────────────────────────
        self.mimic_active  = False   # el modo mimic está corriendo
        self.mimic_enabled = False   # el gate está abierto (robot imita)
        self.mic_owner     = "none"

        # ── Vosk ──────────────────────────────────────────────────────────────
        workspace_dir = os.getcwd()
        model_path = os.path.join(
            workspace_dir, "src", "YAREN2", "yaren_chat", "models", "STT",
            "vosk-model-small-es-0.42"
        )
        if not os.path.exists(model_path):
            self.get_logger().error(f"Modelo Vosk no encontrado: {model_path}")
            return

        self.vosk_model = Model(model_path)
        self.recognizer = KaldiRecognizer(self.vosk_model, 16000)

        # ── Micrófono ─────────────────────────────────────────────────────────
        self.mic    = pyaudio.PyAudio()
        self.stream = None   # se abre solo cuando mimic está activo y es dueño del mic

        # ── ROS2 ──────────────────────────────────────────────────────────────
        qos_tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.mimic_pub    = self.create_publisher(Bool,   "/mimic/enabled",   10)
        self.mic_owner_pub = self.create_publisher(String, "/yaren/mic_owner", qos_tl)

        self.create_subscription(String, "/yaren_mode",       self._cb_mode,      10)
        self.create_subscription(String, "/yaren/mic_owner",  self._cb_mic_owner, qos_tl)

        self.create_timer(0.1, self._audio_loop)

        self.get_logger().info("mimic_gate_node listo.")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_mode(self, msg: String):
        if msg.data == "yaren_mimic":
            if not self.mimic_active:
                self.mimic_active = True
                self._set_enabled(False)          # empieza desactivado
                self._claim_mic()
                self.get_logger().info("Modo mimic detectado. Di 'activar modo'.")
        else:
            if self.mimic_active:
                self.mimic_active = False
                self._set_enabled(False)
                self._release_mic()

    def _cb_mic_owner(self, msg: String):
        self.mic_owner = msg.data
        if self.mic_owner == "mimic_gate":
            self._open_stream()
        elif self.mic_owner != "mimic_gate":
            self._close_stream()

    # ── Audio loop ────────────────────────────────────────────────────────────

    def _audio_loop(self):
        if not self.mimic_active or self.mic_owner != "mimic_gate":
            return
        if self.stream is None:
            return

        try:
            data = self.stream.read(1600, exception_on_overflow=False)
            if not data:
                return

            if self.recognizer.AcceptWaveform(data):
                result = json.loads(self.recognizer.Result())
                text   = result.get("text", "").strip()
                if not text:
                    return

                self.get_logger().info(f"STT: '{text}'")

                if _matches(text, ENABLE_PHRASES):
                    self._set_enabled(True)
                elif _matches(text, DISABLE_PHRASES):
                    self._set_enabled(False)

        except Exception:
            pass

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _set_enabled(self, enabled: bool):
        self.mimic_enabled = enabled
        msg = Bool()
        msg.data = enabled
        self.mimic_pub.publish(msg)
        self.get_logger().info(f"/mimic/enabled → {enabled}")

    def _claim_mic(self):
        msg = String()
        msg.data = "mimic_gate"
        self.mic_owner_pub.publish(msg)

    def _release_mic(self):
        msg = String()
        msg.data = "none"
        self.mic_owner_pub.publish(msg)
        self._close_stream()

    def _open_stream(self):
        if self.stream is not None:
            return
        try:
            self.stream = self.mic.open(
                format=pyaudio.paInt16, channels=1, rate=16000,
                input=True, frames_per_buffer=8000
            )
            self.stream.start_stream()
            self.recognizer = KaldiRecognizer(self.vosk_model, 16000)
            self.get_logger().info("Stream abierto.")
        except Exception as e:
            self.get_logger().error(f"Error abriendo stream: {e}")
            self.stream = None

    def _close_stream(self):
        if self.stream is None:
            return
        try:
            self.stream.stop_stream()
            self.stream.close()
        except Exception:
            pass
        self.stream = None
        self.get_logger().info("Stream cerrado.")

    def destroy_node(self):
        self._close_stream()
        try:
            self.mic.terminate()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MimicGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()