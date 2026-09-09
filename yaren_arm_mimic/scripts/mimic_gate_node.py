#!/usr/bin/env python3
"""
mimic_gate_node.py
==================
Convertido a LifecycleNode para ahorrar recursos.
Se activa/desactiva directamente desde el C++ Orchestrator.
"""
import os
import json
import pyaudio
from vosk import Model, KaldiRecognizer

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String

ENABLE_PHRASES  = ["activar modo", "activar", "enable", "start", "comenzar", "listo", "iniciar"]
DISABLE_PHRASES = ["apagar modo", "a pagar", "disable", "stop", "parar", "detener","fin"]

def _matches(text: str, phrases: list) -> bool:
    t = text.lower().strip()
    return any(p in t for p in phrases)

class MimicGateNode(LifecycleNode):
    def __init__(self):
        super().__init__("mimic_gate_node")
        self.mimic_enabled = False
        self.mic_owner = "none"
        self._is_active = False
        
        self.vosk_model = None
        self.recognizer = None
        self.mic = None
        self.stream = None
        
        self.get_logger().info("mimic_gate_node creado (Unconfigured).")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configurando mimic_gate_node...")
        
        # Cargar modelo Vosk
        workspace_dir = os.getcwd()
        model_path = os.path.join(
            workspace_dir, "src", "YAREN2", "yaren_chat", "models", "STT",
            "vosk-model-small-es-0.42"
        )
        if not os.path.exists(model_path):
            self.get_logger().error(f"Modelo Vosk no encontrado: {model_path}")
            return TransitionCallbackReturn.FAILURE

        self.vosk_model = Model(model_path)
        
        # Iniciar PyAudio
        self.mic = pyaudio.PyAudio()

        # Configurar Publishers y Subscribers (Lifecycle)
        qos_tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mimic_pub = self.create_lifecycle_publisher(Bool, "/mimic/enabled", 10)
        self.mic_owner_pub = self.create_lifecycle_publisher(String, "/yaren/mic_owner", qos_tl)

        self.sub_mic_owner = self.create_subscription(String, "/yaren/mic_owner", self._cb_mic_owner, qos_tl)

        # Timer para el loop de audio
        self.audio_timer = self.create_timer(0.1, self._audio_loop)
        
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activando mimic_gate_node...")
        super().on_activate(state)
        
        self._is_active = True
        self._set_enabled(False) # Comienza desactivado (esperando la voz "activar modo")
        self._claim_mic()        # Pedir el micrófono
        
        self.get_logger().info("Modo mimic ACTIVO. Di 'activar modo'.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Desactivando mimic_gate_node...")
        self._is_active = False
        
        self._set_enabled(False)
        self._release_mic()      # Devolver el micrófono
        
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Limpiando mimic_gate_node...")
        self._close_stream()
        
        if self.mic:
            self.mic.terminate()
            self.mic = None
        
        if self.audio_timer:
            self.destroy_timer(self.audio_timer)
            self.audio_timer = None
            
        self.vosk_model = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._close_stream()
        if self.mic:
            self.mic.terminate()
        return TransitionCallbackReturn.SUCCESS

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_mic_owner(self, msg: String):
        self.mic_owner = msg.data
        if self.mic_owner == "mimic_gate" and self._is_active:
            self._open_stream()
        elif self.mic_owner != "mimic_gate":
            self._close_stream()

    # ── Audio loop ────────────────────────────────────────────────────────────

    def _audio_loop(self):
        if not self._is_active or self.mic_owner != "mimic_gate":
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

                self.get_logger().info(f"STT Mimic: '{text}'")

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
        self.get_logger().info(f"/mimic/enabled -> {enabled}")

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
            self.get_logger().info("Microfono abierto para Mimic Gate.")
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
        self.get_logger().info("Microfono cerrado para Mimic Gate.")

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