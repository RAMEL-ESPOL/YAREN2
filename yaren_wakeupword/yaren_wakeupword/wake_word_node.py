#!/usr/bin/env python3
"""wake_word_node.py (unificado): wake words + comandos con un solo micrófono."""
import json, time
from vosk import KaldiRecognizer
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from voice_commands_node import VoiceCommandsNode, parse

WAKE_WORDS_ES = [
    "hola yaren", "hola amigo", "oye yaren",
    "hola hallaren", "o llegaren", "o llegaron", "ya harén", "y ya harén",
    "oye ya haré", "hola jardín", "hola clarín", "hola harén", "oye harén",
    "hola llaren", "oye llaren", "hola jaren", "oye jaren",
    "hola laren", "oye laren", "hola robot", "oye robot",
]
WAKE_WORDS_EN = [
    "hola inglés", "jay llevaren", "y yo harén", "jay robots", "llevaren",
    "jay harén", "jay laren", "jay jardín",
    "hey yaren", "hello yaren", "hey friend", "hey robot",
    "ey yaren", "ey harén", "hay yaren", "hay harén",
]


class WakeAndCommandsNode(VoiceCommandsNode):
    def __init__(self):
        super().__init__()
        tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.is_face_idle = False
        self.wake_rec = KaldiRecognizer(self.model, 16000)
        self.wake_pub = self.create_publisher(Bool, '/yaren/wake_event', 10)
        self.lang_pub = self.create_publisher(Bool, '/yaren/is_english', tl)
        self.create_subscription(Bool, '/yaren/face_idle', self._cb_idle, tl)
        self._wake_ready = True
        self.get_logger().info('wake_word_node unificado listo (wake words + comandos + caras)')

    def _cb_idle(self, m):
        if m.data and not self.is_face_idle:
            self.wake_rec = KaldiRecognizer(self.model, 16000)
        self.is_face_idle = m.data

    def _reset_recs(self):
        self.rec.Reset()
        self.wake_rec = KaldiRecognizer(self.model, 16000)

    def _check_wake(self, text):
        text = text.lower().strip()
        if not text:
            return
        self.get_logger().info(f'👂 oído: "{text}"')
        es = max((w for w in WAKE_WORDS_ES if w in text), key=len, default=None)
        en = max((w for w in WAKE_WORDS_EN if w in text), key=len, default=None)
        if not (es or en):
            return
        is_en = en is not None and es is None
        self.get_logger().info(f"✨ Wake word: '{en if is_en else es}' → {'EN' if is_en else 'ES'}")
        self.lang_pub.publish(Bool(data=is_en))
        time.sleep(0.05)
        self.wake_pub.publish(Bool(data=True))
        self.is_face_idle = False

    def _on_audio(self, data):
        if not getattr(self, '_wake_ready', False):
            return
        if self._capture is not None:                  # capturando nombre: nada más escucha
            return super()._on_audio(data)
        cmd_done = self.rec.AcceptWaveform(data)
        wake_done = self.wake_rec.AcceptWaveform(data)
        cmd = None
        if cmd_done:
            text = json.loads(self.rec.Result()).get('text', '')
            cmd = parse(text, self.require_robot)
            if text:
                self.get_logger().info(f'heard: "{text}" -> {cmd}')
            if cmd:
                self._dispatch(cmd)
        if wake_done:
            wtext = json.loads(self.wake_rec.Result()).get('text', '')
            if not cmd and self.is_face_idle:
                self._check_wake(wtext)


def main():
    rclpy.init()
    node = WakeAndCommandsNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
