#!/usr/bin/env python3
"""TTS offline (Piper; respaldo espeak-ng) con reproducción por frases y aviso de 'estoy hablando'.
- Publica tts_speaking (Bool) para que el STT se silencie mientras el robot habla.
- Sintetiza frase por frase: empieza a hablar antes de terminar de generar todo el audio.
- tts_stop (Empty) calla al robot de inmediato.
"""
import io, os, queue, subprocess, threading, wave
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String, Bool, Empty
from ament_index_python.packages import get_package_share_directory
from voice_utils import load_pron, normalize_tts, split_sentences, resample

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


def _dev(v):
    v = str(v).strip()
    return None if v == "" else (int(v) if v.lstrip("-").isdigit() else v)


class TTSNode(Node):
    def __init__(self):
        super().__init__("tts_node")
        P = self.declare_parameter
        share = get_package_share_directory("yaren_chatbot")
        P("backend", "piper")            # piper | espeak
        P("model_path", "/home/roberto/robotis_ws/src/YAREN2/yaren_chat/models/TTS/es_MX-claude-high.onnx")
        P("length_scale", 0.85)           # >1 = más lento (más claro para niños)
        P("speaker_id", -1)
        P("volume", 1.0)
        P("device", "")                  # salida de audio ("" = por defecto)
        P("output_rate", 0)              # 0 = frecuencia de la voz
        P("pron_path", os.path.join(share, "data", "pronunciaciones.json"))
        P("espeak_voice", "es-419")
        P("espeak_speed", 150)
        P("max_queue", 4)
        P("in_topic", "tts_text")
        P("speaking_topic", "tts_speaking")
        P("stop_topic", "tts_stop")
        g = lambda k: self.get_parameter(k).value

        import sounddevice as sd
        self.sd = sd
        self.backend = g("backend")
        self.volume, self.out_rate, self.dev = float(g("volume")), int(g("output_rate")), _dev(g("device"))
        self.length_scale = float(g("length_scale"))
        self.espeak_voice, self.espeak_speed = g("espeak_voice"), int(g("espeak_speed"))
        self.pron = load_pron(g("pron_path"))
        if self.backend == "piper":
            self._load_piper(os.path.expanduser(g("model_path")), int(g("speaker_id")))

        self.lock = threading.Lock()
        self.epoch, self.active = 0, 0
        self.in_q = queue.Queue(maxsize=int(g("max_queue")))
        self.play_q = queue.Queue(maxsize=3)
        self.pub_speaking = self.create_publisher(Bool, g("speaking_topic"), LATCHED)
        self.create_subscription(String, g("in_topic"), self.on_text, 10)
        self.create_subscription(Empty, g("stop_topic"), lambda _: self.stop(), 10)
        threading.Thread(target=self._synth_loop, daemon=True).start()
        threading.Thread(target=self._play_loop, daemon=True).start()
        self._set_speaking(False)
        self.get_logger().info(f"tts listo (backend={self.backend})")

    # ---- síntesis ----
    def _load_piper(self, path, speaker_id):
        from piper import PiperVoice
        if not os.path.isfile(path):
            raise RuntimeError(f"No existe la voz Piper {path}. Ejecuta scripts/descargar_modelos.sh")
        self.voice = PiperVoice.load(path)
        self.syn_cfg = None
        try:                                         # piper-tts >= 1.3
            from piper import SynthesisConfig
            self.syn_cfg = SynthesisConfig(length_scale=self.length_scale,
                                           speaker_id=None if speaker_id < 0 else speaker_id)
        except ImportError:
            pass

    def _synth_piper(self, text):
        if hasattr(self.voice, "synthesize_stream_raw"):          # piper-tts 1.2
            raw = b"".join(self.voice.synthesize_stream_raw(text, length_scale=self.length_scale))
            return np.frombuffer(raw, np.int16), self.voice.config.sample_rate
        chunks = list(self.voice.synthesize(text, syn_config=self.syn_cfg))
        if not chunks:
            return np.zeros(0, np.int16), 22050
        return np.concatenate([c.audio_int16_array for c in chunks]), chunks[0].sample_rate

    def _synth_espeak(self, text):
        p = subprocess.run(["espeak-ng", "-v", self.espeak_voice, "-s", str(self.espeak_speed), "--stdout", text],
                           capture_output=True, timeout=20)
        with wave.open(io.BytesIO(p.stdout)) as w:
            return np.frombuffer(w.readframes(w.getnframes()), np.int16), w.getframerate()

    def _synth(self, text):
        audio, rate = self._synth_piper(text) if self.backend == "piper" else self._synth_espeak(text)
        if self.out_rate and self.out_rate != rate:
            audio, rate = resample(audio, rate, self.out_rate), self.out_rate
        if self.volume != 1.0:
            audio = np.clip(audio.astype(np.float32) * self.volume, -32768, 32767).astype(np.int16)
        return audio, rate

    # ---- cola y reproducción ----
    def _set_speaking(self, v):
        self.pub_speaking.publish(Bool(data=bool(v)))

    def on_text(self, msg):
        text = normalize_tts(msg.data, self.pron)
        if not text:
            return
        with self.lock:
            self.active += 1
            first = self.active == 1
        if first:
            self._set_speaking(True)              # el STT se silencia ANTES de que suene la primera palabra
        try:
            self.in_q.put_nowait((self.epoch, text))
        except queue.Full:
            self.get_logger().warn("cola TTS llena: descarto mensaje")
            self._done_message()

    def _done_message(self):
        with self.lock:
            self.active = max(0, self.active - 1)
            idle = self.active == 0
        if idle:
            self._set_speaking(False)

    def _synth_loop(self):
        while True:
            ep, text = self.in_q.get()
            for s in split_sentences(text):
                if ep != self.epoch:
                    break
                try:
                    audio, rate = self._synth(s)
                except Exception as e:
                    self.get_logger().error(f"error de síntesis: {e}")
                    continue
                self.play_q.put((ep, audio, rate))
            self.play_q.put((ep, None, None))     # fin del mensaje

    def _play_loop(self):
        while True:
            ep, audio, rate = self.play_q.get()
            if ep != self.epoch:
                continue
            if audio is None:
                self._done_message()
                continue
            if len(audio):
                try:
                    self.sd.play(audio, rate, device=self.dev)
                    self.sd.wait()
                except Exception as e:
                    self.get_logger().error(f"error de audio: {e}")

    def stop(self):
        self.epoch += 1
        for q in (self.in_q, self.play_q):
            while not q.empty():
                try:
                    q.get_nowait()
                except queue.Empty:
                    break
        self.sd.stop()
        with self.lock:
            self.active = 0
        self._set_speaking(False)


def main():
    rclpy.init()
    node = TTSNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
