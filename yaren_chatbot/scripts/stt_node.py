#!/usr/bin/env python3
"""STT offline (Vosk) pensado para voz de niños.
- Dos reconocedores en paralelo: LIBRE (cualquier frase) y de GRAMÁTICA (solo las palabras que el chatbot
  espera en ese momento, tópico stt_hint). Si la gramática entiende, se usa esa (más precisa en niños).
- Se silencia mientras el robot habla (tópico tts_speaking) para no oírse a sí mismo.
- Si el niño habla pero no se entiende, publica "[unk]" (una vez seguida) para que YAREN le pida repetir.
"""
import json, os, queue, threading, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String, Bool
from voice_utils import load_vocab, build_grammar, choose_text, apply_gain, resample

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


def _dev(v):
    v = str(v).strip()
    return None if v == "" else (int(v) if v.lstrip("-").isdigit() else v)


class STTNode(Node):
    def __init__(self):
        super().__init__("stt_node")
        P = self.declare_parameter
        P("model_path", "~/yaren_models/vosk-model-small-es-0.42")
        P("device", "")                 # índice o nombre del micrófono ("" = por defecto)
        P("channels", 1)
        P("input_rate", 16000)          # frecuencia nativa del micrófono
        P("gain", 1.0)                  # súbelo (1.5–3.0) si los niños hablan bajito
        P("use_grammar", True)
        P("repeat_on_unknown", True)
        P("post_speak_delay", 0.6)      # segundos de silencio extra tras hablar el robot
        P("min_chars", 2)
        P("list_devices", False)
        P("out_topic", "stt_text")
        P("hint_topic", "stt_hint")
        P("speaking_topic", "tts_speaking")
        g = lambda k: self.get_parameter(k).value

        import sounddevice as sd, vosk
        vosk.SetLogLevel(-1)
        if g("list_devices"):
            self.get_logger().info("\n" + str(sd.query_devices()))
        mp = os.path.expanduser(g("model_path"))
        if not os.path.isdir(mp):
            raise RuntimeError(f"No existe el modelo Vosk en {mp}. Ejecuta scripts/descargar_modelos.sh")
        self.vosk = vosk
        self.rate = 16000
        self.model = vosk.Model(mp)
        self.vocab = load_vocab(mp)
        self.rec_free = vosk.KaldiRecognizer(self.model, self.rate)
        self.rec_gram = None
        self.use_grammar = g("use_grammar")
        self.repeat_on_unknown = g("repeat_on_unknown")
        self.gain, self.in_rate, self.channels = float(g("gain")), int(g("input_rate")), int(g("channels"))
        self.post_delay, self.min_chars = float(g("post_speak_delay")), int(g("min_chars"))

        self.speaking, self.unmute_at, self._was_muted = False, 0.0, False
        self._hint, self._hint_dirty, self._last_unk = [], False, False
        self.q = queue.Queue(maxsize=300)
        self.pub = self.create_publisher(String, g("out_topic"), 10)
        self.create_subscription(String, g("hint_topic"), self.on_hint, 10)
        self.create_subscription(Bool, g("speaking_topic"), self.on_speaking, LATCHED)

        self.run = True
        threading.Thread(target=self._loop, daemon=True).start()
        self.stream = sd.RawInputStream(samplerate=self.in_rate, blocksize=int(self.in_rate * 0.1),
                                        device=_dev(g("device")), dtype="int16", channels=self.channels,
                                        callback=self._audio_cb)
        self.stream.start()
        self.get_logger().info(f"stt listo (vocabulario: {len(self.vocab)} palabras, gramática={self.use_grammar})")

    # ---- callbacks ROS ----
    def on_hint(self, msg):
        try:
            hint = [str(w) for w in json.loads(msg.data)]
        except Exception:
            return
        if hint != self._hint:
            self._hint, self._hint_dirty = hint, True

    def on_speaking(self, msg):
        self.speaking = bool(msg.data)
        if not self.speaking:
            self.unmute_at = time.time() + self.post_delay
        self._was_muted = True

    def _audio_cb(self, indata, frames, t, status):
        try:
            self.q.put_nowait(bytes(indata))
        except queue.Full:
            pass

    # ---- procesamiento (hilo propio) ----
    def _apply_hint(self):
        self._hint_dirty = False
        self.rec_gram = None
        if not (self.use_grammar and self._hint):
            return
        grammar = build_grammar(self._hint, self.vocab)
        if grammar:
            try:
                self.rec_gram = self.vosk.KaldiRecognizer(self.model, self.rate, grammar)
            except Exception as e:      # modelos grandes no soportan gramática dinámica
                self.get_logger().warn(f"gramática no disponible: {e}")
                self.use_grammar = False

    def _reset(self):
        self.rec_free.Reset()
        if self.rec_gram:
            self.rec_gram.Reset()
        while not self.q.empty():
            try:
                self.q.get_nowait()
            except queue.Empty:
                break

    def _loop(self):
        while self.run:
            if self._hint_dirty:
                self._apply_hint()
            try:
                data = self.q.get(timeout=0.2)
            except queue.Empty:
                continue
            if self.speaking or time.time() < self.unmute_at:      # el robot habla: ignorar audio
                self._was_muted = True
                continue
            if self._was_muted:                                    # recién volvimos a escuchar: empezar limpio
                self._was_muted = False
                self._reset()
                continue
            pcm = np.frombuffer(data, np.int16)
            if self.channels > 1:
                pcm = pcm.reshape(-1, self.channels)[:, 0]
            pcm = apply_gain(resample(pcm, self.in_rate, self.rate), self.gain)
            b = pcm.tobytes()
            if self.rec_gram:
                self.rec_gram.AcceptWaveform(b)
            if self.rec_free.AcceptWaveform(b):
                free = json.loads(self.rec_free.Result()).get("text", "")
                gram = json.loads(self.rec_gram.FinalResult()).get("text", "") if self.rec_gram else ""
                if self.rec_gram:
                    self.rec_gram.Reset()
                self._emit(free, gram)

    def _emit(self, free, gram):
        text, src, unintelligible = choose_text(free, gram)
        if len(text) >= self.min_chars:
            self._last_unk = False
            self.get_logger().info(f"[{src}] {text}")
            self.pub.publish(String(data=text))
        elif unintelligible and self.repeat_on_unknown and not self._last_unk:
            self._last_unk = True           # una sola vez seguida, para no repetir sin parar
            self.get_logger().info("habló pero no se entendió -> [unk]")
            self.pub.publish(String(data="[unk]"))

    def destroy_node(self):
        self.run = False
        try:
            self.stream.stop()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = STTNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
