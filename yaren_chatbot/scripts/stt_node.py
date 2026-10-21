#!/usr/bin/env python3
"""STT offline (Vosk) pensado para voz de niños.
- Dos reconocedores en paralelo: LIBRE (cualquier frase) y de GRAMÁTICA (solo las palabras que el chatbot
  espera en ese momento, tópico stt_hint). Si la gramática entiende, se usa esa (más precisa en niños).
- Se silencia mientras el robot habla (tópico tts_speaking) para no oírse a sí mismo.
- Compuerta por nivel: prioriza la voz cercana (el niño) y descarta voces de fondo.
- Corrección fuzzy hacia las palabras esperadas y metadatos (stt_meta) para que el chatbot interprete con benevolencia.
- Si el niño habla pero no se entiende, publica "[unk]" (una vez seguida) para que YAREN le pida repetir.
"""
import difflib, json, os, queue, threading, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String, Bool
from voice_utils import load_vocab, build_grammar, choose_text, apply_gain, resample

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


class _CountPub:
    """Cuenta los mensajes publicados (para saber si el robot va a responder)."""
    def __init__(self, pub, owner):
        self._pub, self._owner = pub, owner

    def publish(self, msg):
        self._owner._published += 1
        self._pub.publish(msg)


def _dev(v):
    v = str(v).strip()
    return None if v == "" else (int(v) if v.lstrip("-").isdigit() else v)


class STTNode(Node):
    def __init__(self):
        super().__init__("stt_node")
        P = self.declare_parameter
        P("model_path", "/home/roberto/robotis_ws/src/YAREN2/yaren_chat/models/STT/vosk-model-small-es-0.42")
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
        P("meta_topic", "stt_meta")
        P("use_ptt", False)             # True = solo escucha mientras se mantiene el boton
        P("ptt_topic", "/yaren/ptt")
        P("listening_topic", "/stt_listening")
        P("reply_timeout", 8.0)         # s esperando respuesta del chatbot antes de volver a escuchar
        P("min_rms", 400.0)             # nivel mínimo (RMS int16) de "voz cercana"; calibra con debug_levels
        P("snr_ratio", 1.8)             # la voz cercana debe superar el piso de ruido x esto
        P("hang_frames", 6)             # frames de 100 ms que se mantiene abierta la compuerta
        P("bg_ratio", 0.6)              # fracción de frames "débiles" a partir de la cual es voz de fondo
        P("fuzzy_cutoff", 0.6)          # similitud mínima (0-1) para corregir hacia lo esperado
        P("debug_levels", False)        # imprime RMS y piso para calibrar
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
        self.rec_free.SetWords(True)
        self.rec_gram = None
        self.use_grammar = g("use_grammar")
        self.repeat_on_unknown = g("repeat_on_unknown")
        self.gain, self.in_rate, self.channels = float(g("gain")), int(g("input_rate")), int(g("channels"))
        self.post_delay, self.min_chars = float(g("post_speak_delay")), int(g("min_chars"))
        self.min_rms, self.snr_ratio = float(g("min_rms")), float(g("snr_ratio"))
        self.hang_frames, self.bg_ratio = int(g("hang_frames")), float(g("bg_ratio"))
        self.fuzzy_cutoff, self.debug_levels = float(g("fuzzy_cutoff")), bool(g("debug_levels"))
        self.floor, self._hang, self._near, self._weak = 150.0, 0, 0, 0

        self.speaking, self.unmute_at, self._was_muted = False, 0.0, False
        self._hint, self._hint_dirty, self._last_unk = [], False, False
        self.q = queue.Queue(maxsize=300)
        self._published = 0
        self.pub = _CountPub(self.create_publisher(String, g("out_topic"), 10), self)
        self.pub_meta = self.create_publisher(String, g("meta_topic"), 10)
        self.create_subscription(String, g("hint_topic"), self.on_hint, 10)
        self.create_subscription(Bool, g("speaking_topic"), self.on_speaking, 10)

        self.use_ptt = bool(g("use_ptt"))
        self.reply_timeout = float(g("reply_timeout"))
        self.ptt_held, self._ptt_recording = False, False
        self._pf, self._pg = [], []
        self._listening, self._awaiting = None, False
        self._await_until, self._seen_speaking = 0.0, False
        self.pub_listening = self.create_publisher(Bool, g("listening_topic"), 10)
        if self.use_ptt:
            self.create_subscription(Bool, g("ptt_topic"), self.on_ptt, 10)
            self.create_timer(0.1, self._tick)

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
                self.rec_gram.SetWords(True)
            except Exception as e:      # modelos grandes no soportan gramática dinámica
                self.get_logger().warn(f"gramática no disponible: {e}")
                self.rec_gram = None
                self.use_grammar = False

    def _reset(self):
        self.rec_free.Reset()
        if self.rec_gram:
            self.rec_gram.Reset()
        self._hang = self._near = self._weak = 0
        while not self.q.empty():
            try:
                self.q.get_nowait()
            except queue.Empty:
                break

    @staticmethod
    def _conf(res):
        w = res.get("result", [])
        return sum(x.get("conf", 0.0) for x in w) / len(w) if w else 0.0

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
                self._ptt_recording = False
                self._pf, self._pg = [], []
                continue
            if self._was_muted:                                    # recién volvimos a escuchar: empezar limpio
                self._was_muted = False
                self._reset()
                continue
            held = self.ptt_held
            if self.use_ptt:
                if not held and not self._ptt_recording:
                    continue                       # sin boton apretado: no se escucha nada
                if held and not self._ptt_recording:
                    self._ptt_recording = True
                    self._reset()
                    self._pf, self._pg = [], []
            pcm = np.frombuffer(data, np.int16)
            if self.channels > 1:
                pcm = pcm.reshape(-1, self.channels)[:, 0]
            pcm = resample(pcm, self.in_rate, self.rate)

            # ---- compuerta de voz cercana (prioriza al niño) ----
            rms = float(np.sqrt(np.mean(np.square(pcm.astype(np.float32))))) if len(pcm) else 0.0
            if rms < self.floor:
                self.floor = 0.8 * self.floor + 0.2 * rms
            else:
                self.floor = 0.995 * self.floor + 0.005 * rms
            near = rms >= max(self.min_rms, self.floor * self.snr_ratio)
            weak = (not near) and rms >= self.floor * 1.5
            if self.debug_levels:
                self.get_logger().info(f"rms={rms:.0f} floor={self.floor:.0f} near={near} weak={weak}")
            if near:
                self._hang, self._near = self.hang_frames, self._near + 1
            elif self._hang > 0:
                self._hang -= 1
            elif weak:
                self._weak += 1

            if near or self._hang > 0 or self.use_ptt:
                b = apply_gain(pcm, self.gain).tobytes()
            else:
                b = bytes(len(pcm) * 2)      # silencio: la voz lejana nunca llega a Vosk

            if self.use_ptt:
                if self.rec_gram and self.rec_gram.AcceptWaveform(b):
                    self._pg.append(json.loads(self.rec_gram.Result()))
                if self.rec_free.AcceptWaveform(b):
                    self._pf.append(json.loads(self.rec_free.Result()))
                if not held:
                    self._ptt_release()
                continue
            if self.rec_gram:
                self.rec_gram.AcceptWaveform(b)
            if self.rec_free.AcceptWaveform(b):
                free = json.loads(self.rec_free.Result())
                gram = json.loads(self.rec_gram.FinalResult()) if self.rec_gram else {}
                if self.rec_gram:
                    self.rec_gram.Reset()
                self._emit(free, gram)

    # ---- pulsar para hablar ----
    def on_ptt(self, msg):
        self.ptt_held = bool(msg.data)

    def _set_listening(self, v):
        if self._listening == v:
            return
        self._listening = v
        self.pub_listening.publish(Bool(data=v))

    @staticmethod
    def _merge(lst):
        words = []
        for r in lst:
            words += r.get("result", [])
        text = " ".join(r.get("text", "") for r in lst if r.get("text")).strip()
        return {"text": text, "result": words}

    def _ptt_release(self):
        self._ptt_recording = False
        self._last_unk = False
        self._set_listening(False)
        self._pf.append(json.loads(self.rec_free.FinalResult()))
        self.rec_free.Reset()
        if self.rec_gram:
            self._pg.append(json.loads(self.rec_gram.FinalResult()))
            self.rec_gram.Reset()
        free, gram = self._merge(self._pf), self._merge(self._pg)
        self._pf, self._pg = [], []
        n0 = self._published
        self._emit(free, gram)
        if self._published > n0:                   # hubo mensaje: esperar la respuesta del robot
            self._awaiting, self._seen_speaking = True, False
            self._await_until = time.time() + self.reply_timeout
        else:                                      # no se entendio nada: seguir esperando al nino
            self.get_logger().info("nada reconocido -> vuelvo a esperar")
            self._set_listening(True)

    def _tick(self):
        now = time.time()
        if self._listening is None:
            self._set_listening(True)
            return
        if self.ptt_held or self._ptt_recording:
            return
        if self.speaking:
            self._seen_speaking = True
            self._set_listening(False)
            return
        if self._awaiting:
            if (self._seen_speaking and now >= self.unmute_at) or \
               (not self._seen_speaking and now > self._await_until):
                self._awaiting = False
                self._set_listening(True)
            return
        if not self._listening and now >= self.unmute_at:
            self._set_listening(True)

    def _fuzzy(self, text):
        """Acerca lo entendido a las palabras esperadas. Devuelve (texto, fue_corregido)."""
        if not text or not self._hint:
            return text, False
        hint = [h.lower() for h in self._hint]
        t = text.lower()
        if t in hint:
            return text, False
        m = difflib.get_close_matches(t, hint, n=1, cutoff=self.fuzzy_cutoff)   # frase completa
        if m:
            return m[0], True
        out, changed = [], False
        for w in text.split():
            if w.lower() in hint:
                out.append(w)
                continue
            m = difflib.get_close_matches(w.lower(), hint, n=1, cutoff=self.fuzzy_cutoff)
            out.append(m[0] if m else w)
            changed |= bool(m)
        return " ".join(out), changed

    def _emit(self, free_res, gram_res):
        free, gram = free_res.get("text", ""), gram_res.get("text", "")
        near, weak = self._near, self._weak
        conf = max(self._conf(free_res), self._conf(gram_res))
        self._near = self._weak = 0

        # ---- único descarte: voces de fondo claras ----
        if not self.use_ptt and near == 0 and not (gram and weak > 0):      # voz suave que calza con lo esperado: se acepta
            if free or gram:
                self.get_logger().info(f"descartado (lejana): {free or gram}")
            return
        if not self.use_ptt and near and weak / (near + weak) > self.bg_ratio:
            self.get_logger().info(f"descartado (voces de fondo): {free or gram}")
            return

        text, src, unintelligible = choose_text(free, gram)
        fixed = False
        if text:
            text, fixed = self._fuzzy(text)

        if len(text) >= self.min_chars:
            self._last_unk = False
            self.get_logger().info(f"[{src}{'+fuzzy' if fixed else ''}] {text} (conf={conf:.2f})")
            self.pub_meta.publish(String(data=json.dumps(
                {"conf": round(conf, 2), "src": src, "fuzzy": fixed, "raw": free or gram},
                ensure_ascii=False)))
            self.pub.publish(String(data=text))
        elif self.repeat_on_unknown and not self._last_unk and (unintelligible or near >= 2):
            self._last_unk = True
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
