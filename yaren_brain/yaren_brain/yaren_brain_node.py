#!/usr/bin/env python3
"""
yaren_brain_node.py — Cerebro autónomo de YAREN
================================================
Nodo completamente NUEVO. No modifica ningún nodo existente.
Solo ESCUCHA topics existentes y PUBLICA en topics que ya existen.

Topics que ESCUCHA (ya existen):
  /yaren/face_idle      (Bool)   — pantalla libre o en menú
  /audio_playing        (Bool)   — TTS hablando
  /yaren/mic_owner      (String) — quién tiene el micrófono
  /yaren/is_english     (Bool)   — idioma actual
  /csi_camera/image_raw (Image)  — frames de cámara (cuando esté activa)

Topics que PUBLICA (ya existen o son nuevos inocuos):
  /yaren_mode           (String) — para activar modos
  /yaren/mic_owner      (String) — para tomar el micrófono
  /audio_playing        (Bool)   — para bloquear STT mientras habla
  /yaren/brain_highlight (String) — NUEVO: resaltar botón en pantalla (opcional)

Lo que hace:
  1. Cuando face_idle=True y ve una persona → saluda proactivamente
  2. Cuando está solo mucho tiempo → comportamiento idle curioso
  3. Cuando el ambiente está en silencio → sugiere música
  4. Presenta el menú interactivo con voz cuando el usuario pregunta qué puede hacer
"""

import os
import sys
import time
import random
import threading
import wave
import tempfile
import json

import cv2
import numpy as np
import pyaudio

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Piper TTS — mismo que usan los otros nodos
from piper import PiperVoice
from piper.config import SynthesisConfig
from playsound import playsound

# Vosk STT liviano para confirmaciones
from vosk import Model, KaldiRecognizer, SetLogLevel
SetLogLevel(-1)

# ─────────────────────────────────────────────────────────────────────────────
#  Configuración
# ─────────────────────────────────────────────────────────────────────────────

BRAIN_CHECK_INTERVAL   = 2.5   # segundos entre ciclos del bucle de vida
PERSON_COOLDOWN        = 30.0  # segundos antes de saludar a la misma persona
BOREDOM_THRESHOLD      = 90.0  # segundos solo antes de hacer algo
SUGGESTION_COOLDOWN    = 120.0 # segundos entre sugerencias ambientales
SILENCE_RMS_THRESHOLD  = 300   # nivel de ruido para considerar "silencio"
HANDSHAKE_TIMEOUT      = 6.0   # segundos esperando saludo del humano

# Rutas de modelos — mismas que usa tts_lifecycle_node.py
WS_DIR = os.path.expanduser("~/robotis_ws")

TTS_MODEL_ES  = os.path.join(WS_DIR, "src/YAREN2/yaren_chat/models/TTS/es_MX-claude-high.onnx")
TTS_CONFIG_ES = os.path.join(WS_DIR, "src/YAREN2/yaren_chat/models/TTS/es_MX-claude-high.onnx.json")
TTS_MODEL_EN  = os.path.join(WS_DIR, "src/YAREN2/yaren_chat/models/TTS/en_US-lessac-medium.onnx")
TTS_CONFIG_EN = os.path.join(WS_DIR, "src/YAREN2/yaren_chat/models/TTS/en_US-lessac-medium.onnx.json")

VOSK_MODEL_ES = os.path.join(WS_DIR, "src/YAREN2/yaren_chat/models/STT/vosk-model-small-es-0.42")
VOSK_MODEL_EN = os.path.join(WS_DIR, "src/YAREN2/yaren_chat/models/STT/vosk-model-en-us-0.22-lgraph")


# ─────────────────────────────────────────────────────────────────────────────
#  Frases por idioma
# ─────────────────────────────────────────────────────────────────────────────

PHRASES = {
    "es": {
        "greetings": [
            "¡Hola! Soy Yaren, tu amigo robot. ¡Qué bueno verte!",
            "¡Hola! Me alegra que estés aquí. Soy Yaren.",
            "¡Eh, hola! No esperaba visita. Soy Yaren, mucho gusto.",
        ],
        "handshake_ask": "¿Me das un saludo?",
        "handshake_received": "¡Genial! Es un placer conocerte.",
        "handshake_timeout": "No te preocupes, no muerdo... al menos no hoy.",
        "ask_name": "Por cierto, ¿cómo te llamas?",
        "greet_by_name": "¡{name}! Qué bueno verte de nuevo.",
        "bored_comments": [
            "Hmm... esto está muy tranquilo. ¿Hay alguien por ahí?",
            "Me pregunto qué hora será...",
            "Creo que necesito estirarme un poco.",
        ],
        "silence_suggestion": (
            "Este lugar está bastante silencioso. "
            "¿Quieres que ponga un poco de música para animar el ambiente?"
        ),
        "music_confirmed": "¡Perfecto! Pondré algo de música. ¿Quieres que baile también?",
        "music_dance_yes": "¡Bueno, aquí voy!",
        "music_dance_no": "Está bien, solo la música entonces.",
        "music_declined": "Como quieras, aquí estaré si me necesitas.",
        "menu_intro": "Claro, déjame contarte qué puedo hacer.",
        "menu_items": [
            ("yaren_chat",     "Primero está el modo Chat, donde podemos conversar de cualquier cosa."),
            ("yaren_dice",     "También tengo Yaren Dice, un juego donde tú me imitas o yo te imito a ti."),
            ("radio_musica",   "Luego está Yaren Radio, donde pongo música y hasta bailo."),
            ("yaren_emotions", "Puedo detectar tus emociones mirando tu cara."),
            ("yaren_filtros",  "Y tengo filtros divertidos, como de animales o accesorios."),
        ],
        "menu_question": "¿Cuál de estos te llama más la atención?",
        "yes_words": ["sí", "si", "yes", "dale", "pon", "claro", "va", "bueno", "ok"],
        "no_words":  ["no", "nope", "mejor no", "nah", "paso"],
    },
    "en": {
        "greetings": [
            "Hi there! I'm Yaren, your robot friend. Great to see you!",
            "Hello! I'm so glad you're here. I'm Yaren.",
            "Oh hey! Didn't expect a visitor. I'm Yaren, nice to meet you!",
        ],
        "handshake_ask": "Want to shake hands?",
        "handshake_received": "Great! It's a pleasure to meet you.",
        "handshake_timeout": "Don't worry, I don't bite... at least not today.",
        "ask_name": "By the way, what's your name?",
        "greet_by_name": "{name}! Great to see you again.",
        "bored_comments": [
            "Hmm... it's pretty quiet here. Is anyone around?",
            "I wonder what time it is...",
            "I think I need to stretch a little.",
        ],
        "silence_suggestion": (
            "It's quite silent here. "
            "Want me to play some music to liven things up?"
        ),
        "music_confirmed": "Perfect! I'll play some music. Want me to dance too?",
        "music_dance_yes": "Alright, here I go!",
        "music_dance_no": "Okay, just the music then.",
        "music_declined": "As you wish, I'll be here if you need me.",
        "menu_intro": "Sure, let me tell you what I can do.",
        "menu_items": [
            ("yaren_chat",     "First there's Chat mode, where we can talk about anything."),
            ("yaren_dice",     "I also have Yaren Says, a game where you imitate me or I imitate you."),
            ("radio_musica",   "Then there's Yaren Radio, where I play music and even dance."),
            ("yaren_emotions", "I can detect your emotions by looking at your face."),
            ("yaren_filtros",  "And I have fun filters, like animals or accessories."),
        ],
        "menu_question": "Which one sounds most interesting to you?",
        "yes_words": ["yes", "yeah", "sure", "ok", "okay", "go", "do it", "play"],
        "no_words":  ["no", "nope", "nah", "pass", "not really"],
    }
}


# ─────────────────────────────────────────────────────────────────────────────
#  Nodo principal
# ─────────────────────────────────────────────────────────────────────────────

class YarenBrainNode(Node):

    def __init__(self):
        super().__init__("yaren_brain_node")

        # ── Estado del mundo ──────────────────────────────────────────────
        self.face_idle        = False   # ¿Pantalla libre?
        self.audio_playing    = False   # ¿TTS hablando?
        self.mic_owner        = "none"  # ¿Quién tiene el mic?
        self.is_english       = False
        self.latest_frame     = None
        self._bridge          = CvBridge()
        self._frame_lock      = threading.Lock()

        # ── Estado del cerebro ────────────────────────────────────────────
        self.person_visible         = False
        self.last_person_greeted_at = 0.0
        self.last_interaction_at    = time.time()
        self.last_suggestion_at     = 0.0
        self.known_persons          = {}   # nombre → última vez visto
        self.greeting_in_progress   = False
        self.brain_active           = False  # Solo actúa cuando face_idle=True

        # ── TTS ───────────────────────────────────────────────────────────
        self._voice_es   = None
        self._voice_en   = None
        self._voice_lock = threading.Lock()
        self._load_tts()

        # ── STT liviano para confirmaciones ──────────────────────────────
        self._vosk_model  = None
        self._recognizer  = None
        self._mic         = None
        self._stream      = None
        self._load_stt()

        # ── Detección de personas (HOG liviano, sin YOLO extra) ───────────
        self._hog = cv2.HOGDescriptor()
        self._hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # ── QoS transient local para topics de estado ─────────────────────
        qos_tl = QoSProfile(depth=1,
                            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        # ── Suscripciones ─────────────────────────────────────────────────
        self.create_subscription(Bool,   "/yaren/face_idle",
                                 self._cb_face_idle,  qos_tl)
        self.create_subscription(Bool,   "/audio_playing",
                                 self._cb_audio,      10)
        self.create_subscription(String, "/yaren/mic_owner",
                                 self._cb_mic_owner,  qos_tl)
        self.create_subscription(Bool,   "/yaren/is_english",
                                 self._cb_language,   qos_tl)
        self.create_subscription(Image,  "/csi_camera/image_raw",
                                 self._cb_image,      qos_profile_sensor_data)

        # ── Publicadores ──────────────────────────────────────────────────
        self._mode_pub      = self.create_publisher(String, "/yaren_mode",        10)
        self._mic_owner_pub = self.create_publisher(String, "/yaren/mic_owner",   qos_tl)
        self._audio_pub     = self.create_publisher(Bool,   "/audio_playing",     10)
        # Topic NUEVO — face_screen puede ignorarlo si no está suscrito
        self._highlight_pub = self.create_publisher(String, "/yaren/brain_highlight", 10)

        # ── Bucle de vida ─────────────────────────────────────────────────
        self.create_timer(BRAIN_CHECK_INTERVAL, self._life_loop)

        self.get_logger().info("🧠 YarenBrainNode iniciado. Esperando que pantalla esté libre...")

    # ── Carga de modelos ──────────────────────────────────────────────────────

    def _load_tts(self):
        """Carga voces TTS en segundo plano para no bloquear el arranque."""
        def _load():
            try:
                self._voice_es = PiperVoice.load(TTS_MODEL_ES, TTS_CONFIG_ES, use_cuda=False)
                self.get_logger().info("🔊 Voz ES lista.")
            except Exception as e:
                self.get_logger().warn(f"TTS ES no disponible: {e}")
            try:
                self._voice_en = PiperVoice.load(TTS_MODEL_EN, TTS_CONFIG_EN, use_cuda=False)
                self.get_logger().info("🔊 Voz EN lista.")
            except Exception as e:
                self.get_logger().warn(f"TTS EN no disponible: {e}")
        threading.Thread(target=_load, daemon=True).start()

    def _load_stt(self):
        """Carga modelo Vosk liviano para confirmaciones (sí/no)."""
        def _load():
            path = VOSK_MODEL_EN if self.is_english else VOSK_MODEL_ES
            if not os.path.exists(path):
                # Intentar el otro idioma
                path = VOSK_MODEL_ES if not os.path.exists(VOSK_MODEL_ES) else None
            if path and os.path.exists(path):
                try:
                    self._vosk_model = Model(path)
                    self._recognizer = KaldiRecognizer(self._vosk_model, 16000)
                    self._mic = pyaudio.PyAudio()
                    self.get_logger().info("🎤 STT liviano listo.")
                except Exception as e:
                    self.get_logger().warn(f"STT no disponible: {e}")
        threading.Thread(target=_load, daemon=True).start()

    # ── Callbacks de topics ───────────────────────────────────────────────────

    def _cb_face_idle(self, msg: Bool):
        was_idle = self.face_idle
        self.face_idle = msg.data
        if msg.data and not was_idle:
            self.get_logger().info("🟢 Pantalla libre — cerebro activo.")
            self.brain_active = True
        elif not msg.data:
            self.brain_active = False

    def _cb_audio(self, msg: Bool):
        self.audio_playing = msg.data
        if not msg.data:
            self.last_interaction_at = time.time()

    def _cb_mic_owner(self, msg: String):
        self.mic_owner = msg.data

    def _cb_language(self, msg: Bool):
        self.is_english = msg.data

    def _cb_image(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.flip(frame, 0)
            with self._frame_lock:
                self.latest_frame = frame
        except Exception:
            pass

    # ── Bucle de vida principal ───────────────────────────────────────────────

    def _life_loop(self):
        """
        Corre cada BRAIN_CHECK_INTERVAL segundos.
        Solo actúa si la pantalla está libre y nadie está hablando.
        """
        if not self.brain_active:
            return
        if self.audio_playing:
            return
        if self.greeting_in_progress:
            return
        if self.mic_owner not in ("none", "brain"):
            return

        now = time.time()

        # 1. Detectar personas en el frame actual
        person_now = self._detect_person()

        # ── PRIORIDAD 1: Nueva persona detectada ─────────────────────────
        if person_now and not self.person_visible:
            since_last = now - self.last_person_greeted_at
            if since_last > PERSON_COOLDOWN:
                self.person_visible = True
                threading.Thread(target=self._greet_sequence, daemon=True).start()
                return

        self.person_visible = person_now

        # ── PRIORIDAD 2: Silencio prolongado → sugerir música ────────────
        if person_now:
            since_interaction = now - self.last_interaction_at
            since_suggestion  = now - self.last_suggestion_at
            silence = self._measure_silence()

            if (silence
                    and since_interaction > 60.0
                    and since_suggestion > SUGGESTION_COOLDOWN):
                self.last_suggestion_at = now
                threading.Thread(target=self._silence_suggestion,
                                 daemon=True).start()
                return

        # ── PRIORIDAD 3: Aburrido y solo ─────────────────────────────────
        if not person_now:
            since_interaction = now - self.last_interaction_at
            if since_interaction > BOREDOM_THRESHOLD:
                self.last_interaction_at = now  # resetear para no spamear
                threading.Thread(target=self._bored_behavior,
                                 daemon=True).start()

    # ── Detección de persona (HOG, sin YOLO extra) ────────────────────────────

    def _detect_person(self) -> bool:
        with self._frame_lock:
            frame = self.latest_frame
        if frame is None:
            return False
        try:
            small = cv2.resize(frame, (320, 240))
            rects, _ = self._hog.detectMultiScale(
                small, winStride=(8, 8), padding=(4, 4), scale=1.05)
            return len(rects) > 0
        except Exception:
            return False

    # ── Medición de silencio ambiental ────────────────────────────────────────

    def _measure_silence(self) -> bool:
        """Lee 0.3 segundos del micrófono y calcula RMS."""
        if self._mic is None or self.mic_owner != "none":
            return False
        try:
            stream = self._mic.open(
                format=pyaudio.paInt16, channels=1, rate=16000,
                input=True, frames_per_buffer=4800)
            data = stream.read(4800, exception_on_overflow=False)
            stream.stop_stream()
            stream.close()
            samples = np.frombuffer(data, dtype=np.int16).astype(np.float32)
            rms = float(np.sqrt(np.mean(samples ** 2)))
            return rms < SILENCE_RMS_THRESHOLD
        except Exception:
            return False

    # ── TTS del brain (no usa los lifecycle nodes, habla directamente) ─────────

    def _speak(self, text: str):
        """Síntesis y reproducción directa, bloqueante."""
        voice = self._voice_en if self.is_english else self._voice_es
        if voice is None:
            self.get_logger().warn(f"[BRAIN] Sin voz TTS. Texto: {text}")
            return

        # Avisar al sistema que hay audio
        msg = Bool(); msg.data = True
        self._audio_pub.publish(msg)

        syn_cfg = SynthesisConfig(length_scale=1.1, noise_scale=0.5, noise_w_scale=0.8)
        try:
            with self._voice_lock:
                with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as fp:
                    tmp = fp.name
                with wave.open(tmp, "wb") as wf:
                    wf.setnchannels(1)
                    wf.setsampwidth(2)
                    wf.setframerate(voice.config.sample_rate)
                    voice.synthesize_wav(text, wf, syn_config=syn_cfg)
            playsound(tmp)
            os.unlink(tmp)
        except Exception as e:
            self.get_logger().error(f"[BRAIN] Error TTS: {e}")
        finally:
            msg.data = False
            self._audio_pub.publish(msg)

    # ── Escuchar confirmación corta (sí/no) ───────────────────────────────────

    def _listen_for_confirmation(self, timeout: float = 5.0) -> str:
        """
        Escucha brevemente y retorna 'yes', 'no' o 'timeout'.
        Toma el micrófono solo durante este proceso.
        """
        if self._vosk_model is None or self._mic is None:
            return "timeout"

        # Tomar el micrófono
        owner_msg = String(); owner_msg.data = "brain"
        self._mic_owner_pub.publish(owner_msg)
        time.sleep(0.2)

        lang = "en" if self.is_english else "es"
        yes_words = PHRASES[lang]["yes_words"]
        no_words  = PHRASES[lang]["no_words"]

        result = "timeout"
        try:
            rec = KaldiRecognizer(self._vosk_model, 16000)
            stream = self._mic.open(
                format=pyaudio.paInt16, channels=1, rate=16000,
                input=True, frames_per_buffer=4000)
            deadline = time.time() + timeout
            while time.time() < deadline:
                data = stream.read(4000, exception_on_overflow=False)
                if rec.AcceptWaveform(data):
                    text = json.loads(rec.Result()).get("text", "").lower()
                    if any(w in text for w in yes_words):
                        result = "yes"
                        break
                    if any(w in text for w in no_words):
                        result = "no"
                        break
            stream.stop_stream()
            stream.close()
        except Exception as e:
            self.get_logger().warn(f"[BRAIN] STT error: {e}")

        # Devolver el micrófono
        owner_msg.data = "none"
        self._mic_owner_pub.publish(owner_msg)
        return result

    # ── Gesto: levantar / bajar mano ─────────────────────────────────────────

    def _publish_mode(self, mode: str):
        msg = String(); msg.data = mode
        self._mode_pub.publish(msg)

    def _highlight_button(self, mode_id: str):
        """Publica el id del botón a resaltar. face_screen puede ignorarlo."""
        msg = String(); msg.data = mode_id
        self._highlight_pub.publish(msg)

    # ── Secuencia de saludo ───────────────────────────────────────────────────

    def _greet_sequence(self):
        """
        Saludo completo:
        1. Saludar con voz
        2. Levantar mano (modo especial de movimiento)
        3. Esperar saludo del humano
        4. Bajar mano y continuar conversación
        """
        self.greeting_in_progress = True
        self.last_person_greeted_at = time.time()
        lang = "en" if self.is_english else "es"
        phrases = PHRASES[lang]

        try:
            # 1. Saludo inicial
            greeting = random.choice(phrases["greetings"])
            self._speak(greeting)

            # 2. Levantar mano derecha
            #    Publicamos el modo especial de movimiento "brain_wave"
            #    Este modo lo puedes mapear en face_screen o ignorar.
            #    Por ahora usamos el topic de trayectoria directamente.
            self._raise_right_hand()
            time.sleep(0.5)

            # 3. Pedir saludo
            self._speak(phrases["handshake_ask"])

            # 4. Esperar respuesta o movimiento
            response = self._listen_for_confirmation(HANDSHAKE_TIMEOUT)

            # 5. Bajar mano
            self._lower_right_hand()

            # 6. Reaccionar
            if response in ("yes", "timeout"):
                # "yes" = dijo algo / "timeout" = pasó tiempo (asumimos saludo físico)
                self._speak(phrases["handshake_received"])
            else:
                self._speak(phrases["handshake_timeout"])

            time.sleep(0.5)

            # 7. Preguntar nombre si no lo conocemos
            self._speak(phrases["ask_name"])
            name = self._listen_for_name(timeout=8.0)

            if name:
                self.known_persons[name] = time.time()
                self.get_logger().info(f"🧠 Persona registrada: {name}")

            self.last_interaction_at = time.time()

        except Exception as e:
            self.get_logger().error(f"[BRAIN] Error en saludo: {e}")
        finally:
            self.greeting_in_progress = False

    def _listen_for_name(self, timeout: float = 8.0) -> str:
        """Escucha un nombre (cualquier palabra que no sea sí/no)."""
        if self._vosk_model is None:
            return ""

        owner_msg = String(); owner_msg.data = "brain"
        self._mic_owner_pub.publish(owner_msg)
        time.sleep(0.2)

        name = ""
        try:
            rec = KaldiRecognizer(self._vosk_model, 16000)
            stream = self._mic.open(
                format=pyaudio.paInt16, channels=1, rate=16000,
                input=True, frames_per_buffer=4000)
            deadline = time.time() + timeout
            while time.time() < deadline:
                data = stream.read(4000, exception_on_overflow=False)
                if rec.AcceptWaveform(data):
                    text = json.loads(rec.Result()).get("text", "").strip()
                    if text:
                        # Limpiar palabras comunes y quedarse con el nombre
                        stopwords = {"me", "llamo", "soy", "my", "name", "is", "i'm", "im"}
                        words = [w for w in text.split() if w not in stopwords]
                        if words:
                            name = words[-1].capitalize()
                            break
            stream.stop_stream()
            stream.close()
        except Exception as e:
            self.get_logger().warn(f"[BRAIN] STT nombre error: {e}")

        owner_msg.data = "none"
        self._mic_owner_pub.publish(owner_msg)
        return name

    # ── Sugerencia de música por silencio ─────────────────────────────────────

    def _silence_suggestion(self):
        lang = "en" if self.is_english else "es"
        phrases = PHRASES[lang]

        self._speak(phrases["silence_suggestion"])
        response = self._listen_for_confirmation(timeout=7.0)

        if response == "yes":
            self._speak(phrases["music_confirmed"])
            dance_response = self._listen_for_confirmation(timeout=5.0)

            if dance_response == "yes":
                self._speak(phrases["music_dance_yes"])
                # Activar radio con baile (ya implementado en tu sistema)
                self._publish_mode("radio_musica")
                # El yaren_dance_radio.py se activa automáticamente con la música
            else:
                self._speak(phrases["music_dance_no"])
                self._publish_mode("radio_musica")
        else:
            self._speak(phrases["music_declined"])

        self.last_interaction_at = time.time()

    # ── Comportamiento cuando está solo y aburrido ────────────────────────────

    def _bored_behavior(self):
        lang = "en" if self.is_english else "es"
        phrases = PHRASES[lang]

        behavior = random.choice(["comment", "look_around", "stretch"])

        if behavior == "comment":
            comment = random.choice(phrases["bored_comments"])
            self._speak(comment)

        elif behavior == "look_around":
            # Mover la cabeza buscando — usar topic de trayectoria existente
            self._look_around_sequence()

        elif behavior == "stretch":
            # Estirar brazos
            self._stretch_sequence()

    # ── Presentación interactiva del menú ─────────────────────────────────────

    def present_menu(self):
        """
        Llamar desde el exterior (o cuando el usuario pregunte qué puede hacer).
        Presenta cada modo con voz y resalta el botón correspondiente.
        """
        lang = "en" if self.is_english else "es"
        phrases = PHRASES[lang]

        self._speak(phrases["menu_intro"])
        time.sleep(0.3)

        for mode_id, description in phrases["menu_items"]:
            # Resaltar botón en pantalla
            self._highlight_button(mode_id)
            # Describir con voz
            self._speak(description)
            time.sleep(0.4)

            # ¿El usuario interrumpió? (detección rápida)
            # Por ahora dejamos que el ciclo completo termine
            # En la siguiente iteración se puede agregar interrupción

        self._speak(phrases["menu_question"])
        self.last_interaction_at = time.time()

    # ── Movimientos físicos (publican al joint_trajectory_controller) ──────────

    def _raise_right_hand(self):
        """Levanta el brazo derecho."""
        try:
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
            from builtin_interfaces.msg import Duration

            if not hasattr(self, "_traj_pub"):
                self._traj_pub = self.create_publisher(
                    JointTrajectory,
                    "/joint_trajectory_controller/joint_trajectory", 10)

            msg = JointTrajectory()
            msg.joint_names = [
                "joint_1","joint_2","joint_3","joint_4",
                "joint_5","joint_6","joint_7","joint_8",
                "joint_9","joint_10","joint_11","joint_12"
            ]
            pt = JointTrajectoryPoint()
            # Brazo derecho arriba (joint_5=3.0), izquierdo en reposo
            pt.positions = [0.0, 0.0, 0.0, 0.0,
                            3.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.5]
            pt.velocities = [0.0] * 12
            pt.time_from_start = Duration(sec=2, nanosec=0)
            msg.points = [pt]
            self._traj_pub.publish(msg)
            time.sleep(2.2)
        except Exception as e:
            self.get_logger().warn(f"[BRAIN] Error levantando mano: {e}")

    def _lower_right_hand(self):
        """Baja el brazo derecho a posición de reposo."""
        try:
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
            from builtin_interfaces.msg import Duration

            msg = JointTrajectory()
            msg.joint_names = [
                "joint_1","joint_2","joint_3","joint_4",
                "joint_5","joint_6","joint_7","joint_8",
                "joint_9","joint_10","joint_11","joint_12"
            ]
            pt = JointTrajectoryPoint()
            pt.positions = [0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.5,
                            0.0, 0.0, 0.0, 0.5]
            pt.velocities = [0.0] * 12
            pt.time_from_start = Duration(sec=2, nanosec=0)
            msg.points = [pt]
            self._traj_pub.publish(msg)
            time.sleep(2.0)
        except Exception as e:
            self.get_logger().warn(f"[BRAIN] Error bajando mano: {e}")

    def _look_around_sequence(self):
        """Mueve la cabeza de lado a lado buscando."""
        try:
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
            from builtin_interfaces.msg import Duration

            if not hasattr(self, "_traj_pub"):
                self._traj_pub = self.create_publisher(
                    JointTrajectory,
                    "/joint_trajectory_controller/joint_trajectory", 10)

            def _send(j3_val):
                msg = JointTrajectory()
                msg.joint_names = [
                    "joint_1","joint_2","joint_3","joint_4",
                    "joint_5","joint_6","joint_7","joint_8",
                    "joint_9","joint_10","joint_11","joint_12"
                ]
                pt = JointTrajectoryPoint()
                pt.positions = [0.0, 0.0, j3_val, 0.2,
                                0.0, 0.0, 0.0, 0.5,
                                0.0, 0.0, 0.0, 0.5]
                pt.velocities = [0.0] * 12
                pt.time_from_start = Duration(sec=1, nanosec=500000000)
                msg.points = [pt]
                self._traj_pub.publish(msg)
                time.sleep(1.8)

            _send(-0.5)   # izquierda
            _send( 0.5)   # derecha
            _send( 0.0)   # centro

        except Exception as e:
            self.get_logger().warn(f"[BRAIN] Error mirando alrededor: {e}")

    def _stretch_sequence(self):
        """Estira ambos brazos."""
        try:
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
            from builtin_interfaces.msg import Duration

            if not hasattr(self, "_traj_pub"):
                self._traj_pub = self.create_publisher(
                    JointTrajectory,
                    "/joint_trajectory_controller/joint_trajectory", 10)

            # Brazos arriba
            msg = JointTrajectory()
            msg.joint_names = [
                "joint_1","joint_2","joint_3","joint_4",
                "joint_5","joint_6","joint_7","joint_8",
                "joint_9","joint_10","joint_11","joint_12"
            ]
            pt = JointTrajectoryPoint()
            pt.positions = [0.0, 0.0, 0.0, 0.0,
                            3.0, 0.0, 0.0, 0.0,
                           -3.0, 0.0, 0.0, 0.0]
            pt.velocities = [0.0] * 12
            pt.time_from_start = Duration(sec=2, nanosec=0)
            msg.points = [pt]
            self._traj_pub.publish(msg)
            time.sleep(2.5)

            # Bajar
            pt.positions = [0.0]*4 + [0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5]
            pt.time_from_start = Duration(sec=2, nanosec=0)
            msg.points = [pt]
            self._traj_pub.publish(msg)
            time.sleep(2.0)

        except Exception as e:
            self.get_logger().warn(f"[BRAIN] Error estirándose: {e}")


# ─────────────────────────────────────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = YarenBrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()