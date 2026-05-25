#!/usr/bin/env python3
"""
yaren_voice_menu.py
===================
Nodo ROS2 que replica el árbol de menús del face_screen (C++) pero por voz.

Flujo:
  1. Escucha /yaren/face_idle → solo actúa cuando la pantalla está en reposo
  2. Wake word "despierta" → saluda y entra al menú principal
  3. STT continuo con Vosk para entender comandos del usuario
  4. TTS con Piper local (sin internet) para responder
  5. Ejecuta los mismos comandos del menú visual (os.system + /yaren_mode)

Topics ROS2:
  SUB  /yaren/face_idle      (std_msgs/Bool)   - pantalla libre o en menú
  SUB  /yaren/wake_event     (std_msgs/Bool)   - wake word ya detectado
  PUB  /yaren_mode           (std_msgs/String) - activa modos (igual que click)
  PUB  /audio_playing        (std_msgs/Bool)   - pausa el TTS del chat
  PUB  /yaren/wake_event     (std_msgs/Bool)   - para abrir el menú visual también
"""

import os
import sys
import json
import wave
import time
import queue
import tempfile
import threading
import subprocess
import random

import pyaudio
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String

# ─── Vosk STT ────────────────────────────────────────────────────────────────
from vosk import Model, KaldiRecognizer

# ─── Piper TTS ───────────────────────────────────────────────────────────────
from piper import PiperVoice
from piper.config import SynthesisConfig
from playsound import playsound

from ament_index_python.packages import get_package_share_directory


# =============================================================================
#  Árbol de menús  (espejo exacto del C++)
# =============================================================================

# Cada entrada del menú tiene:
#   "label"     → palabras clave que el usuario puede decir
#   "speak"     → lo que dice Yaren al entrar / confirmar
#   "cmd"       → comando de sistema a ejecutar (igual que en C++)
#   "mode_id"   → id que se publica en /yaren_mode
#   "stop_cmd"  → comando para detener el modo (igual que en C++)
#   "children"  → sub-menú (lista de ítems, vacía si es hoja)
#   "ask"       → pregunta que hace Yaren después de listar opciones

MENU_TREE = {
    "speak_welcome": (
        "¡Hola! Soy Yaren, tu amigo virtual. ¿En qué puedo ayudarte hoy?"
    ),
    "speak_welcome_back": (
        "¡Hola de nuevo! ¿En qué te puedo ayudar ahora?"
    ),
    "speak_what_can_i_do": (
        "Claro, puedes hacer muchas cosas. "
        "Puedes conversar conmigo, detectar tus emociones, escuchar música, "
        "ver videos, jugar al Yaren Dice, ponerte filtros de animales, "
        "ponerte filtros de accesorios, ponerte un fondo virtual "
        "o grabar una rutina personal. ¿Qué te gustaría hacer?"
    ),
    "items": [
        {
            "label": ["chat", "conversar", "hablar", "conversa", "habla", "charlar", "platicar", "chatear", "hablemos", "conversacion", "modo chat", "inicia el chat", "dialogar", "hablame", "contame algo"],
            "speak": "¡Genial! Iniciando el modo chat. ¡Cuéntame todo!",
            "cmd":     "ros2 launch yaren_chat yaren_chat.launch.py &",
            "mode_id": "yaren_chat",
            "stop_cmd": "for pid in $(ps aux | grep -E 'yaren_chat' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done",
            "children": [],
        },
        {
            "label": ["emociones", "emocion", "detectar emocion", "detecta", "sentimientos", "como me siento", "como estoy", "analiza mi cara", "lee mis emociones", "estado de animo", "detectar sentimientos", "emociones faciales", "que emocion tengo"],
            "speak": "Activando detección de emociones. ¡Muéstrame tu cara!",
            "cmd":     "ros2 launch yaren_emotions yaren_emotions.launch.py &",
            "mode_id": "yaren_emotions",
            "stop_cmd": "for pid in $(ps aux | grep -E 'yaren_emotions' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done",
            "children": [],
        },
        {
            "label": ["musica", "música", "canciones", "radio", "escuchar musica", "pon musica", "reproduce", "quiero musica", "pon una cancion", "musiquita", "escuchar canciones", "radio yaren", "poner musica", "escucha", "musica por favor", "play", "bailar"],
            "speak": "¡Buena elección! Tengo varias canciones disponibles. "
                     "Puedes pedir: Ara Ram Sam Sam, Baile del Gorila, Barney, "
                     "Chopi Chopi, Libre Soy, Sa Sa, o Si Tienes Ganas. "
                     "¿Cuál quieres escuchar? También puedes decir cualquiera para una sorpresa.",
            "cmd":     "",
            "mode_id": "",
            "stop_cmd": "",
            "children": [
                {
                    "label": ["aramsamsam", "ara ram sam sam", "luli", "ram sam", "aramsam", "cancion de luli", "luli pampin", "ara ram"],
                    "speak": "¡Reproduciendo Ara Ram Sam Sam!",
                    "cmd":     "",
                    "mode_id": "radio_musica",
                    "stop_cmd": "",
                    "song_index": 0,
                    "children": [],
                },
                {
                    "label": ["gorila", "baile del gorila", "cantajuego", "el gorila", "baile gorila", "canta juego", "gorila baila"],
                    "speak": "¡Reproduciendo el Baile del Gorila!",
                    "cmd":     "",
                    "mode_id": "radio_musica",
                    "stop_cmd": "",
                    "song_index": 1,
                    "children": [],
                },
                {
                    "label": ["barney", "dinosaurio", "barney el dinosaurio", "tema de barney", "intro barney", "dino", "barney y sus amigos"],
                    "speak": "¡Reproduciendo el intro de Barney!",
                    "cmd":     "",
                    "mode_id": "radio_musica",
                    "stop_cmd": "",
                    "song_index": 2,
                    "children": [],
                },
                {
                    "label": ["chopi chopi", "chipi chipi", "chapa chapa", "christell", "dubidubidu", "chipi chipi chapa chapa", "dubi dubi daba daba", "cancion de christell"],
                    "speak": "¡Reproduciendo Chopi Chopi!",
                    "cmd":     "",
                    "mode_id": "radio_musica",
                    "stop_cmd": "",
                    "song_index": 3,
                    "children": [],
                },
                {
                    "label": ["libre soy", "frozen", "martina", "libre soi", "frozen cancion", "martina stoessel", "cancion de frozen"],
                    "speak": "¡Reproduciendo Libre Soy!",
                    "cmd":     "",
                    "mode_id": "radio_musica",
                    "stop_cmd": "",
                    "song_index": 4,
                    "children": [],
                },
                {
                    "label": ["sasa", "sa sa", "serpiente", "sa sa la serpiente", "la serpiente", "cancion de la serpiente"],
                    "speak": "¡Reproduciendo Sa Sa La Serpiente!",
                    "cmd":     "",
                    "mode_id": "radio_musica",
                    "stop_cmd": "",
                    "song_index": 5,
                    "children": [],
                },
                {
                    "label": ["si tienes ganas", "tienes ganas", "aplaudir", "si tu tienes ganas", "ganas de aplaudir", "aplaude", "cancion de aplaudir"],
                    "speak": "¡Reproduciendo Si Tienes Ganas!",
                    "cmd":     "",
                    "mode_id": "radio_musica",
                    "stop_cmd": "",
                    "song_index": 6,
                    "children": [],
                },
               {
                    "label": ["cualquiera", "sorpresa", "pon cualquiera", "la que sea", "aleatoria", "aleatorio", "al azar", "elige tu", "ponme una cualquiera", "sorpresa musical", "random", "reproduce cualquier cosa", "pon cualquier cosa", "cualquier cancion", "cualquier musica", "lo que sea"],
                    "speak": "¡De acuerdo! Reproduciré una canción al azar.",
                    "cmd":     "",
                    "mode_id": "radio_musica",
                    "stop_cmd": "",
                    "song_index": -1,
                    "children": [],
                },]
        },
        {
            "label": ["videos", "ver videos", "video", "canciones animadas", "ver un video", "reproduce un video", "videos para niños", "animacion", "videos infantiles", "pon un video", "mirar videos", "ver dibujos", "caricaturas"],
            "speak": "¡Claro! Tengo varios videos disponibles. "
                     "Puedes pedir: Pollito Pío, Gallina Turuleca, La Vaca Lola o Susanita. "
                     "¿Cuál quieres ver? También puedes decir cualquiera para una sorpresa.",
            "cmd":     "",
            "mode_id": "",
            "stop_cmd": "",
            "children": [
                {
                    "label": ["pollito", "pollito pio", "pío", "granja", "el pollito pio", "pio pio", "pollito pío", "granja de pollito"],
                    "speak": "¡Reproduciendo Pollito Pío!",
                    "cmd":     "python3 src/YAREN2/yaren_radio/yaren_radio/pollitopio.py &",
                    "mode_id": "vid_pollito",
                    "stop_cmd": "for pid in $(ps aux | grep -E 'pollitopio.py' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done",
                    "children": [],
                },
                {
                    "label": ["gallina turuleca", "turuleca", "gallina", "la gallina turuleca", "gallina loca", "turuleca cancion"],
                    "speak": "¡Reproduciendo la Gallina Turuleca!",
                    "cmd":     "python3 src/YAREN2/yaren_radio/yaren_radio/gallinaturuleca.py &",
                    "mode_id": "vid_gallina",
                    "stop_cmd": "for pid in $(ps aux | grep -E 'gallinaturuleca.py' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done",
                    "children": [],
                },
                {
                    "label": ["vaca lola", "vaca", "lola", "la vaca lola", "vaca lola vaca lola", "cancion de la vaca"],
                    "speak": "¡Reproduciendo La Vaca Lola!",
                    "cmd":     "python3 src/YAREN2/yaren_radio/yaren_radio/vacalola.py &",
                    "mode_id": "vid_vaca",
                    "stop_cmd": "for pid in $(ps aux | grep -E 'vacalola.py' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done",
                    "children": [],
                },
                {
                    "label": ["susanita", "zenon", "granja de zenon", "susanita tiene un raton", "zenon granja", "susanita cancion"],
                    "speak": "¡Reproduciendo Susanita!",
                    "cmd":     "python3 src/YAREN2/yaren_radio/yaren_radio/susanita.py &",
                    "mode_id": "vid_susanita",
                    "stop_cmd": "for pid in $(ps aux | grep -E 'susanita.py' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done",
                    "children": [],
                },
                {
                    "label": ["un video cualquiera", "pon un video cualquiera", "cualquier video", "video aleatorio", "un video al azar", "video sorpresa", "reproduce un video cualquiera", "reproduce cualquier video"],
                    "speak": "¡De acuerdo! Preparando un video sorpresa para ti.",
                    "cmd": "RANDOM_VIDEO",
                    "mode_id": "",
                    "stop_cmd": "",
                    "children": [],
                },
            ],
        },
        {
            "label": ["yaren dice", "yo digo", "simon dice", "jugar", "juguemos", "inicia yaren dice", "juego de imitacion", "juego de repetir", "jugamos", "modo juego", "simon dice juego"],
            "speak": "¡Vamos a jugar! Iniciando Yaren Dice.",
            "cmd":     "ros2 launch yaren_dice yaren_dice.launch.py &",
            "mode_id": "yaren_dice",
            "stop_cmd": "for pid in $(ps aux | grep -E 'yaren_dice' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done",
            "children": [],
        },
        {
            "label": ["filtro animales", "animales", "filtro animal", "modo animales", "ponme de animal", "cara de animal", "filtro de animalitos", "transforma en animal", "mascota", "animalitos"],
            "speak": "¡Activando filtro de animales! ¡Mira qué gracioso quedas!",
            "cmd":     "ros2 launch yaren_filters yaren_animales.launch.py &",
            "mode_id": "yaren_animales",
            "stop_cmd": "for pid in $(ps aux | grep -E 'yaren_animales|AnimalFaceNode|animal_filter|face_landmark|csi_cam' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done",
            "children": [],
        },
        {
            "label": ["filtro accesorios", "accesorios", "filtro accesorio", "sombrero", "gafas", "modo accesorios", "ponme gafas", "lentes", "anteojos", "sombreros", "gorro", "accesorios para la cara", "ponme un sombrero", "filtros divertidos"],
            "speak": "¡Activando filtro de accesorios!",
            "cmd":     "ros2 launch yaren_filters yaren_accesorios.launch.py &",
            "mode_id": "yaren_accesorios",
            "stop_cmd": "for pid in $(ps aux | grep -E 'yaren_accesorios|face_filter|face_landmark|csi_cam' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done",
            "children": [],
        },
        {
            "label": ["fondo virtual", "fondo", "fondo magico", "background", "cambia el fondo", "pon un fondo", "fondo nuevo", "cambiar escenario", "fondo de pantalla", "modo fondo", "entorno virtual"],
            "speak": "¡Activando fondo virtual!",
            "cmd":     "ros2 launch yaren_filters yaren_fondo.launch.py &",
            "mode_id": "yaren_fondo",
            "stop_cmd": "for pid in $(ps aux | grep -E 'yaren_fondo|fondo_virtual|csi_cam' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done",
            "children": [],
        },
        {
            "label": ["rutina", "rutinas", "grabar rutina", "rutina personal", "movimientos", "crear rutina", "enseñame un movimiento", "nueva rutina", "programar movimientos", "secuencia de movimientos", "bailar rutina", "grabar movimiento"],
            "speak": "¡Genial! Abriendo el gestor de rutinas personales.",
            "cmd":     "python3 src/YAREN2/yaren_movements/yaren_movements/yaren_rutinanueva.py &",
            "mode_id": "yaren_rutinanueva",
            "stop_cmd": "for pid in $(ps aux | grep -E 'yaren_rutinanueva' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done",
            "children": [],
        },
        {
            "label": ["mimic", "imitar", "copiar", "imítame", "imitame", "haz lo mismo que yo", "copia mis movimientos", "modo imitacion", "repite lo que hago", "espejo", "modo espejo", "imitacion"],
            "speak": "¡Activando el modo Mimic! Ahora te imito.",
            "cmd":     "ros2 launch yaren_arm_mimic yaren_mimic.launch.py &",
            "mode_id": "yaren_mimic",
            "stop_cmd": "for pid in $(ps aux | grep -E 'yaren_mimic' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done",
            "children": [],
        },
    ],
}

# Lista de canciones (mismo orden que en el C++ RadioApp)
SONGS = [
    "Luli Pampín - ARAM SAM SAM 2021.mp3",
    "CantaJuego - El Baile del Gorila.mp3",
    "Intro de Barney y sus amigos.mp3",
    "Chipi chipi chapa chapa dubi dubi daba daba Christell - Dubidubidu Subtitulada en español.mp3",
    "Martina Stoessel_ Libre Soy - Frozen_ Una Aventura Congelada.mp3",
    "Luli Pampín - SASA LA SERPIENTE (Official Video).mp3",
    "Luli Pampín - SI TÚ TIENES MUCHAS GANAS DE APLAUDIR - Official Video.mp3",
]

SONG_NAMES_SPEAK = [
    "Ara Ram Sam Sam",
    "Baile del Gorila",
    "Barney el Dinosaurio",
    "Chopi Chopi",
    "Libre Soy",
    "Sa Sa la Serpiente",
    "Si Tienes Ganas de Aplaudir",
]

# Respuestas cuando no entiende
FALLBACK_RESPONSES = [
    "Lo siento, no entendí bien. ¿Puedes repetirlo?",
    "Hmm, no estoy seguro de qué quieres. ¿Puedes decirlo de otra forma?",
    "No entendí. ¿Qué te gustaría hacer?",
    "¿Puedes repetir eso, por favor?",
]

# Confirmaciones generales
CONFIRM_RESPONSES = [
    "¡Perfecto!", "¡Claro que sí!", "¡De acuerdo!", "¡Enseguida!", "¡Entendido!",
]


# =============================================================================
#  Helpers de matching de texto
# =============================================================================

def _normalize(text: str) -> str:
    """Minúsculas y sin tildes para comparación robusta."""
    text = text.lower().strip()
    replacements = {
        "á": "a", "é": "e", "í": "i", "ó": "o", "ú": "u", "ü": "u", "ñ": "n",
    }
    for src, dst in replacements.items():
        text = text.replace(src, dst)
    return text


def _matches(text: str, keywords: list) -> bool:
    """True si alguna keyword aparece en el texto normalizado."""
    norm = _normalize(text)
    for kw in keywords:
        if _normalize(kw) in norm:
            return True
    return False


def _get_match_score(text: str, keywords: list) -> int:
    """Calcula el puntaje basado en la frase clave más larga que coincida."""
    norm = _normalize(text)
    best_score = 0
    for kw in keywords:
        kw_norm = _normalize(kw)
        if kw_norm in norm:
            # Mientras más larga y específica sea la frase clave, más puntos gana
            if len(kw_norm) > best_score:
                best_score = len(kw_norm)
    return best_score

def _find_best_item(text: str, items: list):
    """
    Busca en TODO el árbol y devuelve el ítem con el puntaje más alto.
    Esto soluciona los conflictos si una palabra se repite en varios menús.
    """
    best_item = None
    best_score = 0

    def traverse(current_items):
        nonlocal best_item, best_score
        for item in current_items:
            score = _get_match_score(text, item["label"])
            if score > best_score:
                best_score = score
                best_item = item
            
            # Busca recursivamente en los submenús
            if item.get("children"):
                traverse(item["children"])

    traverse(items)
    return best_item


def _is_yes(text: str) -> bool:
    return _matches(text, ["si", "sí", "claro", "dale", "ok", "okay", "bueno", "por favor", "quiero"])


def _is_no(text: str) -> bool:
    return _matches(text, ["no", "nada", "ninguno", "ninguna", "salir", "cancela", "cancelar", "adios", "adiós"])


def _is_stop(text: str) -> bool:
    return _matches(text, ["detener", "para", "parar", "stop", "detente", "suficiente"])


def _is_help(text: str) -> bool:
    norm = _normalize(text)
    
    # 1. Si la frase contiene la palabra "menu" pero NO especifica de qué
    # (musica, videos, rutinas, etc.), entonces quiere ir al menú principal.
    palabras_especificas = ["musica", "música", "video", "rutina", "filtro", "animal", "juego", "chat"]
    if "menu" in norm and not any(palabra in norm for palabra in palabras_especificas):
        return True
        
    # 2. Palabras clave tradicionales para ir al menú principal
    return _matches(text, ["ayuda", "que puedo hacer", "opciones", "que haces", "que puedes hacer", "menu principal", "volver al inicio"])

def _is_goodbye(text: str) -> bool:
    return _matches(text, ["adios", "adiós", "hasta luego", "chao", "bye", "salir", "terminar"])


# =============================================================================
#  Nodo principal
# =============================================================================

class YarenVoiceMenuNode(Node):

    def __init__(self):
        super().__init__("yaren_voice_menu")

        # ── Estado ────────────────────────────────────────────────────
        self.is_face_idle   = False   # pantalla libre → podemos escuchar
        self.is_active      = False   # estamos en conversación activa
        self.active_mode    = ""      # modo actual ejecutándose
        self.active_stop    = ""      # comando para detenerlo
        self.current_menu   = None    # submenú actual (lista de ítems)
        self.waiting_for    = None    # "main" | "sub" | "confirm_stop"
        self.tts_busy       = False   # evita escuchar mientras hablañ
        self.first_greeting_done = False  # <--- NUEVO: Recuerda si ya saludó

        # ── Rutas ─────────────────────────────────────────────────────
        workspace_dir = os.getcwd()
        self.audios_dir = os.path.join(
            workspace_dir, "src", "YAREN2", "yaren_radio", "audios"
        )

        pkg_share = get_package_share_directory("yaren_chat")
        self.tts_model_es = os.path.join(pkg_share, "models", "TTS", "es_MX-claude-high.onnx")
        self.tts_cfg_es   = os.path.join(pkg_share, "models", "TTS", "es_MX-claude-high.onnx.json")

        vosk_model_path = os.path.join(
            workspace_dir, "src", "YAREN2", "yaren_chat", "models", "STT",
            "vosk-model-small-es-0.42"
        )

        # ── TTS (Piper) ───────────────────────────────────────────────
        self.voice = None
        self.voice_lock = threading.Lock()
        self._load_tts()

        # ── STT (Vosk) ────────────────────────────────────────────────
        if not os.path.exists(vosk_model_path):
            self.get_logger().error(f"Modelo Vosk no encontrado: {vosk_model_path}")
            return

        self.vosk_model = Model(vosk_model_path)
        self.recognizer  = KaldiRecognizer(self.vosk_model, 16000)

        self.mic     = pyaudio.PyAudio()
        self.stream  = self.mic.open(
            format=pyaudio.paInt16, channels=1, rate=16000,
            input=True, frames_per_buffer=8000
        )
        self.stream.start_stream()

        # ── ROS2 pub / sub ────────────────────────────────────────────
        qos_tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.mode_pub    = self.create_publisher(String, "/yaren_mode",      10)
        self.audio_pub   = self.create_publisher(Bool,   "/audio_playing",   10)
        self.wake_pub    = self.create_publisher(Bool,   "/yaren/wake_event", 10)

        self.create_subscription(Bool,   "/yaren/face_idle",  self._cb_idle,  qos_tl)
        self.create_subscription(Bool,   "/yaren/wake_event", self._cb_wake,  10)

        # ── Timer de escucha ─────────────────────────────────────────
        self.create_timer(0.1, self._audio_loop)

        self.get_logger().info("✅ yaren_voice_menu listo. Esperando wake word...")

    # ═══════════════════════════════════════════════════════════════════
    #  Callbacks ROS2
    # ═══════════════════════════════════════════════════════════════════

    def _cb_idle(self, msg: Bool):
        self.is_face_idle = msg.data
        state = "LIBRE" if msg.data else "EN MENÚ"
        self.get_logger().info(f"Pantalla: {state}")

        if not msg.data:
            # Pantalla entró al menú → forzamos silencio completo
            if self.is_active:
                self._reset_conversation()
            # Limpiamos el estado del reconocedor para que no arrastre texto viejo
            self.recognizer.Reset()
            self.get_logger().info("STT reseteado por cambio de estado de pantalla.")

    def _cb_wake(self, msg: Bool):
        """
        El wake_word_node detecta 'despierta' y publica aquí.
        Si la pantalla está libre y nosotros no estamos activos, activamos la conversación.
        """
        if msg.data and self.is_face_idle and not self.is_active:
            self.get_logger().info("✨ Wake event recibido → iniciando menú de voz")
            threading.Thread(target=self._start_conversation, daemon=True).start()

    # ═══════════════════════════════════════════════════════════════════
    #  Loop de audio STT
    # ═══════════════════════════════════════════════════════════════════

    def _audio_loop(self):
        # ── Guardia de seguridad: si la pantalla no está libre, silencio total ──
        if not self.is_face_idle:
            # Vaciamos el buffer para no acumular audio durante el menú
            try:
                available = self.stream.get_read_available()
                if available > 0:
                    self.stream.read(available, exception_on_overflow=False)
            except Exception:
                pass
            return

        # ── Solo escucha si hay conversación activa y el TTS no habla ─────────
        if not self.is_active or self.tts_busy:
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
                text   = result.get("text", "").strip()
                if text:
                    self.get_logger().info(f"STT: '{text}'")
                    threading.Thread(
                        target=self._handle_text,
                        args=(text,),
                        daemon=True
                    ).start()
        except Exception:
            pass

    # ═══════════════════════════════════════════════════════════════════
    #  Lógica de conversación
    # ═══════════════════════════════════════════════════════════════════

    def _start_conversation(self):
        """Saludo inicial y entrada al menú principal."""
        self.is_active    = True
        self.waiting_for  = "main"
        self.current_menu = MENU_TREE["items"]
        
        # Evalúa si es la primera vez que te saluda desde que se encendió
        if not self.first_greeting_done:
            self._speak(MENU_TREE["speak_welcome"])
            self.first_greeting_done = True
        else:
            self._speak(MENU_TREE["speak_welcome_back"])

    def _handle_text(self, text: str):
        """Procesa el texto transcripto según el estado actual."""
        if not self.is_active:
            return

        # ── Despedida en cualquier momento ───────────────────────────
        if _is_goodbye(text):
            self._speak("¡Hasta luego! Fue un placer ayudarte.")
            self._reset_conversation()
            return

        # ── Pedir ayuda / opciones ────────────────────────────────────
        if _is_help(text):
            self._speak(MENU_TREE["speak_what_can_i_do"])
            self.waiting_for  = "main"
            self.current_menu = MENU_TREE["items"]
            return

        # ── Detener modo activo ───────────────────────────────────────
        if _is_stop(text):
            self._stop_active_mode()
            self._speak("He detenido lo que estaba haciendo. ¿En qué más puedo ayudarte?")
            self.waiting_for  = "main"
            self.current_menu = MENU_TREE["items"]
            return
        
        # ── Estados del menú ─────────────────────────────────────────
        if self.waiting_for == "main":
            self._handle_main_menu(text)

        elif self.waiting_for == "sub":
            self._handle_sub_menu(text)

    def _handle_main_menu(self, text: str):
        """Procesa el menú raíz de forma inteligente usando puntuación."""
        item = _find_best_item(text, MENU_TREE["items"])

        if item is None:
            self._speak(random.choice(FALLBACK_RESPONSES))
            return

        # Si el ítem ganador tiene hijos (es un menú principal), entra a él
        if item.get("children"):
            self.current_menu = item["children"]
            self.waiting_for  = "sub"
            self._speak(item["speak"])
            return

        # Si es una orden final (hoja), la ejecuta directamente
        if "song_index" in item:
            self._play_song(item)
        else:
            self._execute_item(item)

    def _handle_sub_menu(self, text: str):
        """Procesa estando dentro de un sub-menú permitiendo saltos globales."""
        if self.current_menu is None:
            self._reset_to_main()
            return

        # Si el usuario quiere salir o dice "no"
        if _is_no(text):
            self._speak("De acuerdo. ¿Qué más puedo hacer por ti?")
            self._reset_to_main()
            return

        # Yaren evalúa TODO el árbol de nuevo. 
        # Esto permite que si estás en música y dices "mejor pon un video",
        # el robot entienda el salto de contexto.
        item = _find_best_item(text, MENU_TREE["items"])

        if item is None:
            self._speak(random.choice(FALLBACK_RESPONSES))
            return

        if item.get("children"):
            self.current_menu = item["children"]
            self.waiting_for  = "sub"
            self._speak(item["speak"])
            return

        if "song_index" in item:
            self._play_song(item)
        else:
            self._execute_item(item)
    # ═══════════════════════════════════════════════════════════════════
    #  Ejecución de modos (igual que click en C++)
    # ═══════════════════════════════════════════════════════════════════

    def _execute_item(self, item: dict):
        """Habla la confirmación, limpia procesos y lanza el nuevo modo."""
        self._speak(random.choice(CONFIRM_RESPONSES) + " " + item["speak"])

        # Siempre matamos todo antes de iniciar un modo nuevo por voz
        self._stop_active_mode()

        mode_id = item.get("mode_id", "")
        raw_cmd = item.get("cmd", "")
        stop_cmd = item.get("stop_cmd", "")

        # ── MANEJO ESPECIAL PARA VIDEO ALEATORIO ──
        if raw_cmd == "RANDOM_VIDEO":
            video_options = [
                ("vid_pollito", "python3 src/YAREN2/yaren_radio/yaren_radio/pollitopio.py", "for pid in $(ps aux | grep -E 'pollitopio.py' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done"),
                ("vid_gallina", "python3 src/YAREN2/yaren_radio/yaren_radio/gallinaturuleca.py", "for pid in $(ps aux | grep -E 'gallinaturuleca.py' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done"),
                ("vid_vaca", "python3 src/YAREN2/yaren_radio/yaren_radio/vacalola.py", "for pid in $(ps aux | grep -E 'vacalola.py' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done"),
                ("vid_susanita", "python3 src/YAREN2/yaren_radio/yaren_radio/susanita.py", "for pid in $(ps aux | grep -E 'susanita.py' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done")
            ]
            mode_id, raw_cmd, stop_cmd = random.choice(video_options)

        if mode_id:
            msg = String()
            msg.data = mode_id
            self.mode_pub.publish(msg)
            self.active_mode = mode_id
            self.active_stop = stop_cmd

        # Ejecutar comando del sistema
        if raw_cmd:
            # Quitamos el '&' final para poder monitorear cuándo termina el proceso
            cmd = raw_cmd.strip()
            if cmd.endswith("&"):
                cmd = cmd[:-1].strip()

            self.get_logger().info(f"Ejecutando: {cmd}")

            def run_and_monitor():
                os.system(cmd)
                # Si el comando es un video y el usuario lo cierra con un clic, el script termina.
                # Debemos avisar al sistema que el robot vuelve a estar libre (idle).
                if mode_id.startswith("vid_") and self.active_mode == mode_id:
                    self.get_logger().info("Video finalizado. Publicando idle...")
                    msg_idle = String()
                    msg_idle.data = "idle"
                    self.mode_pub.publish(msg_idle)
                    self.active_mode = ""
                    self.active_stop = ""
                    time.sleep(0.5)  # CRUCIAL: Da tiempo a ROS 2 para enviar el mensaje

            # Lanzamos el proceso en un hilo para no bloquear el menú de voz
            threading.Thread(target=run_and_monitor, daemon=True).start()

        # El modo se ha iniciado; volvemos al estado de espera de comandos
        self.waiting_for  = "main"
        self.current_menu = MENU_TREE["items"]

    def _play_song(self, item: dict):
        """Manda la orden a C++ para abrir la interfaz de Radio y reproducir."""
        idx = item.get("song_index", -1)

        if idx == -1:
            idx = random.randrange(len(SONGS))
            speak_text = (
                random.choice(CONFIRM_RESPONSES)
                + f" Reproduciendo {SONG_NAMES_SPEAK[idx]}."
            )
        else:
            speak_text = random.choice(CONFIRM_RESPONSES) + " " + item["speak"]

        self._speak(speak_text)

        # Matamos todos los procesos anteriores
        self._stop_active_mode()

        # Enviar comando al C++ indicando el índice de la canción a reproducir
        msg = String()
        msg.data = f"play_radio_song:{idx}"
        self.mode_pub.publish(msg)

        # Establecemos el estado de "qué detener" por si dices "Detener"
        self.active_mode = "radio_musica"
        self.active_stop = "ros2 topic pub --once /yaren_mode std_msgs/msg/String \"{data: 'idle'}\""

        self.waiting_for  = "main"
        self.current_menu = MENU_TREE["items"]

    def _stop_active_mode(self):
        """Detiene cualquier modo activo matando todos los procesos posibles de Yaren."""
        self.get_logger().info("Limpiando entorno y deteniendo cualquier modo activo...")

        # Lista maestra de procesos de todos los modos
        kill_pattern = (
            "yaren_chat|yaren_emotions|pollitopio|gallinaturuleca|"
            "vacalola|susanita|yaren_dice|yaren_animales|AnimalFaceNode|"
            "animal_filter|face_landmark|csi_cam|yaren_accesorios|face_filter|"
            "yaren_fondo|fondo_virtual|yaren_rutinanueva|yaren_mimic|"
            "mpg123|yaren_dance_radio|yaren_rutina|yaren_fullmovement"
        )
        
        # Comando masivo para limpiar cualquier nodo que haya quedado abierto
        kill_cmd = f"for pid in $(ps aux | grep -E '{kill_pattern}' | grep -v grep | awk '{{print $2}}'); do kill -9 $pid; done"
        os.system(kill_cmd)

        # Avisar a C++ que el sistema está libre para que oculte el botón de detener
        msg = String()
        msg.data = "idle"
        self.mode_pub.publish(msg)
        self.active_mode = ""
        self.active_stop = ""

    # ═══════════════════════════════════════════════════════════════════
    #  Helpers de estado
    # ═══════════════════════════════════════════════════════════════════

    def _reset_to_main(self):
        self.waiting_for  = "main"
        self.current_menu = MENU_TREE["items"]

    def _reset_conversation(self):
        """Limpia todo el estado de la conversación."""
        self.is_active    = False
        self.waiting_for  = None
        self.current_menu = None
        self.get_logger().info("Conversación terminada.")

    # ═══════════════════════════════════════════════════════════════════
    #  TTS (Piper local)
    # ═══════════════════════════════════════════════════════════════════

    def _load_tts(self):
        """Carga el modelo Piper en español."""
        try:
            self.voice = PiperVoice.load(
                model_path=self.tts_model_es,
                config_path=self.tts_cfg_es,
                use_cuda=True,
            )
            self.get_logger().info("✅ Modelo TTS cargado correctamente.")
        except Exception as e:
            self.get_logger().error(f"Error cargando TTS: {e}")
            self.voice = None

    def _speak(self, text: str):
        """Sintetiza y reproduce texto con Piper. Bloquea hasta que termina."""
        if not text:
            return

        self.tts_busy = True
        self.get_logger().info(f"TTS → '{text}'")

        # Avisar al sistema que hay audio reproduciéndose
        audio_msg = Bool()
        audio_msg.data = True
        self.audio_pub.publish(audio_msg)

        try:
            syn_cfg = SynthesisConfig(
                length_scale=1.1,
                noise_scale=0.5,
                noise_w_scale=0.8,
            )
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as fp:
                tmp_path = fp.name

            with self.voice_lock:
                if self.voice is None:
                    return
                with wave.open(tmp_path, "wb") as wf:
                    wf.setnchannels(1)
                    wf.setsampwidth(2)
                    wf.setframerate(self.voice.config.sample_rate)
                    self.voice.synthesize_wav(text, wf, syn_config=syn_cfg)

            playsound(tmp_path)
            os.unlink(tmp_path)

        except Exception as e:
            self.get_logger().error(f"Error TTS: {e}")
        finally:
            audio_msg.data = False
            self.audio_pub.publish(audio_msg)
            self.tts_busy = False

    # ═══════════════════════════════════════════════════════════════════
    #  Destructor
    # ═══════════════════════════════════════════════════════════════════

    def destroy_node(self):
        try:
            self.stream.stop_stream()
            self.stream.close()
            self.mic.terminate()
        except Exception:
            pass
        super().destroy_node()


# =============================================================================
#  Entrada
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = YarenVoiceMenuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()