#!/usr/bin/env python3
"""
yaren_voice_menu.py
===================
Nodo ROS2 que replica el árbol de menús del face_screen (C++) pero por voz.
Soporta español e inglés según el idioma detectado por wake_word_node.

FIX: Al cambiar idioma se recarga el KaldiRecognizer con el modelo Vosk
     correspondiente (ES o EN), en lugar de quedarse siempre en español.

Topics ROS2:
  SUB  /yaren/face_idle      (std_msgs/Bool)   - pantalla libre o en menú
  SUB  /yaren/wake_event     (std_msgs/Bool)   - wake word ya detectado
  SUB  /yaren/is_english     (std_msgs/Bool)   - idioma activo
  PUB  /yaren_mode           (std_msgs/String) - activa modos
  PUB  /audio_playing        (std_msgs/Bool)   - pausa el TTS del chat
"""

import os
import json
import wave
import time
import tempfile
import threading
import random
import numpy as np
import pyaudio
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String

from vosk import Model, KaldiRecognizer
from piper import PiperVoice
from piper.config import SynthesisConfig
from playsound import playsound
from ament_index_python.packages import get_package_share_directory


# =============================================================================
#  Árboles de menú — Español
# =============================================================================
MENU_TREE_ES = {
    "speak_welcome": "¡Hola! Soy Yaren, tu amigo virtual. ¿En qué puedo ayudarte hoy?",
    "speak_welcome_back": "¡Hola de nuevo! ¿En qué te puedo ayudar ahora?",
    "speak_what_can_i_do": (
        "Claro, puedes hacer muchas cosas. "
        "Puedes conversar conmigo, detectar tus emociones, escuchar música, "
        "ver videos, jugar al Yaren Dice, ponerte filtros de animales, "
        "ponerte filtros de accesorios, ponerte un fondo virtual "
        "o grabar una rutina personal. ¿Qué te gustaría hacer?"
    ),
    "items": [
        {
            "label": ["chat", "conversar", "hablar", "conversa", "habla", "charlar",
                      "platicar", "chatear", "hablemos", "conversacion", "modo chat",
                      "inicia el chat", "dialogar", "hablame", "contame algo"],
            "speak": "¡Genial! Iniciando el modo chat. ¡Cuéntame todo!",
            "cmd": "", "mode_id": "yaren_chat", "children": [],
        },
        {
            "label": ["emociones", "emocion", "detectar emocion", "detecta", "sentimientos",
                      "como me siento", "como estoy", "analiza mi cara", "lee mis emociones",
                      "estado de animo", "detectar sentimientos", "emociones faciales", "que emocion tengo"],
            "speak": "Activando detección de emociones. ¡Muéstrame tu cara!",
            "cmd": "", "mode_id": "yaren_emotions", "children": [],
        },
        {
            "label": ["musica", "música", "canciones", "radio", "escuchar musica", "pon musica",
                      "reproduce", "quiero musica", "pon una cancion", "musiquita",
                      "escuchar canciones", "radio yaren", "poner musica", "escucha",
                      "musica por favor", "play", "bailar"],
            "speak": (
                "¡Buena elección! Tengo varias canciones disponibles. "
                "Puedes pedir: Ara Ram Sam Sam, Baile del Gorila, Barney, "
                "Chopi Chopi, Libre Soy, Sa Sa, o Si Tienes Ganas. "
                "¿Cuál quieres escuchar? También puedes decir cualquiera para una sorpresa."
            ),
            "cmd": "", "mode_id": "",
            "children": [
                {"label": ["aramsamsam", "ara ram sam sam", "luli", "ram sam", "aramsam", "cancion de luli", "luli pampin", "ara ram"],
                 "speak": "¡Reproduciendo Ara Ram Sam Sam!", "cmd": "", "mode_id": "radio_musica", "song_index": 0, "children": []},
                {"label": ["gorila", "baile del gorila", "cantajuego", "el gorila", "baile gorila", "canta juego", "gorila baila"],
                 "speak": "¡Reproduciendo el Baile del Gorila!", "cmd": "", "mode_id": "radio_musica", "song_index": 1, "children": []},
                {"label": ["barney", "dinosaurio", "barney el dinosaurio", "tema de barney", "intro barney", "dino", "barney y sus amigos"],
                 "speak": "¡Reproduciendo el intro de Barney!", "cmd": "", "mode_id": "radio_musica", "song_index": 2, "children": []},
                {"label": ["chopi chopi", "chipi chipi", "chapa chapa", "christell", "dubidubidu", "chipi chipi chapa chapa", "dubi dubi daba daba", "cancion de christell"],
                 "speak": "¡Reproduciendo Chopi Chopi!", "cmd": "", "mode_id": "radio_musica", "song_index": 3, "children": []},
                {"label": ["libre soy", "frozen", "martina", "libre soi", "frozen cancion", "martina stoessel", "cancion de frozen"],
                 "speak": "¡Reproduciendo Libre Soy!", "cmd": "", "mode_id": "radio_musica", "song_index": 4, "children": []},
                {"label": ["sasa", "sa sa", "serpiente", "sa sa la serpiente", "la serpiente", "cancion de la serpiente"],
                 "speak": "¡Reproduciendo Sa Sa La Serpiente!", "cmd": "", "mode_id": "radio_musica", "song_index": 5, "children": []},
                {"label": ["si tienes ganas", "tienes ganas", "aplaudir", "si tu tienes ganas", "ganas de aplaudir", "aplaude", "cancion de aplaudir"],
                 "speak": "¡Reproduciendo Si Tienes Ganas!", "cmd": "", "mode_id": "radio_musica", "song_index": 6, "children": []},
                {"label": ["cualquiera", "sorpresa", "pon cualquiera", "la que sea", "aleatoria", "aleatorio",
                            "al azar", "elige tu", "ponme una cualquiera", "sorpresa musical", "random",
                            "reproduce cualquier cosa", "pon cualquier cosa", "cualquier cancion", "cualquier musica", "lo que sea"],
                 "speak": "¡De acuerdo! Reproduciré una canción al azar.", "cmd": "", "mode_id": "radio_musica", "song_index": -1, "children": []},
            ],
        },
        {
            "label": ["videos", "ver videos", "video", "canciones animadas", "ver un video",
                      "reproduce un video", "videos para niños", "animacion", "videos infantiles",
                      "pon un video", "mirar videos", "ver dibujos", "caricaturas"],
            "speak": (
                "¡Claro! Tengo varios videos disponibles. "
                "Puedes pedir: Pollito Pío, Gallina Turuleca, La Vaca Lola o Susanita. "
                "¿Cuál quieres ver? También puedes decir cualquiera para una sorpresa."
            ),
            "cmd": "", "mode_id": "",
            "children": [
                {"label": ["pollito", "pollito pio", "pío", "granja", "el pollito pio", "pio pio", "pollito pío", "granja de pollito"],
                 "speak": "¡Reproduciendo Pollito Pío!", "cmd": "python3 src/YAREN2/yaren_radio/yaren_radio/pollitopio.py &", "mode_id": "vid_pollito", "children": []},
                {"label": ["gallina turuleca", "turuleca", "gallina", "la gallina turuleca", "gallina loca", "turuleca cancion"],
                 "speak": "¡Reproduciendo la Gallina Turuleca!", "cmd": "python3 src/YAREN2/yaren_radio/yaren_radio/gallinaturuleca.py &", "mode_id": "vid_gallina", "children": []},
                {"label": ["vaca lola", "vaca", "lola", "la vaca lola", "vaca lola vaca lola", "cancion de la vaca"],
                 "speak": "¡Reproduciendo La Vaca Lola!", "cmd": "python3 src/YAREN2/yaren_radio/yaren_radio/vacalola.py &", "mode_id": "vid_vaca", "children": []},
                {"label": ["susanita", "zenon", "granja de zenon", "susanita tiene un raton", "zenon granja", "susanita cancion"],
                 "speak": "¡Reproduciendo Susanita!", "cmd": "python3 src/YAREN2/yaren_radio/yaren_radio/susanita.py &", "mode_id": "vid_susanita", "children": []},
                {"label": ["un video cualquiera", "pon un video cualquiera", "cualquier video", "video aleatorio",
                            "un video al azar", "video sorpresa", "reproduce un video cualquiera", "reproduce cualquier video"],
                 "speak": "¡De acuerdo! Preparando un video sorpresa para ti.", "cmd": "RANDOM_VIDEO", "mode_id": "", "children": []},
            ],
        },
        {
            "label": ["yaren dice", "yo digo", "simon dice", "jugar", "juguemos", "inicia yaren dice",
                      "juego de imitacion", "juego de repetir", "jugamos", "modo juego", "simon dice juego"],
            "speak": "¡Vamos a jugar! Iniciando Yaren Dice.",
            "cmd": "", "mode_id": "yaren_dice", "children": [],
        },
        {
            "label": ["filtro animales", "animales", "filtro animal", "modo animales", "ponme de animal",
                      "cara de animal", "filtro de animalitos", "transforma en animal", "mascota", "animalitos"],
            "speak": "¡Activando filtro de animales! ¡Mira qué gracioso quedas!",
            "cmd": "", "mode_id": "yaren_animales", "children": [],
        },
        {
            "label": ["filtro accesorios", "accesorios", "filtro accesorio", "sombrero", "gafas",
                      "modo accesorios", "ponme gafas", "lentes", "anteojos", "sombreros", "gorro",
                      "accesorios para la cara", "ponme un sombrero", "filtros divertidos"],
            "speak": "¡Activando filtro de accesorios!",
            "cmd": "", "mode_id": "yaren_accesorios", "children": [],
        },
        {
            "label": ["fondo virtual", "fondo", "fondo magico", "background", "cambia el fondo",
                      "pon un fondo", "fondo nuevo", "cambiar escenario", "fondo de pantalla",
                      "modo fondo", "entorno virtual"],
            "speak": "¡Activando fondo virtual!",
            "cmd": "", "mode_id": "yaren_fondo", "children": [],
        },
        {
            "label": ["rutina", "rutinas", "grabar rutina", "rutina personal", "movimientos",
                      "crear rutina", "enseñame un movimiento", "nueva rutina",
                      "programar movimientos", "secuencia de movimientos", "bailar rutina", "grabar movimiento"],
            "speak": "¡Genial! Abriendo el gestor de rutinas personales.",
            "cmd": "python3 src/YAREN2/yaren_movements/yaren_movements/yaren_rutinanueva.py &",
            "mode_id": "yaren_rutinanueva", "children": [],
        },
        {
            "label": ["mimic", "imitar", "copiar", "imítame", "imitame", "haz lo mismo que yo",
                      "copia mis movimientos", "modo imitacion", "repite lo que hago",
                      "espejo", "modo espejo", "imitacion"],
            "speak": "¡Activando el modo Mimic! Ahora te imito.",
            "cmd": "ros2 launch yaren_arm_mimic yaren_mimic.launch.py &",
            "mode_id": "yaren_mimic", "children": [],
        },
    ],
}

# =============================================================================
#  Árboles de menú — English
# =============================================================================
MENU_TREE_EN = {
    "speak_welcome": "Hi! I'm Yaren, your virtual friend. How can I help you today?",
    "speak_welcome_back": "Welcome back! How can I help you now?",
    "speak_what_can_i_do": (
        "Sure! There are many things you can do. "
        "You can chat with me, detect your emotions, listen to music, "
        "watch videos, play Yaren Says, put on animal filters, "
        "accessories filters, a virtual background, "
        "or record a personal routine. What would you like to do?"
    ),
    "items": [
        {
            "label": ["chat", "talk", "speak", "conversation", "let's talk", "start chat",
                      "talk to me", "tell me something", "have a conversation"],
            "speak": "Great! Starting chat mode. Tell me everything!",
            "cmd": "", "mode_id": "yaren_chat", "children": [],
        },
        {
            "label": ["emotions", "emotion", "detect emotion", "feelings", "how do i look",
                      "read my face", "facial emotions", "what emotion", "detect feelings", "mood"],
            "speak": "Activating emotion detection. Show me your face!",
            "cmd": "", "mode_id": "yaren_emotions", "children": [],
        },
        {
            "label": ["music", "songs", "radio", "play music", "play a song", "listen to music",
                      "dance", "play something", "i want music"],
            "speak": (
                "Great choice! I have several songs available. "
                "You can ask for: Ara Ram Sam Sam, Baile del Gorila, Barney, "
                "Chopi Chopi, Libre Soy, Sa Sa, or Si Tienes Ganas. "
                "Which one do you want? You can also say any for a surprise."
            ),
            "cmd": "", "mode_id": "",
            "children": [
                {"label": ["ara ram sam sam", "luli", "ram sam"],
                 "speak": "Playing Ara Ram Sam Sam!", "cmd": "", "mode_id": "radio_musica", "song_index": 0, "children": []},
                {"label": ["gorilla", "gorilla dance", "cantajuego"],
                 "speak": "Playing Baile del Gorila!", "cmd": "", "mode_id": "radio_musica", "song_index": 1, "children": []},
                {"label": ["barney", "dinosaur", "barney the dinosaur", "barney song"],
                 "speak": "Playing Barney!", "cmd": "", "mode_id": "radio_musica", "song_index": 2, "children": []},
                {"label": ["chopi chopi", "chipi chipi", "christell", "dubidubidu"],
                 "speak": "Playing Chopi Chopi!", "cmd": "", "mode_id": "radio_musica", "song_index": 3, "children": []},
                {"label": ["let it go", "frozen", "martina", "libre soy", "free"],
                 "speak": "Playing Libre Soy!", "cmd": "", "mode_id": "radio_musica", "song_index": 4, "children": []},
                {"label": ["sa sa", "serpent", "snake song", "sasa"],
                 "speak": "Playing Sa Sa!", "cmd": "", "mode_id": "radio_musica", "song_index": 5, "children": []},
                {"label": ["if you're happy", "clap your hands", "si tienes ganas"],
                 "speak": "Playing Si Tienes Ganas!", "cmd": "", "mode_id": "radio_musica", "song_index": 6, "children": []},
                {"label": ["any", "surprise", "random", "whatever", "any song", "pick one", "choose one"],
                 "speak": "Sure! Playing a random song.", "cmd": "", "mode_id": "radio_musica", "song_index": -1, "children": []},
            ],
        },
        {
            "label": ["video", "videos", "watch a video", "play a video", "animated songs",
                      "kids videos", "cartoons", "show me a video"],
            "speak": (
                "Sure! I have several videos available. "
                "You can ask for: Pollito Pio, Gallina Turuleca, La Vaca Lola or Susanita. "
                "Which one do you want? You can also say any for a surprise."
            ),
            "cmd": "", "mode_id": "",
            "children": [
                {"label": ["pollito pio", "little chick", "chick", "farm song"],
                 "speak": "Playing Pollito Pío!", "cmd": "python3 src/YAREN2/yaren_radio/yaren_radio/pollitopio.py &", "mode_id": "vid_pollito", "children": []},
                {"label": ["gallina turuleca", "turuleca", "hen", "crazy hen"],
                 "speak": "Playing Gallina Turuleca!", "cmd": "python3 src/YAREN2/yaren_radio/yaren_radio/gallinaturuleca.py &", "mode_id": "vid_gallina", "children": []},
                {"label": ["vaca lola", "lola the cow", "lola cow", "cow song"],
                 "speak": "Playing La Vaca Lola!", "cmd": "python3 src/YAREN2/yaren_radio/yaren_radio/vacalola.py &", "mode_id": "vid_vaca", "children": []},
                {"label": ["susanita", "zenon", "zenon farm", "susanita song"],
                 "speak": "Playing Susanita!", "cmd": "python3 src/YAREN2/yaren_radio/yaren_radio/susanita.py &", "mode_id": "vid_susanita", "children": []},
                {"label": ["any video", "random video", "surprise video", "any", "whatever video"],
                 "speak": "Sure! Preparing a surprise video for you.", "cmd": "RANDOM_VIDEO", "mode_id": "", "children": []},
            ],
        },
        {
            "label": ["yaren says", "simon says", "play", "let's play", "game", "play a game",
                      "imitation game", "repeat game"],
            "speak": "Let's play! Starting Yaren Says.",
            "cmd": "", "mode_id": "yaren_dice", "children": [],
        },
        {
            "label": ["animal filter", "animals", "animal face", "put an animal", "animal mode",
                      "transform into animal", "animal mask"],
            "speak": "Activating animal filter! Look how funny you look!",
            "cmd": "", "mode_id": "yaren_animales", "children": [],
        },
        {
            "label": ["accessories filter", "accessories", "hat", "glasses", "sunglasses",
                      "accessories mode", "put glasses", "put a hat", "fun filters",
                      "working girl", "accessory"],
            "speak": "Activating accessories filter!",
            "cmd": "", "mode_id": "yaren_accesorios", "children": [],
        },
        {
            "label": ["virtual background", "background", "magic background", "change background",
                      "new background", "virtual environment", "background mode"],
            "speak": "Activating virtual background!",
            "cmd": "", "mode_id": "yaren_fondo", "children": [],
        },
        {
            "label": ["routine", "routines", "record routine", "personal routine", "movements",
                      "create routine", "teach me a move", "new routine", "program movements",
                      "sequence of movements", "dance routine", "record movement"],
            "speak": "Great! Opening the personal routines manager.",
            "cmd": "python3 src/YAREN2/yaren_movements/yaren_movements/yaren_rutinanueva.py &",
            "mode_id": "yaren_rutinanueva", "children": [],
        },
        {
            "label": ["mimic", "imitate", "copy", "imitate me", "do what i do", "copy my movements",
                      "imitation mode", "repeat what i do", "mirror", "mirror mode"],
            "speak": "Activating Mimic mode! I'll imitate you now.",
            "cmd": "ros2 launch yaren_arm_mimic yaren_mimic.launch.py &",
            "mode_id": "yaren_mimic", "children": [],
        },
    ],
}

# =============================================================================
#  Canciones
# =============================================================================
SONGS = [
    "Luli Pampín - ARAM SAM SAM 2021.mp3",
    "CantaJuego - El Baile del Gorila.mp3",
    "Intro de Barney y sus amigos.mp3",
    "Chipi chipi chapa chapa dubi dubi daba daba Christell - Dubidubidu Subtitulada en español.mp3",
    "Martina Stoessel_ Libre Soy - Frozen_ Una Aventura Congelada.mp3",
    "Luli Pampín - SASA LA SERPIENTE (Official Video).mp3",
    "Luli Pampín - SI TÚ TIENES MUCHAS GANAS DE APLAUDIR - Official Video.mp3",
]

SONG_NAMES = {
    "es": ["Ara Ram Sam Sam", "Baile del Gorila", "Barney el Dinosaurio", "Chopi Chopi",
           "Libre Soy", "Sa Sa la Serpiente", "Si Tienes Ganas de Aplaudir"],
    "en": ["Ara Ram Sam Sam", "Baile del Gorila", "Barney the Dinosaur", "Chopi Chopi",
           "Libre Soy", "Sa Sa", "Si Tienes Ganas"],
}

FALLBACK_RESPONSES = {
    "es": ["Lo siento, no entendí bien. ¿Puedes repetirlo?",
           "Hmm, no estoy seguro de qué quieres. ¿Puedes decirlo de otra forma?",
           "No entendí. ¿Qué te gustaría hacer?",
           "¿Puedes repetir eso, por favor?"],
    "en": ["Sorry, I didn't understand. Can you repeat that?",
           "Hmm, I'm not sure what you want. Can you say it differently?",
           "I didn't catch that. What would you like to do?",
           "Could you please repeat that?"],
}

CONFIRM_RESPONSES = {
    "es": ["¡Perfecto!", "¡Claro que sí!", "¡De acuerdo!", "¡Enseguida!", "¡Entendido!"],
    "en": ["Perfect!", "Sure!", "Got it!", "Right away!", "Understood!"],
}


# =============================================================================
#  Helpers de matching
# =============================================================================
def _normalize(text: str) -> str:
    text = text.lower().strip()
    for src, dst in {"á": "a", "é": "e", "í": "i", "ó": "o", "ú": "u", "ü": "u", "ñ": "n"}.items():
        text = text.replace(src, dst)
    return text

def _get_match_score(text: str, keywords: list) -> int:
    norm = _normalize(text)
    best = 0
    for kw in keywords:
        kw_n = _normalize(kw)
        if kw_n in norm and len(kw_n) > best:
            best = len(kw_n)
    return best

def _find_best_item(text: str, items: list):
    best_item, best_score = None, 0
    def traverse(current):
        nonlocal best_item, best_score
        for item in current:
            score = _get_match_score(text, item["label"])
            if score > best_score:
                best_score = score
                best_item  = item
            if item.get("children"):
                traverse(item["children"])
    traverse(items)
    return best_item

def _matches(text: str, keywords: list) -> bool:
    norm = _normalize(text)
    return any(_normalize(kw) in norm for kw in keywords)

def _is_no(text: str) -> bool:
    return _matches(text, ["no", "nada", "ninguno", "ninguna", "salir", "cancela",
                            "cancelar", "adios", "nope", "cancel", "exit", "nevermind"])

def _is_stop(text: str) -> bool:
    return _matches(text, ["detener", "para", "parar", "stop", "detente", "suficiente", "enough"])

def _is_help(text: str) -> bool:
    norm = _normalize(text)
    specific = ["musica", "music", "video", "rutina", "routine", "filtro", "filter",
                "animal", "juego", "game", "chat"]
    if ("menu" in norm or "help" in norm) and not any(p in norm for p in specific):
        return True
    return _matches(text, ["ayuda", "que puedo hacer", "opciones", "que haces",
                            "que puedes hacer", "menu principal", "volver al inicio",
                            "what can you do", "what can i do", "options", "main menu"])

def _is_goodbye(text: str) -> bool:
    return _matches(text, ["adios", "hasta luego", "chao", "bye", "goodbye",
                            "see you", "salir", "terminar", "exit", "quit"])


# =============================================================================
#  Nodo principal
# =============================================================================
class YarenVoiceMenuNode(Node):

    def __init__(self):
        super().__init__("yaren_voice_menu")

        self.is_face_idle        = False
        self.is_active           = False
        self.is_english          = False
        self.active_mode         = ""
        self.current_menu        = None
        self.waiting_for         = None
        self.tts_busy            = False
        self.first_greeting_done = False
        self._start_lock         = threading.Lock()
        self._conversation_start_time = 0.0

        workspace_dir = os.getcwd()
        pkg_share     = get_package_share_directory("yaren_chat")

        self.tts_model_es = os.path.join(pkg_share, "models", "TTS", "es_MX-claude-high.onnx")
        self.tts_cfg_es   = os.path.join(pkg_share, "models", "TTS", "es_MX-claude-high.onnx.json")
        self.tts_model_en = os.path.join(pkg_share, "models", "TTS", "en_US-lessac-medium.onnx")
        self.tts_cfg_en   = os.path.join(pkg_share, "models", "TTS", "en_US-lessac-medium.onnx.json")

        self.voice_es   = None
        self.voice_en   = None
        self.voice_lock = threading.Lock()
        self._load_tts()
        qos_tl_mic = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mic_owner_pub = self.create_publisher(String, "/yaren/mic_owner", qos_tl_mic)
        self.mic_owner = "none"
        self.create_subscription(String, "/yaren/mic_owner", self._cb_mic_owner, qos_tl_mic)
        self.lipsync_pub = self.create_publisher(String, "/yaren/tts_text", 10)
        # ── STT: paths de ambos modelos Vosk ─────────────────────────────────
        vosk_path_es = os.path.join(
            workspace_dir, "src", "YAREN2", "yaren_chat", "models", "STT",
            "vosk-model-small-es-0.42"
        )
        vosk_path_en = os.path.join(
            workspace_dir, "src", "YAREN2", "yaren_chat", "models", "STT",
            "vosk-model-en-us-0.22-lgraph"   # TODO: ajusta al nombre exacto de tu carpeta
        )

        if not os.path.exists(vosk_path_es):
            self.get_logger().error(f"Modelo Vosk ES no encontrado: {vosk_path_es}")
            return

        # ── Cargar modelo ES (obligatorio) ───────────────────────────────────
        self.vosk_model_es = Model(vosk_path_es)
        self.get_logger().info("✅ Modelo Vosk ES cargado.")

        # ── Cargar modelo EN (opcional, fallback a ES si no existe) ──────────
        if os.path.exists(vosk_path_en):
            self.vosk_model_en = Model(vosk_path_en)
            self.get_logger().info("✅ Modelo Vosk EN cargado.")
        else:
            self.vosk_model_en = None
            self.get_logger().warn(
                f"⚠️  Modelo Vosk EN no encontrado en: {vosk_path_en}\n"
                "   Descárgalo con:\n"
                "   wget https://alphacephei.com/vosk/models/vosk-model-en-us-0.22-lgraph.zip\n"
                "   y descomprímelo en src/YAREN2/yaren_chat/models/STT/\n"
                "   Por ahora se usará el modelo ES como fallback."
            )

        # Recognizer activo (inicia en ES)
        self.vosk_model = self.vosk_model_es
        self.recognizer = KaldiRecognizer(self.vosk_model, 16000)

        # ── Micrófono ─────────────────────────────────────────────────────────
        self.mic    = pyaudio.PyAudio()
        self.stream = self.mic.open(
            format=pyaudio.paInt16, channels=1, rate=16000,
            input=True, frames_per_buffer=8000
        )
        self.stream.start_stream()

        qos_tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.mode_pub  = self.create_publisher(String, "/yaren_mode",    10)
        self.audio_pub = self.create_publisher(Bool,   "/audio_playing", 10)

        self.create_subscription(Bool, "/yaren/face_idle",  self._cb_idle,     qos_tl)
        self.create_subscription(Bool, "/yaren/wake_event", self._cb_wake,     10)
        self.create_subscription(Bool, "/yaren/is_english", self._cb_language, qos_tl)

        self.create_timer(0.1, self._audio_loop)

        self.get_logger().info("✅ yaren_voice_menu listo. Esperando wake word...")

    # ── Propiedades helpers ───────────────────────────────────────────────────
    @property
    def _lang(self) -> str:
        return "en" if self.is_english else "es"

    @property
    def _menu_tree(self) -> dict:
        return MENU_TREE_EN if self.is_english else MENU_TREE_ES

    def _fallback(self) -> str:
        return random.choice(FALLBACK_RESPONSES[self._lang])

    def _confirm(self) -> str:
        return random.choice(CONFIRM_RESPONSES[self._lang])

    def _song_name(self, idx: int) -> str:
        return SONG_NAMES[self._lang][idx]
    def _cb_mic_owner(self, msg: String):
        self.mic_owner = msg.data

    # ── Callbacks ROS2 ────────────────────────────────────────────────────────
    def _cb_language(self, msg: Bool):
        self.is_english = msg.data
        self.get_logger().info(f"Idioma: {'EN' if self.is_english else 'ES'}")

        # ── FIX: cambiar modelo STT activo y recrear el recognizer ───────────
        if self.is_english and self.vosk_model_en is not None:
            self.vosk_model = self.vosk_model_en
            self.get_logger().info("STT → modelo EN activo.")
        else:
            self.vosk_model = self.vosk_model_es
            if self.is_english:
                self.get_logger().warn("STT → modelo EN no disponible, usando ES como fallback.")
            else:
                self.get_logger().info("STT → modelo ES activo.")

        self.recognizer = KaldiRecognizer(self.vosk_model, 16000)

    def _cb_idle(self, msg: Bool):
        self.is_face_idle = msg.data
        self.get_logger().info(f"Pantalla: {'LIBRE' if msg.data else 'EN MENÚ'}")
        
        if not msg.data:
            # Solo terminar conversación si el usuario abrió el menú manualmente
            # (detectado porque is_active lleva más de 1 segundo activo)
            if self.is_active:
                elapsed = time.time() - self._conversation_start_time
                if elapsed > 2.0:
                    # El usuario tocó el menú — sí terminar
                    self._reset_conversation()
                    self.recognizer.Reset()
                else:
                    self.get_logger().info("face_idle=False transitorio durante saludo, ignorando.")
            else:
                self._reset_conversation()
                self.recognizer.Reset()

    def _cb_wake(self, msg: Bool):
        self.get_logger().info(
            f"Wake cb: data={msg.data} idle={self.is_face_idle} active={self.is_active}"
        )
        if msg.data and self.is_face_idle and not self.is_active:
            threading.Thread(target=self._start_conversation, daemon=True).start()

    def _audio_loop(self):
        if not self.is_face_idle or not self.is_active or self.tts_busy or self.mic_owner != "voice_menu":
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
                        target=self._handle_text, args=(text,), daemon=True
                    ).start()
        except Exception:
            pass

    # ── Conversación ─────────────────────────────────────────────────────────
    def _start_conversation(self):
        with self._start_lock:
            if self.is_active:
                return
            owner_msg = String()
            owner_msg.data = "voice_menu"
            self.mic_owner_pub.publish(owner_msg)
            self.is_active    = True
            self.waiting_for  = "main"
            self.current_menu = self._menu_tree["items"]
            self._conversation_start_time = time.time()  # ← agregar esto

        if not self.first_greeting_done:
            self._speak(self._menu_tree["speak_welcome"])
            self.first_greeting_done = True
        else:
            self._speak(self._menu_tree["speak_welcome_back"])

    def _handle_text(self, text: str):
        if not self.is_active:
            return

        if _is_goodbye(text):
            self._speak("¡Hasta luego! Fue un placer ayudarte." if not self.is_english
                        else "Goodbye! It was a pleasure helping you.")
            self._reset_conversation()
            return

        if _is_help(text):
            self._speak(self._menu_tree["speak_what_can_i_do"])
            self.waiting_for  = "main"
            self.current_menu = self._menu_tree["items"]
            return

        if _is_stop(text):
            self._stop_active_mode()
            self._speak("He detenido lo que estaba haciendo. ¿En qué más puedo ayudarte?"
                        if not self.is_english
                        else "I've stopped what I was doing. How else can I help you?")
            self.waiting_for  = "main"
            self.current_menu = self._menu_tree["items"]
            return

        if self.waiting_for == "main":
            self._handle_main_menu(text)
        elif self.waiting_for == "sub":
            self._handle_sub_menu(text)

    def _handle_main_menu(self, text: str):
        item = _find_best_item(text, self._menu_tree["items"])
        if item is None:
            self._speak(self._fallback())
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

    def _handle_sub_menu(self, text: str):
        if self.current_menu is None:
            self._reset_to_main()
            return
        if _is_no(text):
            self._speak("De acuerdo. ¿Qué más puedo hacer por ti?" if not self.is_english
                        else "Alright. What else can I do for you?")
            self._reset_to_main()
            return
        item = _find_best_item(text, self._menu_tree["items"])
        if item is None:
            self._speak(self._fallback())
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

    def _execute_item(self, item: dict):
        self._speak(self._confirm() + " " + item["speak"])
        self._stop_active_mode()

        mode_id = item.get("mode_id", "")
        raw_cmd = item.get("cmd", "")

        if raw_cmd == "RANDOM_VIDEO":
            video_options = [
                ("vid_pollito", "python3 src/YAREN2/yaren_radio/yaren_radio/pollitopio.py"),
                ("vid_gallina", "python3 src/YAREN2/yaren_radio/yaren_radio/gallinaturuleca.py"),
                ("vid_vaca",    "python3 src/YAREN2/yaren_radio/yaren_radio/vacalola.py"),
                ("vid_susanita","python3 src/YAREN2/yaren_radio/yaren_radio/susanita.py"),
            ]
            mode_id, raw_cmd = random.choice(video_options)

        if mode_id:
            msg      = String()
            msg.data = mode_id
            self.mode_pub.publish(msg)
            self.active_mode = mode_id

        if raw_cmd:
            cmd = raw_cmd.strip().rstrip("& ").strip()
            self.get_logger().info(f"Ejecutando: {cmd}")

            def run_and_monitor():
                os.system(cmd)
                if mode_id.startswith("vid_") and self.active_mode == mode_id:
                    msg_idle      = String()
                    msg_idle.data = "idle"
                    self.mode_pub.publish(msg_idle)
                    self.active_mode = ""

            threading.Thread(target=run_and_monitor, daemon=True).start()

        self.waiting_for  = "main"
        self.current_menu = self._menu_tree["items"]

    def _play_song(self, item: dict):
        idx = item.get("song_index", -1)
        if idx == -1:
            idx = random.randrange(len(SONGS))
            speak_text = (self._confirm()
                          + (" Reproduciendo " if not self.is_english else " Playing ")
                          + self._song_name(idx) + ".")
        else:
            speak_text = self._confirm() + " " + item["speak"]

        self._speak(speak_text)
        self._stop_active_mode()

        msg      = String()
        msg.data = f"play_radio_song:{idx}"
        self.mode_pub.publish(msg)

        self.active_mode  = "radio_musica"
        self.waiting_for  = "main"
        self.current_menu = self._menu_tree["items"]

    def _stop_active_mode(self):
        msg      = String()
        msg.data = "idle"
        self.mode_pub.publish(msg)
        self.active_mode = ""

    def _reset_to_main(self):
        self.waiting_for  = "main"
        self.current_menu = self._menu_tree["items"]

    def _reset_conversation(self):
        self.is_active           = False
        self.first_greeting_done = False
        self.waiting_for         = None
        self.current_menu        = None
        self.get_logger().info("Conversación terminada.")
        owner_msg = String()
        owner_msg.data = "none"
        self.mic_owner_pub.publish(owner_msg)

    # ── TTS ──────────────────────────────────────────────────────────────────
    def _load_tts(self):
        try:
            self.voice_es = PiperVoice.load(
                model_path=self.tts_model_es,
                config_path=self.tts_cfg_es,
                use_cuda=True,
            )
            self.get_logger().info("✅ TTS español cargado.")
        except Exception as e:
            self.get_logger().error(f"Error cargando TTS ES: {e}")

        try:
            self.voice_en = PiperVoice.load(
                model_path=self.tts_model_en,
                config_path=self.tts_cfg_en,
                use_cuda=True,
            )
            self.get_logger().info("✅ TTS inglés cargado.")
        except Exception as e:
            self.get_logger().warn(f"TTS EN no disponible, usando ES como fallback: {e}")
            self.voice_en = self.voice_es
    
    def _compute_lipsync(self, wav_path: str, sample_rate: int) -> str:
        try:
            with wave.open(wav_path, 'rb') as wf:
                n_frames   = wf.getnframes()
                n_channels = wf.getnchannels()
                sampwidth  = wf.getsampwidth()
                raw        = wf.readframes(n_frames)

            if sampwidth == 2:
                samples = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
            elif sampwidth == 4:
                samples = np.frombuffer(raw, dtype=np.int32).astype(np.float32) / 2147483648.0
            else:
                samples = np.frombuffer(raw, dtype=np.uint8).astype(np.float32) / 128.0 - 1.0

            if n_channels > 1:
                samples = samples.reshape(-1, n_channels).mean(axis=1)

            FRAME_MS   = 80
            frame_size = int(sample_rate * FRAME_MS / 1000)
            if frame_size == 0:
                return ""

            indices = []
            for i in range(0, len(samples), frame_size):
                chunk = samples[i:i + frame_size]
                if len(chunk) == 0:
                    break
                rms = float(np.sqrt(np.mean(chunk ** 2)))
                if   rms < 0.01: idx = 0
                elif rms < 0.03: idx = 1
                elif rms < 0.06: idx = 2
                elif rms < 0.10: idx = 3
                elif rms < 0.15: idx = 4
                elif rms < 0.20: idx = 5
                elif rms < 0.27: idx = 6
                elif rms < 0.35: idx = 7
                else:             idx = 8
                indices.append(str(idx))

            return f"{FRAME_MS}:{','.join(indices)}" if indices else ""

        except Exception as e:
            self.get_logger().error(f"[LipSync] Error: {e}")
            return ""
    
    def _speak(self, text: str):
        if not text:
            return

        self.tts_busy = True
        self.get_logger().info(f"TTS → '{text}'")

        audio_msg      = Bool()
        audio_msg.data = True
        self.audio_pub.publish(audio_msg)

        try:
            voice = self.voice_en if self.is_english else self.voice_es
            if voice is None:
                return

            syn_cfg = SynthesisConfig(length_scale=1.1, noise_scale=0.5, noise_w_scale=0.8)

            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as fp:
                tmp_path = fp.name

            # 1. Sintetizar WAV
            with self.voice_lock:
                with wave.open(tmp_path, "wb") as wf:
                    wf.setnchannels(1)
                    wf.setsampwidth(2)
                    wf.setframerate(voice.config.sample_rate)
                    voice.synthesize_wav(text, wf, syn_config=syn_cfg)

            # 2. Calcular lip sync y publicar ANTES de reproducir
            lipsync_str = self._compute_lipsync(tmp_path, voice.config.sample_rate)
            if lipsync_str:
                ls_msg      = String()
                ls_msg.data = lipsync_str
                self.lipsync_pub.publish(ls_msg)
                self.get_logger().debug(f"[LipSync] Publicado: {lipsync_str[:60]}...")

            # 3. Reproducir
            from playsound import playsound
            playsound(tmp_path)
            os.unlink(tmp_path)

        except Exception as e:
            self.get_logger().error(f"Error TTS: {e}")
        finally:
            audio_msg.data = False
            self.audio_pub.publish(audio_msg)
            self.tts_busy = False
    # ── Cleanup ───────────────────────────────────────────────────────────────
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