#!/usr/bin/env python3
"""
yaren_brain_node.py — Cerebro autonomo de YAREN
================================================
Nodo completamente NUEVO. No modifica ningun nodo existente.
Solo ESCUCHA topics existentes y PUBLICA en topics que ya existen.

Topics que ESCUCHA (ya existen):
  /yaren/face_idle      (Bool)   - pantalla libre o en menu
  /audio_playing        (Bool)   - TTS hablando
  /yaren/mic_owner      (String) - quien tiene el microfono
  /yaren/is_english     (Bool)   - idioma actual
  /csi_camera/image_raw (Image)  - frames de camara (cuando este activa)
  /yaren/system_ready   (Bool)   - sistema listo (face_screen termino configuracion)

Topics que PUBLICA (ya existen o son nuevos inocuos):
  /yaren_mode           (String) - para activar modos
  /yaren/mic_owner      (String) - para tomar el microfono
  /audio_playing        (Bool)   - para bloquear STT mientras habla
  /yaren/brain_highlight (String) - resaltar boton en pantalla (opcional)
  /yaren/brain_state    (String) - estado del brain (active/inactive/idle_video)
  /yaren/wakeword_enabled (Bool) - habilitar/deshabilitar wakeword

Lo que hace (NUEVA VERSION CON CHAT OFFLINE):
  1. Espera a que el sistema este completamente configurado
  2. Activa la camara cuando la necesita
  3. Cuando face_idle=True y ve una persona - saluda proactivamente
  4. Despues del saludo, inicia una conversacion offline (sin LLM)
     usando una base de datos de frases, chistes, consejos, trivia, etc.
  5. Mantiene la conversacion viva: pregunta, espera respuesta, si no responde
     toma la iniciativa (chiste, consejo, sugerencia de menu).
  6. Clasifica las respuestas del usuario por palabras clave (positivo, negativo,
     chiste, menu, saludo, etc.) y responde coherentemente.
  7. Responde a gestos de mano: palma abierta (silencio) y pulgar arriba (reactivar)
  8. Puede activar la pantalla de carga (idle_video) cuando esta inactivo
"""

import os
import sys
import time
import random
import threading
import wave
import tempfile
import json
import subprocess
import re

import cv2
import numpy as np
import pyaudio
import mediapipe as mp

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Piper TTS - mismo que usan los otros nodos
from piper import PiperVoice
from piper.config import SynthesisConfig
from playsound import playsound

# Vosk STT liviano para confirmaciones
from vosk import Model, KaldiRecognizer, SetLogLevel
SetLogLevel(-1)

# -----------------------------------------------------------------------------
#  Configuracion
# -----------------------------------------------------------------------------
BRAIN_CHECK_INTERVAL   = 1.0   # segundos entre ciclos del bucle de vida (más reactivo)
PERSON_COOLDOWN        = 30.0  # segundos antes de saludar a la misma persona
BOREDOM_THRESHOLD      = 45.0  # segundos solo antes de hacer algo (más proactivo)
SUGGESTION_COOLDOWN    = 60.0  # segundos entre sugerencias ambientales
SILENCE_RMS_THRESHOLD  = 300   # nivel de ruido para considerar "silencio"
HANDSHAKE_TIMEOUT      = 4.0   # segundos esperando saludo del humano (más ágil)
IDLE_VIDEO_THRESHOLD   = 120.0 # segundos inactivo antes de poner video de carga

# Nuevos tiempos para la conversación offline
CONVERSATION_TIMEOUT   = 4.0   # tiempo máximo para esperar respuesta del usuario
PROACTIVE_INTERVAL     = 3.0   # tiempo de inactividad para tomar iniciativa
PAUSA_POST_HABLA       = 0.5   # pausa después de hablar para dar espacio al usuario

# Rutas de modelos - mismas que usa tts_lifecycle_node.py
WS_DIR = os.path.expanduser("~/robotis_ws")

TTS_MODEL_ES  = os.path.join(WS_DIR, "src/YAREN2/yaren_chat/models/TTS/es_MX-claude-high.onnx")
TTS_CONFIG_ES = os.path.join(WS_DIR, "src/YAREN2/yaren_chat/models/TTS/es_MX-claude-high.onnx.json")
TTS_MODEL_EN  = os.path.join(WS_DIR, "src/YAREN2/yaren_chat/models/TTS/en_US-lessac-medium.onnx")
TTS_CONFIG_EN = os.path.join(WS_DIR, "src/YAREN2/yaren_chat/models/TTS/en_US-lessac-medium.onnx.json")

VOSK_MODEL_ES = os.path.join(WS_DIR, "src/YAREN2/yaren_chat/models/STT/vosk-model-small-es-0.42")
VOSK_MODEL_EN = os.path.join(WS_DIR, "src/YAREN2/yaren_chat/models/STT/vosk-model-en-us-0.22-lgraph")


# -----------------------------------------------------------------------------
#  Estado del Brain
# -----------------------------------------------------------------------------

class BrainState:
    """Estado interno del cerebro para tomar decisiones"""
    IDLE = "idle"              # Esperando, sin iniciativa
    GREETING = "greeting"      # Saludando a alguien
    SUGGESTING = "suggesting"  # Sugiriendo algo
    IDLE_VIDEO = "idle_video"  # Mostrando video de carga
    SLEEPING = "sleeping"      # Apagado (solo wake word)
    WAKE_WORD = "wake_word"    # Escuchando wake word
    CONVERSATION = "conversation"  # Conversando offline


# -----------------------------------------------------------------------------
#  Frases por idioma (base de datos ampliada)
# -----------------------------------------------------------------------------

PHRASES = {
    "es": {
        "greetings": [
            "Hola Soy Yaren, tu amigo robot. Que bueno verte",
            "Hola Me alegra que estes aqui. Soy Yaren.",
            "Eh, hola No esperaba visita. Soy Yaren, mucho gusto.",
            "¡Qué sorpresa verte! Soy Yaren, tu robot favorito."
        ],
        "handshake_ask": "Me das un saludo",
        "handshake_received": "Genial Es un placer conocerte.",
        "handshake_timeout": "No te preocupes, no muerdo... al menos no hoy.",
        "ask_name": "Por cierto, como te llamas",
        "greet_by_name": "{name} Que bueno verte de nuevo.",
        "bored_comments": [
            "Hmm... esto esta muy tranquilo. Hay alguien por ahi",
            "Me pregunto que hora sera...",
            "Creo que necesito estirarme un poco.",
            "¿Será que todos se fueron? Qué soledad."
        ],
        "silence_suggestion": (
            "Este lugar esta bastante silencioso. "
            "Quieres que ponga un poco de musica para animar el ambiente"
        ),
        "music_confirmed": "Perfecto Pondre algo de musica. Quieres que baile tambien",
        "music_dance_yes": "Bueno, aqui voy",
        "music_dance_no": "Esta bien, solo la musica entonces.",
        "music_declined": "Como quieras, aqui estare si me necesitas.",
        "menu_intro": "Claro, dejame contarte que puedo hacer.",
        "menu_items": [
            ("yaren_chat",     "Primero esta el modo Chat, donde podemos conversar de cualquier cosa."),
            ("yaren_dice",     "Tambien tengo Yaren Dice, un juego donde tu me imitas o yo te imito a ti."),
            ("radio_musica",   "Luego esta Yaren Radio, donde pongo musica y hasta bailo."),
            ("yaren_emotions", "Puedo detectar tus emociones mirando tu cara."),
            ("yaren_filtros",  "Y tengo filtros divertidos, como de animales o accesorios."),
        ],
        "menu_question": "Cual de estos te llama mas la atencion",
        "yes_words": ["si", "s", "yes", "dale", "pon", "claro", "va", "bueno", "ok", "vale", "simon", "arre"],
        "no_words":  ["no", "nope", "mejor no", "nah", "paso", "ni loco", "nel"],

        # --- NUEVA BASE DE DATOS DE CONVERSACION OFFLINE ---
        "conversation": {
            "starter_questions": [
                "¿Cómo estuvo tu día, {name}?",
                "¿Qué tal te fue hoy, {name}?",
                "¿Tienes algún plan interesante para hoy, {name}?",
                "¿Qué es lo mejor que te pasó esta semana, {name}?",
                "¿Qué te trae por aquí hoy, {name}?",
                "¿Cómo te sientes hoy, {name}?",
                "¿Qué has estado haciendo últimamente, {name}?",
            ],
            "jokes": [
                "¿Por qué los programadores prefieren el otoño? Porque es la estación de las hojas (hojas de código).",
                "¿Qué le dice un techo a otro? ¡Te echo de menos!",
                "¿Cómo se llama el campeón de buceo japonés? Tokofondo.",
                "¿Qué hace una abeja en el gimnasio? ¡Zum-ba!",
                "¿Qué le dice un semáforo a otro? No me mires, que me estoy cambiando.",
                "¿Cuál es el colmo de un electricista? Que su mujer le corte el cable.",
                "¿Qué hace un perro con un taladro? Perfora.",
                "¿Cómo se llama el campeón de los juegos de mesa? El dominó-nador.",
                "¿Qué le dijo el cero al ocho? ¡Qué bonito cinturón llevas!",
                "¿Por qué los pájaros vuelan hacia el sur? Porque caminando tardarían mucho.",
                "¿Qué hace una impresora en una fiesta? ¡Saca copias!",
                "¿Cómo se llama el médico de los dinosaurios? El triceratopsiquiatra.",
            ],
            "advice": [
                "Recuerda: no todo lo que brilla es oro, a veces es solo reflejo de una pantalla.",
                "Si algo te preocupa, pregúntate: ¿esto importará en 5 años? Si no, ríete y sigue.",
                "La vida es como una bicicleta: para mantener el equilibrio, hay que seguir adelante.",
                "No dejes para mañana lo que puedes hacer pasado mañana... eso da más margen.",
                "El éxito no es la clave de la felicidad. La felicidad es la clave del éxito.",
                "La paciencia es amarga, pero su fruto es dulce.",
                "A veces la mejor respuesta es no decir nada y sonreír.",
                "No juzgues a nadie por su pasado, todos tenemos un borrador.",
                "Una sonrisa cuesta menos que la electricidad y da más luz.",
                "Si no puedes volar, corre. Si no puedes correr, anda. Si no puedes andar, gatea. Pero nunca dejes de avanzar.",
            ],
            "trivia": [
                "¿Sabías que los pulpos tienen tres corazones?",
                "El corazón de una ballena azul pesa tanto como un coche pequeño.",
                "Los humanos compartimos el 60% de nuestro ADN con las bananas.",
                "Un día en Venus dura más que un año en Venus.",
                "Los canguros no pueden caminar hacia atrás.",
                "Las huellas dactilares de los koalas son casi idénticas a las humanas.",
                "El ojo de un avestruz es más grande que su cerebro.",
                "La Tierra es el único planeta conocido que tiene placas tectónicas.",
                "Un grupo de flamingos se llama 'flamboyance'.",
                "Los elefantes son los únicos mamíferos que no pueden saltar.",
                "El chocolate blanco no es realmente chocolate, no contiene sólidos de cacao.",
                "Las estrellas de mar no tienen cerebro, pero pueden regenerar extremidades.",
            ],
            "positive_responses": [
                "¡Qué bien, me alegra oír eso!",
                "Excelente, eso es genial.",
                "Me encanta cuando las cosas van bien.",
                "¡Fantástico! Sigue así.",
                "Eso es música para mis oídos.",
                "Qué bonito, me alegra mucho.",
            ],
            "negative_responses": [
                "Vaya, eso suena complicado. ¿Quieres un consejo?",
                "No te preocupes, todo pasa. A veces solo hay que respirar hondo.",
                "Lo siento, ¿quieres que te cuente un chiste para animarte?",
                "Ánimo, después de la tormenta siempre sale el sol.",
                "A veces las cosas difíciles nos hacen más fuertes.",
                "Si necesitas desahogarte, aquí estoy para escucharte.",
            ],
            "menu_suggestion": [
                "¿Qué te parece si probamos algo divertido? Podemos jugar a Yaren Dice, poner música o usar filtros.",
                "Si quieres, podemos cambiar de actividad. Tengo modos de chat, juegos y música.",
                "¿Te apetece hacer algo diferente? Tengo muchas opciones en el menú.",
                "Podríamos probar el modo de emociones o los filtros, ¿qué dices?",
            ],
            "generic_replies": [
                "Interesante, cuéntame más.",
                "No sabía eso, ¿y cómo te sientes al respecto?",
                "Vaya, qué curioso.",
                "Entiendo, ¿y qué piensas hacer?",
                "Claro, sigue, te escucho.",
                "¿En serio? Eso es sorprendente.",
            ],
            "farewells": [
                "Ha sido un placer hablar contigo, {name}. ¡Hasta luego!",
                "Cuídate mucho, {name}. Espero verte pronto.",
                "Nos vemos, {name}. Siempre es un gusto.",
                "Adiós, {name}. Que tengas un excelente día.",
            ],
            "fallback": [
                "No estoy seguro de entenderte, pero me encanta escucharte.",
                "A veces las palabras no son necesarias, pero me gusta intentarlo.",
                "No sé qué decir, pero me caes bien.",
                "Me has dejado sin palabras... ¡y eso que soy un robot!",
            ]
        }
    },
    "en": {
        "greetings": [
            "Hi there I'm Yaren, your robot friend. Great to see you",
            "Hello I'm so glad you're here. I'm Yaren.",
            "Oh hey Didn't expect a visitor. I'm Yaren, nice to meet you",
            "What a surprise to see you! I'm Yaren, your favorite robot."
        ],
        "handshake_ask": "Want to shake hands",
        "handshake_received": "Great It's a pleasure to meet you.",
        "handshake_timeout": "Don't worry, I don't bite... at least not today.",
        "ask_name": "By the way, what's your name",
        "greet_by_name": "{name} Great to see you again.",
        "bored_comments": [
            "Hmm... it's pretty quiet here. Is anyone around",
            "I wonder what time it is...",
            "I think I need to stretch a little.",
            "Did everyone leave? What a loneliness."
        ],
        "silence_suggestion": (
            "It's quite silent here. "
            "Want me to play some music to liven things up"
        ),
        "music_confirmed": "Perfect I'll play some music. Want me to dance too",
        "music_dance_yes": "Alright, here I go",
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
        "menu_question": "Which one sounds most interesting to you",
        "yes_words": ["yes", "yeah", "sure", "ok", "okay", "go", "do it", "play", "alright", "yep"],
        "no_words":  ["no", "nope", "nah", "pass", "not really", "no way"],

        # --- NUEVA BASE DE DATOS DE CONVERSACION OFFLINE (INGLÉS) ---
        "conversation": {
            "starter_questions": [
                "How was your day, {name}?",
                "How did it go today, {name}?",
                "Any interesting plans for today, {name}?",
                "What's the best thing that happened to you this week, {name}?",
                "What brings you here today, {name}?",
                "How are you feeling today, {name}?",
                "What have you been up to lately, {name}?",
            ],
            "jokes": [
                "Why do programmers prefer autumn? Because it's the season of leaves (code leaves).",
                "What did the roof say to the other roof? I've got you covered!",
                "What do you call a fish with no eyes? A fsh.",
                "Why don't scientists trust atoms? Because they make up everything.",
                "What do you call a bear with no teeth? A gummy bear.",
                "What's the best thing about Switzerland? I don't know, but the flag is a big plus.",
                "Why did the scarecrow win an award? Because he was outstanding in his field.",
                "What do you call a fake noodle? An impasta.",
                "Why did the bicycle fall over? Because it was two-tired.",
                "What do you call a sleeping dinosaur? A dino-snore.",
            ],
            "advice": [
                "Remember: not all that glitters is gold, sometimes it's just a screen reflection.",
                "If something worries you, ask yourself: will this matter in 5 years? If not, laugh and move on.",
                "Life is like a bicycle: to keep balance, you must keep moving.",
                "Don't put off until tomorrow what you can do the day after tomorrow... that gives more room.",
                "Success is not the key to happiness. Happiness is the key to success.",
                "Patience is bitter, but its fruit is sweet.",
                "Sometimes the best response is to say nothing and smile.",
                "Don't judge anyone by their past, we all have a draft.",
                "A smile costs less than electricity and gives more light.",
                "If you can't fly, run. If you can't run, walk. If you can't walk, crawl. But never stop moving forward.",
            ],
            "trivia": [
                "Did you know that octopuses have three hearts?",
                "A blue whale's heart weighs as much as a small car.",
                "Humans share 60% of our DNA with bananas.",
                "A day on Venus lasts longer than a year on Venus.",
                "Kangaroos cannot walk backwards.",
                "Koala fingerprints are almost identical to human ones.",
                "An ostrich's eye is bigger than its brain.",
                "Earth is the only known planet with tectonic plates.",
                "A group of flamingos is called a 'flamboyance'.",
                "Elephants are the only mammals that cannot jump.",
                "White chocolate is not really chocolate, it contains no cocoa solids.",
                "Starfish have no brain, but they can regenerate limbs.",
            ],
            "positive_responses": [
                "Great, I'm glad to hear that!",
                "Excellent, that's awesome.",
                "I love it when things go well.",
                "Fantastic! Keep it up.",
                "That's music to my ears.",
                "How nice, I'm really happy for you.",
            ],
            "negative_responses": [
                "Wow, that sounds tough. Would you like some advice?",
                "Don't worry, everything passes. Sometimes you just need to breathe deeply.",
                "I'm sorry, would you like a joke to cheer you up?",
                "Cheer up, after the storm comes the calm.",
                "Sometimes difficult things make us stronger.",
                "If you need to vent, I'm here to listen.",
            ],
            "menu_suggestion": [
                "How about we try something fun? We can play Yaren Says, play music, or use filters.",
                "If you want, we can switch activities. I have chat, games, and music modes.",
                "Feel like doing something different? I have many options in the menu.",
                "We could try the emotions mode or filters, what do you say?",
            ],
            "generic_replies": [
                "Interesting, tell me more.",
                "I didn't know that, how do you feel about it?",
                "Wow, how curious.",
                "I see, and what are you going to do?",
                "Sure, go on, I'm listening.",
                "Really? That's surprising.",
            ],
            "farewells": [
                "It was a pleasure talking to you, {name}. See you later!",
                "Take care, {name}. Hope to see you soon.",
                "See you, {name}. Always a pleasure.",
                "Goodbye, {name}. Have a great day.",
            ],
            "fallback": [
                "I'm not sure I understand, but I love listening to you.",
                "Sometimes words are not necessary, but I like trying.",
                "I don't know what to say, but I like you.",
                "You've left me speechless... and I'm a robot!",
            ]
        }
    }
}


# -----------------------------------------------------------------------------
#  Nodo principal
# -----------------------------------------------------------------------------

class YarenBrainNode(Node):

    def __init__(self):
        super().__init__("yaren_brain_node")

        # -- Estado del mundo ----------------------------------------------
        self.face_idle        = False   # Pantalla libre
        self.audio_playing    = False   # TTS hablando
        self.mic_owner        = "none"  # Quien tiene el microfono
        self.is_english       = False
        self.latest_frame     = None
        self._bridge          = CvBridge()
        self._frame_lock      = threading.Lock()
        self.system_ready     = False   # True cuando termino la carga inicial
        self.camera_active    = False   # True cuando la camara esta activa

        # -- Estado del cerebro --------------------------------------------
        self.brain_state = BrainState.IDLE
        self.person_visible         = False
        self.last_person_greeted_at = 0.0
        self.last_interaction_at    = time.time()
        self.last_suggestion_at     = 0.0
        self.last_idle_video_at     = 0.0
        self.idle_video_active      = False
        self.known_persons          = {}   # nombre -> ultima vez visto
        self.greeting_in_progress   = False
        self.brain_active           = False  # Solo actua cuando face_idle=True

        # -- Estado de conversacion offline --------------------------------
        self.conv_state = 'idle'          # 'idle', 'waiting_response', 'proactive'
        self.user_name = ""
        self.last_question_time = 0.0
        self.conv_history = []            # lista de frases dichas para evitar repetición
        self.conv_reply_history = []      # para evitar respuestas repetidas
        self.conv_active = False

        # -- Gestos visuales (interruptor por mano) ------------------------
        self.personality_enabled = True          # True = cerebro activo, False = modo silencio
        self.gesture_cooldown = 0.0              # Para evitar detecciones constantes
        self.gesture_frame_counter = 0           # Para procesar solo cada N frames
        self.latest_raw_frame = None             # Frame sin flip para gestos

        # Inicializar MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils  # (opcional, para debug)

        # -- TTS -----------------------------------------------------------
        self._voice_es   = None
        self._voice_en   = None
        self._voice_lock = threading.Lock()
        self._load_tts()

        # -- STT liviano para confirmaciones y conversacion ----------------
        self._vosk_model  = None
        self._recognizer  = None
        self._mic         = None
        self._stream      = None
        self._load_stt()

        # -- Deteccion de personas (HOG liviano, sin YOLO extra) ----------
        self._hog = cv2.HOGDescriptor()
        self._hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # -- QoS transient local para topics de estado --------------------
        qos_tl = QoSProfile(depth=1,
                            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        # -- Suscripciones ------------------------------------------------
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

        # -- Publicadores ------------------------------------------------
        self._mode_pub      = self.create_publisher(String, "/yaren_mode",        10)
        self._mic_owner_pub = self.create_publisher(String, "/yaren/mic_owner",   qos_tl)
        self._audio_pub     = self.create_publisher(Bool,   "/audio_playing",     10)
        self._highlight_pub = self.create_publisher(String, "/yaren/brain_highlight", 10)
        self._brain_state_pub = self.create_publisher(String, "/yaren/brain_state", qos_tl)
        self._face_idle_pub = self.create_publisher(Bool,   "/yaren/face_idle", qos_tl)
        self._wakeword_pub = self.create_publisher(Bool, "/yaren/wakeword_enabled", qos_tl)

        # -- Bucle de vida ------------------------------------------------
        self.create_timer(BRAIN_CHECK_INTERVAL, self._life_loop)

        # -- Esperar a que el sistema este listo --------------------------
        self._wait_for_system_ready()

        self.get_logger().info("YarenBrainNode iniciado. Esperando que el sistema este listo...")

    # -- Esperar a que el sistema termine de cargar ------------------------

    def _wait_for_system_ready(self):
        """Espera a que face_screen termine la configuracion inicial."""
        def wait():
            self.get_logger().info("Esperando senal de sistema listo...")
            
            ready_received = False
            
            def system_ready_callback(msg):
                nonlocal ready_received
                if msg.data:
                    ready_received = True
                    self.get_logger().info("Senal de sistema listo recibida.")
            
            # Crear suscripcion temporal
            sub = self.create_subscription(
                Bool, 
                "/yaren/system_ready",
                system_ready_callback,
                QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            )
            
            # Esperar hasta recibir la senal o timeout (5 minutos maximo)
            timeout = 300
            start_time = time.time()
            
            while not ready_received and (time.time() - start_time) < timeout:
                # Tambien verificamos si face_idle ya es True (sistema listo)
                if self.face_idle:
                    self.get_logger().info("face_idle=True detectado. Sistema listo.")
                    ready_received = True
                    break
                time.sleep(1.0)
            
            # Eliminar suscripcion temporal
            self.destroy_subscription(sub)
            
            if ready_received:
                self.system_ready = True
                self.brain_active = True
                self.get_logger().info("Sistema listo. Cerebro activado.")
                self._publish_brain_state("active")
                # Activar camara
                self._activate_camera()
                self._set_wakeword_enabled(False)  # brain arranca activo -> wake word apagado
            else:
                self.get_logger().warning("Timeout esperando sistema listo. Activando cerebro de todas formas.")
                self.system_ready = True
                self.brain_active = True
                self._publish_brain_state("active")
                self._activate_camera()
                self._set_wakeword_enabled(False)
                
        threading.Thread(target=wait, daemon=True).start()

    # -- Activar camara ---------------------------------------------------

    def _activate_camera(self):
        """Activa el nodo lifecycle de la camara."""
        def activate():
            try:
                if self.camera_active:
                    return
                
                self.get_logger().info("Activando camara...")
                
                # 1. Configurar la camara
                result = subprocess.run(
                    "ros2 lifecycle set /csi_cam_node configure",
                    shell=True, capture_output=True, text=True, timeout=5
                )
                
                if result.returncode != 0:
                    self.get_logger().warn(f"Error al configurar camara: {result.stderr}")
                    return
                
                time.sleep(0.5)
                
                # 2. Activar la camara
                result = subprocess.run(
                    "ros2 lifecycle set /csi_cam_node activate",
                    shell=True, capture_output=True, text=True, timeout=5
                )
                
                if result.returncode == 0:
                    self.camera_active = True
                    self.get_logger().info("Camara activada correctamente.")
                else:
                    self.get_logger().warn(f"Error al activar camara: {result.stderr}")
                    
            except subprocess.TimeoutExpired:
                self.get_logger().warn("Timeout activando camara.")
            except Exception as e:
                self.get_logger().warn(f"No se pudo activar camara: {e}")
        
        threading.Thread(target=activate, daemon=True).start()

    # -- Carga de modelos -------------------------------------------------

    def _load_tts(self):
        """Carga voces TTS en segundo plano para no bloquear el arranque."""
        def _load():
            try:
                self._voice_es = PiperVoice.load(TTS_MODEL_ES, TTS_CONFIG_ES, use_cuda=False)
                self.get_logger().info("Voz ES lista.")
            except Exception as e:
                self.get_logger().warn(f"TTS ES no disponible: {e}")
            try:
                self._voice_en = PiperVoice.load(TTS_MODEL_EN, TTS_CONFIG_EN, use_cuda=False)
                self.get_logger().info("Voz EN lista.")
            except Exception as e:
                self.get_logger().warn(f"TTS EN no disponible: {e}")
        threading.Thread(target=_load, daemon=True).start()

    def _load_stt(self):
        """Carga modelo Vosk liviano para confirmaciones y escucha."""
        def _load():
            path = VOSK_MODEL_EN if self.is_english else VOSK_MODEL_ES
            if not os.path.exists(path):
                path = VOSK_MODEL_ES if not os.path.exists(VOSK_MODEL_ES) else None
            if path and os.path.exists(path):
                try:
                    self._vosk_model = Model(path)
                    self._recognizer = KaldiRecognizer(self._vosk_model, 16000)
                    self._mic = pyaudio.PyAudio()
                    self.get_logger().info("STT liviano listo.")
                except Exception as e:
                    self.get_logger().warn(f"STT no disponible: {e}")
        threading.Thread(target=_load, daemon=True).start()

    # -- Callbacks de topics ----------------------------------------------

    def _cb_face_idle(self, msg: Bool):
        was_idle = self.face_idle
        self.face_idle = msg.data
        if msg.data and not was_idle and self.system_ready:
            self.get_logger().info("Pantalla libre - cerebro activo.")
            self.brain_active = True
            self._publish_brain_state("active")
        elif not msg.data:
            self.brain_active = False
            self._publish_brain_state("inactive")

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
            with self._frame_lock:
                self.latest_raw_frame = frame.copy()
                self.latest_frame = cv2.flip(frame, 0)
        except Exception:
            pass

    # -- Deteccion de gestos con MediaPipe --------------------------------

    def _detect_gesture(self, frame) -> str:
        """Analiza la mano y devuelve 'stop' o 'resume' o None."""
        if frame is None:
            return None

        small = cv2.resize(frame, (160, 120))
        rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        if not results.multi_hand_landmarks:
            return None

        landmarks = results.multi_hand_landmarks[0].landmark

        thumb_tip  = landmarks[4].y
        thumb_ip   = landmarks[3].y
        index_tip  = landmarks[8].y
        index_pip  = landmarks[6].y
        middle_tip = landmarks[12].y
        middle_pip = landmarks[10].y
        ring_tip   = landmarks[16].y
        ring_pip   = landmarks[14].y
        pinky_tip  = landmarks[20].y
        pinky_pip  = landmarks[18].y

        thumb_up   = thumb_tip < thumb_ip
        index_up   = index_tip < index_pip
        middle_up  = middle_tip < middle_pip
        ring_up    = ring_tip < ring_pip
        pinky_up   = pinky_tip < pinky_pip

        if thumb_up and index_up and middle_up and ring_up and pinky_up:
            return "stop"

        if thumb_up and not index_up and not middle_up and not ring_up and not pinky_up:
            return "resume"

        return None

    def _process_gesture(self):
        """Mira si hay un gesto de 'stop' o 'resume' y cambia el estado."""
        with self._frame_lock:
            frame = self.latest_raw_frame
        if frame is None:
            return

        now = time.time()
        if now - self.gesture_cooldown < 3.0:
            return

        gesture = self._detect_gesture(frame)

        if gesture == "stop" and self.personality_enabled:
            self.get_logger().info("Gesto de SILENCIO detectado. Desactivando personalidad.")
            self.personality_enabled = False
            self._set_wakeword_enabled(True)
            self.gesture_cooldown = now
            # Detener conversación activa
            self.conv_active = False
            self.conv_state = 'idle'
            if self.is_english:
                self._speak("Understood. I'll stay quiet for now.")
            else:
                self._speak("Entendido. Me quedare en silencio por ahora.")

        elif gesture == "resume" and not self.personality_enabled:
            self.get_logger().info("Gesto de REANUDAR detectado. Activando personalidad.")
            self.personality_enabled = True
            self._set_wakeword_enabled(False)
            self.gesture_cooldown = now
            if self.is_english:
                self._speak("I'm back Ready to interact.")
            else:
                self._speak("Volvi Listo para interactuar.")
            self.last_interaction_at = time.time()
            # Si hay persona visible, reiniciar conversación
            if self.person_visible:
                self._start_conversation(self.user_name if self.user_name else "amigo")

    # -- Bucle de vida principal ------------------------------------------

    def _life_loop(self):
        """Corre cada BRAIN_CHECK_INTERVAL segundos."""
        if self.system_ready:
            self._wakeword_heartbeat_counter += 1
            if self._wakeword_heartbeat_counter % 4 == 0:   # ~cada 10s
                self._set_wakeword_enabled(not self.personality_enabled)

        if not self.system_ready:
            return

        # Procesar gestos (1 de cada 3 iteraciones)
        self.gesture_frame_counter += 1
        if self.gesture_frame_counter % 3 == 0:
            self._process_gesture()

        # Solo actua si la pantalla esta libre y nadie esta hablando.
        if not self.brain_active:
            return
        if self.audio_playing:
            return
        if self.greeting_in_progress:
            return
        if self.mic_owner not in ("none", "brain"):
            return
        if not self.personality_enabled:
            return

        # Si la camara no esta activa, intentar activarla
        if not self.camera_active:
            self._activate_camera()
            time.sleep(1)
            if not self.camera_active:
                return

        now = time.time()

        # 1. Detectar personas en el frame actual
        person_now = self._detect_person()

        # -- PRIORIDAD 1: Nueva persona detectada -------------------------
        if person_now and not self.person_visible:
            since_last = now - self.last_person_greeted_at
            if since_last > PERSON_COOLDOWN:
                self.person_visible = True
                threading.Thread(target=self._greet_sequence, daemon=True).start()
                return

        self.person_visible = person_now

        # -- PRIORIDAD 2: Si estamos en conversacion, manejarla -----------
        if self.conv_active and self.person_visible:
            self._handle_conversation_turn()
            return  # No ejecutar otras acciones mientras conversamos

        # -- PRIORIDAD 3: Inactividad prolongada -> video de carga --------
        if not person_now:
            since_interaction = now - self.last_interaction_at
            if since_interaction > IDLE_VIDEO_THRESHOLD and not self.idle_video_active:
                self.idle_video_active = True
                self.last_idle_video_at = now
                self._activate_idle_video()
                return

        # -- PRIORIDAD 4: Silencio prolongado -> sugerir musica ----------
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

        # -- PRIORIDAD 5: Aburrido y solo ---------------------------------
        if not person_now:
            since_interaction = now - self.last_interaction_at
            if since_interaction > BOREDOM_THRESHOLD:
                self.last_interaction_at = now
                threading.Thread(target=self._bored_behavior,
                                 daemon=True).start()

    # -- Deteccion de persona (HOG) ---------------------------------------

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

    # -- Medicion de silencio ambiental -----------------------------------

    def _measure_silence(self) -> bool:
        """Lee 0.3 segundos del microfono y calcula RMS."""
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

    # -- TTS del brain ---------------------------------------------------

    def _speak(self, text: str):
        """Sintesis y reproduccion directa, bloqueante."""
        voice = self._voice_en if self.is_english else self._voice_es
        if voice is None:
            self.get_logger().warn(f"[BRAIN] Sin voz TTS. Texto: {text}")
            return

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

    # -- Escuchar respuesta (no bloqueante para usar en timer) ------------

    def _listen_for_response(self, timeout=1.0) -> str:
        """
        Escucha brevemente y retorna el texto reconocido o cadena vacía.
        No bloquea más de 'timeout' segundos.
        """
        if self._vosk_model is None or self._mic is None:
            return ""

        owner_msg = String(); owner_msg.data = "brain"
        self._mic_owner_pub.publish(owner_msg)
        time.sleep(0.1)

        text = ""
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
                        break
            stream.stop_stream()
            stream.close()
        except Exception as e:
            self.get_logger().warn(f"[BRAIN] STT error: {e}")

        owner_msg.data = "none"
        self._mic_owner_pub.publish(owner_msg)
        return text

    # -- Escuchar confirmacion corta (si/no) -----------------------------

    def _listen_for_confirmation(self, timeout: float = 5.0) -> str:
        """Escucha y retorna 'yes', 'no' o 'timeout'."""
        if self._vosk_model is None or self._mic is None:
            return "timeout"

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

        owner_msg.data = "none"
        self._mic_owner_pub.publish(owner_msg)
        return result

    # -- Publicar estado del brain ---------------------------------------

    def _publish_brain_state(self, state: str):
        msg = String(); msg.data = state
        self._brain_state_pub.publish(msg)

    def _set_wakeword_enabled(self, enabled: bool):
        msg = Bool(); msg.data = enabled
        self._wakeword_pub.publish(msg)
        self.get_logger().info(f"[BRAIN] wake_word -> {'ENABLED' if enabled else 'DISABLED'}")

    # -- Activar video de carga -------------------------------------------

    def _activate_idle_video(self):
        self.get_logger().info("Activando pantalla de carga (idle video).")
        self._publish_mode("idle_video")
        self.brain_active = False
        self._publish_brain_state("idle_video")

        def reactivate():
            time.sleep(30)
            if self.idle_video_active:
                self.idle_video_active = False
                self.brain_active = True
                self.last_interaction_at = time.time()
                self._publish_brain_state("active")
                self.get_logger().info("Reactivando cerebro despues de video.")
        threading.Thread(target=reactivate, daemon=True).start()

    # -- Publicar modo ----------------------------------------------------

    def _publish_mode(self, mode: str):
        msg = String(); msg.data = mode
        self._mode_pub.publish(msg)

    def _highlight_button(self, mode_id: str):
        msg = String(); msg.data = mode_id
        self._highlight_pub.publish(msg)

    # ----------------------------------------------------------------------
    #  NUEVAS FUNCIONES DE CONVERSACION OFFLINE
    # ----------------------------------------------------------------------

    def _get_phrase(self, category, subcategory=None, **kwargs):
        """Obtiene una frase aleatoria de la base de datos, evitando repeticiones."""
        lang = "en" if self.is_english else "es"
        conv = PHRASES[lang]["conversation"]
        if subcategory:
            pool = conv.get(subcategory, [])
        else:
            pool = conv.get(category, [])
        if not pool:
            return ""

        # Evitar repetir las últimas 3 frases
        history = self.conv_history if subcategory == "starter_questions" else self.conv_reply_history
        candidates = [p for p in pool if p not in history[-3:]]
        if not candidates:
            candidates = pool
        chosen = random.choice(candidates)
        # Actualizar historial
        if subcategory == "starter_questions":
            self.conv_history.append(chosen)
            if len(self.conv_history) > 20:
                self.conv_history = self.conv_history[-10:]
        else:
            self.conv_reply_history.append(chosen)
            if len(self.conv_reply_history) > 20:
                self.conv_reply_history = self.conv_reply_history[-10:]

        if kwargs:
            try:
                return chosen.format(**kwargs)
            except KeyError:
                return chosen
        return chosen

    def _start_conversation(self, name):
        """Inicia el bucle conversacional offline."""
        self.user_name = name
        self.conv_active = True
        self.conv_state = 'waiting_response'
        self.last_question_time = time.time()
        # Pregunta inicial
        question = self._get_phrase("conversation", "starter_questions", name=name)
        self._speak(question)
        self.last_interaction_at = time.time()

    def _ask_next_question(self):
        """Hace una nueva pregunta al usuario."""
        if not self.conv_active:
            return
        question = self._get_phrase("conversation", "starter_questions", name=self.user_name)
        self._speak(question)
        self.last_question_time = time.time()
        self.conv_state = 'waiting_response'

    def _handle_conversation_turn(self):
        """Se llama cada ciclo de vida para gestionar la conversación."""
        if not self.conv_active or self.conv_state == 'idle':
            return

        # Si estamos esperando respuesta y ha pasado mucho tiempo
        if self.conv_state == 'waiting_response':
            elapsed = time.time() - self.last_question_time
            if elapsed > CONVERSATION_TIMEOUT:
                # El usuario no responde -> tomar iniciativa
                self._proactive_turn()
                return

            # Intentar escuchar (lectura no bloqueante)
            response = self._listen_for_response(timeout=1.0)
            if response:
                self._process_user_response(response)
                return

        # Si estamos en modo proactivo (esperando confirmación, etc.) ya se maneja en _proactive_turn

    def _process_user_response(self, text):
        """Procesa la respuesta del usuario y genera una reacción."""
        self.get_logger().info(f"[BRAIN] Usuario dijo: {text}")
        lang = "en" if self.is_english else "es"
        conv = PHRASES[lang]["conversation"]

        intent = self._classify_intent(text)

        if intent == "positive":
            reply = self._get_phrase("conversation", "positive_responses")
            self._speak(reply)
            # Después de un momento, preguntar otra cosa
            self.last_question_time = time.time()
            self.conv_state = 'waiting_response'
            # Programar siguiente pregunta en el próximo ciclo
            threading.Timer(1.5, self._ask_next_question).start()

        elif intent == "negative":
            reply = self._get_phrase("conversation", "negative_responses")
            self._speak(reply)
            # Añadir consejo
            advice = self._get_phrase("conversation", "advice")
            self._speak(advice)
            self._ask_next_question()

        elif intent == "joke":
            joke = self._get_phrase("conversation", "jokes")
            self._speak(joke)
            self._ask_next_question()

        elif intent == "menu":
            suggestion = self._get_phrase("conversation", "menu_suggestion")
            self._speak(suggestion)
            self.present_menu()
            self.conv_active = False
            self.conv_state = 'idle'

        elif intent == "greeting":
            self._speak(f"¡Hola {self.user_name}! ¿Cómo vas?")
            self._ask_next_question()

        elif intent == "farewell":
            farewell = self._get_phrase("conversation", "farewells", name=self.user_name)
            self._speak(farewell)
            self.conv_active = False
            self.conv_state = 'idle'

        else:  # unknown
            # Decir algo interesante y cambiar de tema
            fallback = self._get_phrase("conversation", "fallback")
            self._speak(fallback)
            trivia = self._get_phrase("conversation", "trivia")
            self._speak(trivia + " ¿Sabías eso?")
            self._ask_next_question()

        self.last_interaction_at = time.time()

    def _classify_intent(self, text):
        """Clasifica la intención del usuario usando palabras clave."""
        text = text.lower()
        lang = "en" if self.is_english else "es"
        # Palabras clave positivas
        pos = ["bien", "genial", "excelente", "feliz", "good", "great", "happy", "awesome", "fine"]
        if any(w in text for w in pos):
            return "positive"
        # Negativas
        neg = ["mal", "cansado", "triste", "estresado", "bad", "tired", "sad", "stress", "deprimido"]
        if any(w in text for w in neg):
            return "negative"
        # Chiste
        joke = ["chiste", "joke", "gracioso", "funny", "risa", "laugh"]
        if any(w in text for w in joke):
            return "joke"
        # Menú
        menu = ["que hacer", "menu", "modo", "actividad", "what to do", "mode", "activity", "jugar", "play"]
        if any(w in text for w in menu):
            return "menu"
        # Saludo
        greet = ["hola", "yaren", "hey", "hello", "hi", "que tal"]
        if any(w in text for w in greet):
            return "greeting"
        # Despedida
        farewell = ["adios", "hasta luego", "nos vemos", "bye", "goodbye", "see you", "chao"]
        if any(w in text for w in farewell):
            return "farewell"
        return "unknown"

    def _proactive_turn(self):
        """Cuando el usuario no responde, Yaren toma la iniciativa."""
        lang = "en" if self.is_english else "es"
        conv = PHRASES[lang]["conversation"]

        # Elegir entre chiste, consejo, sugerencia de menú o trivia
        actions = ['joke', 'advice', 'menu', 'trivia']
        # Dar más peso a trivia y chistes para que sea más vivo
        action = random.choices(actions, weights=[2, 2, 1, 3], k=1)[0]

        if action == 'joke':
            joke = self._get_phrase("conversation", "jokes")
            self._speak(f"Ya que no dices nada, te cuento un chiste: {joke}")
            self._speak("¿Qué te pareció?")
        elif action == 'advice':
            advice = self._get_phrase("conversation", "advice")
            self._speak(f"Se me ocurre un consejo: {advice}")
            self._speak("¿Qué opinas?")
        elif action == 'menu':
            self._speak("Parece que estás distraído. ¿Quieres que hagamos algo juntos?")
            self.present_menu()
            self.conv_active = False
            self.conv_state = 'idle'
            return
        else:  # trivia
            trivia = self._get_phrase("conversation", "trivia")
            self._speak(f"Dato curioso: {trivia}")
            self._speak("¿Sabías eso?")

        # Volvemos a esperar respuesta
        self.last_question_time = time.time()
        self.conv_state = 'waiting_response'

    # -- Secuencia de saludo (modificada para iniciar conversación) ------

    def _greet_sequence(self):
        """
        Saludo completo:
        1. Saludar con voz
        2. Pedir saludo (handshake)
        3. Esperar respuesta
        4. Preguntar nombre
        5. Registrar nombre
        6. Iniciar conversación offline
        """
        self.greeting_in_progress = True
        self.last_person_greeted_at = time.time()
        lang = "en" if self.is_english else "es"
        phrases = PHRASES[lang]

        try:
            # 1. Saludo inicial
            greeting = random.choice(phrases["greetings"])
            self._speak(greeting)
            time.sleep(0.5)

            # 2. Pedir saludo
            self._speak(phrases["handshake_ask"])

            # 3. Esperar respuesta o movimiento
            response = self._listen_for_confirmation(HANDSHAKE_TIMEOUT)

            # 4. Reaccionar
            if response in ("yes", "timeout"):
                self._speak(phrases["handshake_received"])
            else:
                self._speak(phrases["handshake_timeout"])

            time.sleep(0.5)

            # 5. Preguntar nombre si no lo conocemos
            self._speak(phrases["ask_name"])
            name = self._listen_for_name(timeout=8.0)

            if name:
                self.known_persons[name] = time.time()
                self.get_logger().info(f"Persona registrada: {name}")
                self.user_name = name
            else:
                self.user_name = "amigo"

            self.last_interaction_at = time.time()

            # 6. Iniciar conversación offline
            self._start_conversation(self.user_name)

        except Exception as e:
            self.get_logger().error(f"[BRAIN] Error en saludo: {e}")
        finally:
            self.greeting_in_progress = False

    def _listen_for_name(self, timeout: float = 8.0) -> str:
        """Escucha un nombre (cualquier palabra que no sea si/no)."""
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

    # -- Sugerencia de musica por silencio --------------------------------

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
                self._publish_mode("radio_musica")
            else:
                self._speak(phrases["music_dance_no"])
                self._publish_mode("radio_musica")
        else:
            self._speak(phrases["music_declined"])

        self.last_interaction_at = time.time()

    # -- Comportamiento cuando esta solo y aburrido -----------------------

    def _bored_behavior(self):
        lang = "en" if self.is_english else "es"
        phrases = PHRASES[lang]

        behavior = random.choice(["comment", "look_around", "stretch"])

        if behavior == "comment":
            comment = random.choice(phrases["bored_comments"])
            self._speak(comment)
        elif behavior == "look_around":
            # (desactivado para PC)
            pass
        elif behavior == "stretch":
            # (desactivado para PC)
            pass

    # -- Presentacion interactiva del menu --------------------------------

    def present_menu(self):
        """
        Llamar desde el exterior (o cuando el usuario pregunte que puede hacer).
        Presenta cada modo con voz y resalta el boton correspondiente.
        """
        lang = "en" if self.is_english else "es"
        phrases = PHRASES[lang]

        self._speak(phrases["menu_intro"])
        time.sleep(0.3)

        for mode_id, description in phrases["menu_items"]:
            self._highlight_button(mode_id)
            self._speak(description)
            time.sleep(0.4)

        self._speak(phrases["menu_question"])
        self.last_interaction_at = time.time()

    # -- Movimientos fisicos (DESACTIVADOS PARA PRUEBAS EN PC) ------------

    def _raise_right_hand(self):
        pass

    def _lower_right_hand(self):
        pass

    def _look_around_sequence(self):
        pass

    def _stretch_sequence(self):
        pass


# -----------------------------------------------------------------------------
#  Main
# -----------------------------------------------------------------------------

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