#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from std_msgs.msg import Int16, String, Bool
import time
import threading
import os
import pygame
from ament_index_python.packages import get_package_share_directory

EMOTIONS_ES = ["Enojado", "Disgusto", "Miedo", "Feliz", "Triste", "Sorpresa", "Neutral"]

CHISTES = [
    "¿Pepito, tu papá te ayuda con la tarea?\nNo maestra, él la hace solito.",
    "¿Por qué los pájaros vuelan hacia el sur en invierno?\nPorque caminando tardarían demasiado.",
    "¿Qué hace una abeja en el gimnasio?\n¡Zum-ba!",
    "¿Cómo se despiden los químicos?\nÁcido un placer.",
    "¿Qué le dice un techo a otro techo?\nTecho de menos.",
    "¿Por qué la escoba está feliz?\nPorque se enteró de que va a barrer en las elecciones.",
    "¿Por qué el libro de matemáticas estaba triste?\nPorque tenía demasiados problemas.",
    "La policía atrapó a un ciego\npero a la media hora lo dejó ir porque no tuvo nada que ver.",
    "¿Por qué la luna no come?\nPorque ya está llena.",
    "¿Qué le dice un semáforo a otro?\nNo me mires que me estoy cambiando.",
]

SOUND_PATH = os.path.join(get_package_share_directory('yaren_chistes'), 'sonidos', 'risa_sonido.mp3')

class ChistesNode(LifecycleNode):

    def __init__(self):
        super().__init__('chistes_node')
        self._current_emotions = []
        self._lock_emotions = False
        self._subscription = None
        self._audio_sub = None
        self._speak_pub = None
        self._viseme_pub = None
        self._audio_playing_pub = None
        self._running = False
        self._thread = None
        self._risa = None
        self._is_audio_playing = False

    # ── CONFIGURE ────────────────────────────────────────────────────────────
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[chistes_node] Configurando...')

        try:
            if pygame.mixer.get_init():
                pygame.mixer.quit()
            pygame.mixer.pre_init(44100, -16, 2, 2048)
            pygame.mixer.init()
            if os.path.exists(SOUND_PATH):
                self._risa = pygame.mixer.Sound(SOUND_PATH)
                self.get_logger().info('Sonido de risa cargado ✓')
            else:
                self.get_logger().warn(f'Sonido no encontrado: {SOUND_PATH}')
                self._risa = None
        except Exception as e:
            self.get_logger().error(f'Error inicializando pygame: {e}')
            return TransitionCallbackReturn.FAILURE

        self._speak_pub = self.create_publisher(String, '/yaren/speak_direct', 10)
        self._viseme_pub = self.create_publisher(String, '/yaren/tts_text', 10)
        self._audio_playing_pub = self.create_publisher(Bool, '/audio_playing', 10)
        
        self._subscription = self.create_subscription(Int16, '/emotion', self._emotion_callback, 10)
        self._audio_sub = self.create_subscription(Bool, '/audio_playing', self._audio_cb, 10)

        self.get_logger().info('[chistes_node] Configurado OK.')
        return TransitionCallbackReturn.SUCCESS

    # ── ACTIVATE ─────────────────────────────────────────────────────────────
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[chistes_node] Activando — iniciando ronda de chistes...')
        self._running = True
        self._thread = threading.Thread(target=self._run_chistes, daemon=True)
        self._thread.start()
        return super().on_activate(state)

    # ── DEACTIVATE ───────────────────────────────────────────────────────────
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[chistes_node] Desactivando...')
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3.0)
            
        try:
            pygame.mixer.stop()
        except Exception:
            pass
            
        return super().on_deactivate(state)

    # ── CLEANUP ──────────────────────────────────────────────────────────────
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[chistes_node] Limpiando recursos...')
        self._running = False
        if self._subscription:
            self.destroy_subscription(self._subscription)
            self._subscription = None
        if self._audio_sub:
            self.destroy_subscription(self._audio_sub)
            self._audio_sub = None
            
        try:
            pygame.mixer.quit()
        except Exception:
            pass
        self._risa = None
        
        return TransitionCallbackReturn.SUCCESS

    # ── SHUTDOWN ─────────────────────────────────────────────────────────────
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.on_cleanup(state)
        return TransitionCallbackReturn.SUCCESS

    # ── Callbacks internos ───────────────────────────────────────────────────
    def _audio_cb(self, msg):
        self._is_audio_playing = msg.data

    def _say_sync(self, text):
        """Envía el texto al TTS y bloquea hasta que termina de hablar"""
        msg = String()
        msg.data = text
        self._speak_pub.publish(msg)
        time.sleep(0.5)  # Dar tiempo al TTS de arrancar
        while self._is_audio_playing and self._running:
            time.sleep(0.1)

    def _play_risa_with_face(self):
        """Reproduce la risa y obliga a la pantalla a mostrar el sprite 1 inmediatamente"""
        if self._risa is not None and self._running:
            # Obtener duración del sonido en milisegundos
            dur_ms = int(self._risa.get_length() * 1000)
            
            # 1. Avisar que "hay audio" para que face_screen despierte
            self._audio_playing_pub.publish(Bool(data=True))
            
            # 2. TRUCO: "0:1" (cambio inmediato a boca 1), luego "{dur_ms}:0" (mantener y luego cerrar)
            vis_msg = String()
            vis_msg.data = f"0:1|{dur_ms}:0|"
            self._viseme_pub.publish(vis_msg)
            
            # 3. Reproducir el sonido
            self._risa.play()
            
            # 4. Esperar a que termine
            while pygame.mixer.get_busy() and self._running:
                time.sleep(0.1)
                
            # 5. Apagar la bandera de audio
            self._audio_playing_pub.publish(Bool(data=False))

    def _emotion_callback(self, msg):
        if self._lock_emotions:
            idx = msg.data
            if 0 <= idx < len(EMOTIONS_ES):
                self._current_emotions.append(EMOTIONS_ES[idx])

    def _run_chistes(self):
        self.get_logger().info('Ronda de chistes iniciada.')
        print("\n" + "="*55)
        print("         RONDA DE CHISTES - YAREN2")
        print("="*55 + "\n")

        # Introducción
        self._say_sync("Hola, te contaré algunos chistes. ¿Estás listo? Ahí voy.")
        time.sleep(1.0)

        for i, chiste in enumerate(CHISTES, start=1):
            if not self._running:
                break

            print(f"Ronda {i}:")
            print(f"Chiste:\n{chiste}")

            # Activar captura de emociones (en paralelo al TTS y la risa)
            self._current_emotions.clear()
            self._lock_emotions = True

            # 1. Separar el chiste en partes (pregunta y remate)
            partes = chiste.split('\n')
            
            for idx, parte in enumerate(partes):
                self._say_sync(parte)
                
                # Pausa dramática de 1 segundo antes del remate
                if idx < len(partes) - 1 and self._running:
                    time.sleep(1.0)

            if not self._running:
                break

            # 2. Reproducir el efecto de sonido con animación labial (Sprite 1)
            self._play_risa_with_face()

            # 3. Apagar captura de emociones y evaluar
            self._lock_emotions = False

            if self._current_emotions:
                unique = list(dict.fromkeys(self._current_emotions))
                emociones_str = (
                    " y ".join(unique) if len(unique) <= 2
                    else ", ".join(unique[:-1]) + " y " + unique[-1]
                )
            else:
                emociones_str = "Nada"

            print(f"Emociones detectadas durante el chiste: {emociones_str}")
            print("-"*55 + "\n")

            # 4. Reacción según emoción
            if self._running and emociones_str != "Nada":
                self._say_sync(f"Veo que estás {emociones_str}. ¡Qué bien!")

            # 5. Pausa de 3 segundos antes del siguiente chiste
            if self._running:
                time.sleep(3.0)

        # Despedida
        if self._running:
            self._say_sync("Eso fue todo por los chistes de hoy. ¡Hasta la próxima!")
            print("="*55)
            print("          FIN DE LA RONDA DE CHISTES")
            print("="*55 + "\n")
            
        self.get_logger().info('Ronda de chistes finalizada.')

def main(args=None):
    rclpy.init(args=args)
    node = ChistesNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()