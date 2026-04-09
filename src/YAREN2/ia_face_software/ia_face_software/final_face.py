#!/usr/bin/env python3

import pygame
from itertools import cycle
import subprocess
import sys
import os
import json
import threading
import time
from moviepy.editor import VideoFileClip
from queue import Queue
import numpy as np
from ament_index_python.packages import get_package_share_directory
# ─── ROS2 Jazzy ───────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# ──────────────────────────────────────────────────────────────────────────────

# ELIMINAR esta línea que era exclusiva de ROS1/Noetic:
# sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')  ← BORRAR

from ia_face_software.audio_processing.utils import read_file
from ia_face_software.audio_processing.assistant import Assistant
from ia_face_software.video_audio_response.video_generation import total_video_generation

try:
    current_dir = get_package_share_directory('ia_face_software')
except Exception:
    current_dir = os.path.dirname(os.path.abspath(__file__))

# ── Rutas de imágenes (sin cambios) ──────────────────────────────────────────
carpetaImgs = os.path.join(current_dir, 'faces') + "/"
IMAGEN_DEFAULT      = carpetaImgs + "default.png"
IMAGEN_PARPADEO     = carpetaImgs + "blink2_normal.png"
IMAGEN_FELIZ        = carpetaImgs + "happy.png"
IMAGEN_MEH          = carpetaImgs + "meh.png"
IMAGEN_MUERTO       = carpetaImgs + "game_over.png"
IMAGEN_DINERO       = carpetaImgs + "money.png"
IMAGEN_PENSANDO     = carpetaImgs + "thinking.png"
IMAGEN_BOCAABIERTA  = carpetaImgs + "open_mouth.png"
IMAGEN_BOCACERRADA  = carpetaImgs + "close_mouth.png"
IMAGEN_LISTO        = carpetaImgs + "ready.png"

SECUENCIA_IMAGENES = [IMAGEN_PARPADEO, IMAGEN_DEFAULT]
TIEMPOS_IMAGENES   = [0.1, 2.0]

window_position  = [0, 0]
pasosHorizontal  = 5
pasosVertical    = 5


def seleccionar_pantalla():
    """Sin cambios — lógica pygame pura."""
    pygame.init()
    pantallas = pygame.display.get_desktop_sizes()
    print("Detectando pantallas disponibles:")
    for i, res in enumerate(pantallas):
        print(f"Pantalla {i+1}: {res[0]}x{res[1]}")

    seleccionada    = 0
    seleccion_hecha = False
    fuente          = pygame.font.SysFont("Arial", 30)
    window_temp     = pygame.display.set_mode((500, 300))
    pygame.display.set_caption("Seleccionar Pantalla")

    while not seleccion_hecha:
        window_temp.fill((0, 0, 0))
        texto = fuente.render(
            f"Seleccione la pantalla (1-{len(pantallas)}): {seleccionada+1}",
            True, (255, 255, 255))
        window_temp.blit(texto, (50, 100))
        pygame.display.flip()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RIGHT:
                    seleccionada = (seleccionada + 1) % len(pantallas)
                elif event.key == pygame.K_LEFT:
                    seleccionada = (seleccionada - 1) % len(pantallas)
                elif event.key == pygame.K_RETURN:
                    seleccion_hecha = True

    x_global = sum(pantallas[i][0] for i in range(seleccionada))
    pygame.display.quit()
    return seleccionada, pantallas[seleccionada], (x_global, 0)


def centrar_ventana(pantalla_res, position):
    global window_position
    x_global, y_global = position
    window_width, window_height = 800, 480
    screen_width, screen_height = pantalla_res
    window_position = [
        (screen_width  - window_width)  // 2 + x_global,
        (screen_height - window_height) // 2 + y_global,
    ]


def moverVentana():
    keys = pygame.key.get_pressed()
    if keys[pygame.K_a]: window_position[0] -= pasosHorizontal
    if keys[pygame.K_d]: window_position[0] += pasosHorizontal
    if keys[pygame.K_w]: window_position[1] -= pasosVertical
    if keys[pygame.K_s]: window_position[1] += pasosVertical


# ── Clase principal — ahora hereda de Node ───────────────────────────────────
class FinalFaceNode(Node):           # ← antes: class final_face

    def __init__(self):
        # ── ROS2: inicializar el nodo ──────────────────────────────────────
        super().__init__('final_face')

        self.pub_commands = self.create_publisher(
            String, '/movement_commands', 1)
        # ──────────────────────────────────────────────────────────────────

        # ── RUTAS DE ARCHIVOS (Solución Share Directory Definitiva) ────────
        import os
        from ament_index_python.packages import get_package_share_directory
        
        try:
            package_share_path = get_package_share_directory('ia_face_software')
        except Exception as e:
            self.get_logger().error(f"No se encontró el directorio share: {e}")
            package_share_path = os.path.dirname(os.path.abspath(__file__))

        config_path = os.path.join(package_share_path, 'config', 'config')
        responses_path = os.path.join(package_share_path, 'basics_responses.json')
        faces_dir = os.path.join(package_share_path, 'faces')
        # ──────────────────────────────────────────────────────────────────

        self.number   = "0"
        self.respuesta = ""

        # Cargar configuración YAML
        CONFIG_PARAMS = read_file(config_path, "yaml")
        self.comands  = CONFIG_PARAMS["comands"]

        # Cargar respuestas básicas JSON
        try:
            with open(responses_path, "r", encoding='utf-8') as f:
                basics_responses = json.load(f)
            self.basics_responses = {k.lower(): v for k, v in basics_responses.items()}
        except (FileNotFoundError, json.JSONDecodeError) as e:
            self.get_logger().error(f"Error al cargar respuestas básicas: {e}")
            self.basics_responses = {}

        # Inicializar el Asistente de IA
        self.assistant = Assistant(
            CONFIG_PARAMS["stt"]["model_size"],
            CONFIG_PARAMS["stt"]["recording_time"],
            CONFIG_PARAMS["stt"]["silence_break"],
            CONFIG_PARAMS["stt"]["sensibility"],
            CONFIG_PARAMS["assistant"]["wake_word"],
            CONFIG_PARAMS["comands"],
            CONFIG_PARAMS["prompt"],
            self.basics_responses,
        )

        # ── Configuración de Pantalla y Pygame ────────────────────────────
        pygame.init()
        # Nota: Unificamos el nombre de la variable de posición a 'window_position'
        seleccionada, res_pantalla, window_position = seleccionar_pantalla()
        centrar_ventana(res_pantalla, window_position)

        self.window_width, self.window_height = 800, 480
        os.environ['SDL_VIDEO_WINDOW_POS'] = f"{window_position[0]},{window_position[1]}"
        self.window = pygame.display.set_mode(
            (self.window_width, self.window_height), pygame.NOFRAME)
        pygame.display.set_caption("YAREN Face")

        # ── Control de Animación y Video (Rutas Limpiadas a la Fuerza) ────
        # Le arrancamos cualquier ruta vieja usando os.path.basename
        # y le pegamos la ruta 100% correcta de la carpeta 'faces' de ROS2.
        SECUENCIA_IMAGENES_FIJAS = [
            os.path.join(faces_dir, os.path.basename(img)) for img in SECUENCIA_IMAGENES
        ]

        self.expressions = cycle(SECUENCIA_IMAGENES_FIJAS)
        self.times        = cycle(TIEMPOS_IMAGENES)
        self.current_image     = next(self.expressions)
        self.current_time      = next(self.times)
        self.last_switch_time  = pygame.time.get_ticks()
        
        self.video_queue       = Queue()
        self.video_ready       = threading.Event()
        self.detect_commands_running = threading.Event()

    def detect_commands(self):
        while rclpy.ok():                           # ← reemplaza rospy.is_shutdown()
            self.detect_commands_running.wait()
            pose_num, respuesta = self.assistant.listen_movement_comand()
            self.number = str(pose_num + 1)

            msg = String()
            msg.data = self.number
            self.pub_commands.publish(msg)          # ← en ROS2 se publica un objeto msg
            self.get_logger().info(f"Publicado: {self.number}")  # ← rospy.loginfo

            if pose_num != -1:
                self.get_logger().info(self.comands[pose_num])
                if not self.video_ready.is_set():
                    threading.Thread(
                        target=self.generate_video, args=(respuesta,)).start()

    # ── Video ─────────────────────────────────────────────────────────────────
    def play_video(self):
        if not self.video_ready.is_set():
            return
        video_clip = self.video_queue.get()
        self.video_ready.clear()

        if video_clip.audio:
            video_clip.audio.write_audiofile("/tmp/temp_audio.mp3", fps=44100)
            pygame.mixer.init()
            pygame.mixer.music.load("/tmp/temp_audio.mp3")
            pygame.mixer.music.play()

        for frame in video_clip.iter_frames(fps=24, with_times=False):
            if not pygame.mixer.music.get_busy():
                break
            frame_surface = pygame.surfarray.make_surface(np.swapaxes(frame, 0, 1))
            frame_surface = pygame.transform.scale(
                frame_surface, (self.window_width, self.window_height))
            self.window.blit(frame_surface, (0, 0))
            pygame.display.flip()

        pygame.mixer.music.stop()
        video_clip.close()
        self.detect_commands_running.set()

    def generate_video(self, respuesta):
        try:
            video_clip = total_video_generation(respuesta)
            self.video_queue.put(video_clip)
            self.video_ready.set()
            self.detect_commands_running.clear()
        except Exception as e:
            self.get_logger().error(f"Error generando video: {e}")  # ← rospy.logerr

    def show_special_image(self, image_path, sleep_time=5):
        image = pygame.image.load(image_path)
        self.window.fill((0, 0, 0))
        self.window.blit(image, (0, 0))
        pygame.display.flip()
        time.sleep(sleep_time)

    # ── Bucle principal ───────────────────────────────────────────────────────
    def run(self):                              # ← antes se llamaba main()
        clock   = pygame.time.Clock()
        running = True

        command_thread = threading.Thread(target=self.detect_commands, daemon=True)
        command_thread.start()
        self.detect_commands_running.set()

        while running and rclpy.ok():           # ← reemplaza rospy.is_shutdown()
            try:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                        running = False

                current_ticks = pygame.time.get_ticks()

                if self.video_ready.is_set():
                    self.play_video()

                if self.number == "1":
                    self.show_special_image(IMAGEN_DINERO)
                elif self.number == "2":
                    self.show_special_image(IMAGEN_FELIZ)
                elif self.number == "3":
                    self.show_special_image(IMAGEN_BOCAABIERTA)
                elif self.number == "4":
                    self.show_special_image(IMAGEN_BOCACERRADA)
                elif self.number == "5":
                    self.show_special_image(IMAGEN_LISTO)
                elif self.number in ("6", "7"):
                    self.show_special_image(IMAGEN_FELIZ)

                elif (current_ticks - self.last_switch_time) / 1000.0 >= self.current_time:
                    self.current_image    = next(self.expressions)
                    self.current_time     = next(self.times)
                    self.last_switch_time = current_ticks

                image = pygame.image.load(self.current_image)
                self.window.fill((0, 0, 0))
                self.window.blit(image, (0, 0))
                pygame.display.flip()

                moverVentana()
                subprocess.run([
                    'wmctrl', '-r', ':ACTIVE:', '-e',
                    f'0,{window_position[0]},{window_position[1]},-1,-1'
                ])
                clock.tick(60)

            except Exception as e:
                self.get_logger().error(f"Error en el bucle principal: {e}")
                running = False

        pygame.quit()
        self.get_logger().info("Programa finalizado correctamente.")


# ── Entrypoint ROS2 ───────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)               # ← siempre antes de crear el nodo
    node = FinalFaceNode()
    try:
        node.run()                      # bucle pygame (bloqueante)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()