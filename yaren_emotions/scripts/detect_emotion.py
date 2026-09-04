#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from sensor_msgs.msg import Image
from std_msgs.msg import Int16, Bool, String
from rclpy.qos import QoSProfile, DurabilityPolicy
from cv_bridge import CvBridge
import numpy as np
import tensorflow as tf
import cv2
import mediapipe as mp
import os
import threading
import time
import random
import math
from ament_index_python.packages import get_package_share_directory

EMOTIONS_EN = ["Angry", "Disgust", "Fear", "Happy", "Sad", "Surprise", "Neutral"]
EMOTIONS_ES = ["Enojado", "Disgusto", "Miedo", "Feliz", "Triste", "Sorpresa", "Neutral"]

EMOTION_FILES = {
    "Angry":    ["enojado.png"],
    "Enojado":  ["enojado.png"],
    "Disgust":  ["disgusto.png"],
    "Disgusto": ["disgusto.png"],
    "Fear":     ["miedo.png"],
    "Miedo":    ["miedo.png"],
    "Happy":    ["feliz.png"],
    "Feliz":    ["feliz.png"],
    "Sad":      ["triste.png"],
    "Triste":   ["triste.png"],
    "Surprise": ["sorpresa.png"],
    "Sorpresa": ["sorpresa.png"],
    "Neutral":  ["neutral.png"],
}

EMOTION_COLORS = {
    "Angry":    (50,  50,  220),
    "Enojado":  (50,  50,  220),
    "Disgust":  (50,  180, 50),
    "Disgusto": (50,  180, 50),
    "Fear":     (180, 50,  180),
    "Miedo":    (180, 50,  180),
    "Happy":    (50,  220, 220),
    "Feliz":    (50,  220, 220),
    "Sad":      (220, 100, 50),
    "Triste":   (220, 100, 50),
    "Surprise": (50,  200, 255),
    "Sorpresa": (50,  200, 255),
    "Neutral":  (150, 150, 150),
}

# =============================================================================
#  PLAYLISTS — alternando ES / EN
# =============================================================================
HOME_DIR  = os.path.expanduser("~")
MUSIC_DIR = os.path.join(HOME_DIR, "robotis_ws", "src", "YAREN2",
                         "yaren_radio", "audios")

# Pon aquí los nombres EXACTOS de tus archivos mp3
PLAYLIST_EN = [
    "CantStopTheFeeling.mp3",   # Justin Timberlake
    "JustTheWayYouAre.mp3",     # Milky
    "GetLucky.mp3",             # Daft Punk
    "YourLove.mp3",             # The Outfield
    "SunFlower.mp3",             # Post Malone
]

PLAYLIST_ES = [
    "Picky.mp3",                # Joey Montana
    "TuCarcel.mp3",             # Enanitos Verdes
    "LaBicicleta.mp3",          # Carlos Vives & Shakira
    "LaGozadera.mp3",           # Gente de Zona ft. Marc Anthony
    "MiGente.mp3",              # J Balvin
]


# =============================================================================
#  MUSIC MANAGER
# =============================================================================
class MusicManager:
    def __init__(self, logger=None):
        self._available    = False
        self._logger       = logger
        self._en_index     = 0
        self._es_index     = 0
        self._turn_en      = True   # True = turno inglés, False = turno español
        self._running      = False
        self._check_thread = None
        self._pygame       = None

        try:
            import pygame
            self._pygame = pygame
            if not pygame.mixer.get_init():
                pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=512)
            pygame.mixer.music.set_volume(0.45)
            self._available = True
            self._log("MusicManager listo.")
        except Exception as e:
            self._log(f"pygame no disponible: {e}")

    def _log(self, msg):
        if self._logger:
            self._logger.info(f"[Music] {msg}")
        else:
            print(f"[Music] {msg}")

    def _get_next_path(self):
        """Devuelve la siguiente canción alternando ES/EN y avanza el índice."""
        if self._turn_en:
            playlist = PLAYLIST_EN
            idx      = self._en_index % len(PLAYLIST_EN)
            self._en_index += 1
        else:
            playlist = PLAYLIST_ES
            idx      = self._es_index % len(PLAYLIST_ES)
            self._es_index += 1

        self._turn_en = not self._turn_en  # alternar para la próxima
        return os.path.join(MUSIC_DIR, playlist[idx])

    def _play_path(self, path):
        if not os.path.isfile(path):
            self._log(f"Archivo no encontrado: {path}")
            return False
        try:
            self._pygame.mixer.music.load(path)
            self._pygame.mixer.music.set_volume(0.45)
            self._pygame.mixer.music.play()
            self._log(f"Reproduciendo: {os.path.basename(path)}")
            return True
        except Exception as e:
            self._log(f"Error reproduciendo {path}: {e}")
            return False

    def _monitor_loop(self):
        """Hilo que detecta cuando termina una canción y pone la siguiente."""
        # Esperar un momento para que empiece a sonar
        time.sleep(1.0)
        while self._running:
            try:
                if self._available and not self._pygame.mixer.music.get_busy():
                    path = self._get_next_path()
                    self._play_path(path)
            except Exception as e:
                self._log(f"Error en monitor: {e}")
            time.sleep(0.5)

    def start(self):
        if not self._available:
            return
        self._running = True
        # Tocar la primera canción
        path = self._get_next_path()
        self._play_path(path)
        # Arrancar hilo monitor
        self._check_thread = threading.Thread(
            target=self._monitor_loop, daemon=True)
        self._check_thread.start()

    def stop(self):
        self._running = False
        if self._check_thread:
            self._check_thread.join(timeout=2.0)
            self._check_thread = None
        if not self._available:
            return
        try:
            if self._pygame.mixer.get_init():
                self._pygame.mixer.music.stop()
        except Exception:
            pass

    def quit(self):
        self.stop()
        if not self._available:
            return
        try:
            self._pygame.mixer.quit()
        except Exception:
            pass


# =============================================================================
#  Partícula de Imagen PNG
# =============================================================================
class ImageParticle:
    def __init__(self, W: int, H: int, img_array: np.ndarray):
        self.W = W
        self.H = H
        self.base_img = img_array
        self.reset_random()

    def reset_random(self):
        self.x       = random.randint(20, self.W - 40)
        self.y       = random.randint(-150, -30)
        self.vy      = random.uniform(2.5, 6.0)
        self.vx_amp  = random.uniform(0.3, 1.2)
        self.phase   = random.uniform(0, 2 * math.pi)
        self.freq    = random.uniform(0.04, 0.10)
        target_size  = random.randint(35, 45)
        self.img     = cv2.resize(self.base_img, (target_size, target_size),
                                  interpolation=cv2.INTER_AREA)
        self.t = 0.0

    def update(self):
        self.y += self.vy
        self.x += self.vx_amp * math.sin(self.phase + self.t * self.freq * 60)
        self.t += 1
        if self.y > self.H + 20:
            self.reset_random()

    def draw(self, frame: np.ndarray):
        x, y = int(self.x), int(self.y)
        h, w = self.img.shape[:2]
        y1, y2 = max(0, y),   min(self.H, y + h)
        x1, x2 = max(0, x),   min(self.W, x + w)
        img_y1  = max(0, -y)
        img_y2  = h - max(0, (y + h) - self.H)
        img_x1  = max(0, -x)
        img_x2  = w - max(0, (x + w) - self.W)
        if y1 >= y2 or x1 >= x2:
            return
        emoji_crop = self.img[img_y1:img_y2, img_x1:img_x2]
        frame_crop = frame[y1:y2, x1:x2]
        if emoji_crop.shape[2] == 4:
            alpha_mask = emoji_crop[:, :, 3] / 255.0
            alpha_inv  = 1.0 - alpha_mask
            for c in range(3):
                frame_crop[:, :, c] = (alpha_mask * emoji_crop[:, :, c] +
                                       alpha_inv  * frame_crop[:, :, c])
            frame[y1:y2, x1:x2] = frame_crop


# =============================================================================
#  Nodo principal
# =============================================================================
class EmotionDetectionNode(LifecycleNode):

    PARTICLE_COUNT = 40

    def __init__(self):
        super().__init__('detector')
        self.is_english        = False
        self.model             = None
        self.face_mesh         = None
        self.bridge            = None
        self.publisher         = None
        self.subscription      = None
        self.lang_subscription = None
        self._lock             = threading.Lock()
        self._latest_frame     = None
        self._result_label     = "..."
        self._result_box       = None
        self._frame_count      = 0
        self._INFER_EVERY      = 3
        self._active           = False
        self._infer_thread     = None
        self.window_name       = "YAREN2 - Emotion Detector"
        self._particles: list[ImageParticle] = []
        self._current_emotion  = ""
        self._rain_lock        = threading.Lock()
        self.loaded_emojis     = {}
        self._music            = None   # MusicManager

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def on_configure(self, state):
        self.get_logger().info('Cargando modelo TF, MediaPipe y emojis PNG...')
        try:
            pkg_path   = get_package_share_directory('yaren_emotions')
            model_path = os.path.join(pkg_path, 'models', 'model_mbn_1.h5')
            self.model = tf.keras.models.load_model(model_path)
            dummy = np.zeros((1, 48, 48, 3), dtype=np.float32)
            self.model(dummy, training=False)

            emojis_dir = os.path.join(pkg_path, 'emojis')
            for emotion, files in EMOTION_FILES.items():
                self.loaded_emojis[emotion] = []
                for file in files:
                    img_path = os.path.join(emojis_dir, file)
                    img = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
                    if img is not None:
                        if img.shape[2] == 4:
                            self.loaded_emojis[emotion].append(img)
                        else:
                            self.get_logger().warning(
                                f"La imagen {file} no tiene Alpha.")
                    else:
                        self.get_logger().warning(
                            f"No se encontró el emoji: {img_path}")

            mp_face_mesh   = mp.solutions.face_mesh
            self.face_mesh = mp_face_mesh.FaceMesh(
                max_num_faces=1, refine_landmarks=False,
                min_detection_confidence=0.5, min_tracking_confidence=0.5)

            self.bridge    = CvBridge()
            self.publisher = self.create_publisher(Int16, '/emotion', 10)

            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.lang_subscription = self.create_subscription(
                Bool, '/yaren/is_english', self.language_callback, qos)

            # Iniciar música
            self._music = MusicManager(logger=self.get_logger())

            self.get_logger().info('Modelo TF + MediaPipe + Emojis + Música listos ✓')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Error en configure: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        self.get_logger().info('EmotionDetector ACTIVO')
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(self.window_name,
                              cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.setWindowProperty(self.window_name, cv2.WND_PROP_TOPMOST, 1)
        cv2.setMouseCallback(self.window_name, self.on_mouse_click)

        def force_focus():
            time.sleep(0.3)
            cmd = (f"xdotool search --sync --name '{self.window_name}' "
                   "windowactivate --sync windowraise 2>/dev/null")
            os.system(cmd)
        threading.Thread(target=force_focus, daemon=True).start()

        self._active       = True
        self._frame_count  = 0
        self._latest_frame = None
        self._particles    = []
        self._current_emotion = ""

        self.subscription = self.create_subscription(
            Image, '/csi_camera/image_raw', self.image_callback, 10)

        self._infer_thread = threading.Thread(
            target=self._infer_loop, daemon=True)
        self._infer_thread.start()

        # Arrancar música al activarse
        if self._music:
            self._music.start()

        return super().on_activate(state)

    def on_deactivate(self, state):
        self.get_logger().info('EmotionDetector en PAUSA')
        self._active = False

        if self._infer_thread is not None:
            self._infer_thread.join(timeout=2.0)
            self._infer_thread = None
        if self.subscription is not None:
            self.destroy_subscription(self.subscription)
            self.subscription = None

        # Parar música al desactivarse
        if self._music:
            self._music.stop()

        cv2.destroyAllWindows()
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self.model     = None
        self.face_mesh = None
        self.loaded_emojis.clear()
        if self.publisher is not None:
            self.destroy_publisher(self.publisher)
            self.publisher = None
        if self.lang_subscription is not None:
            self.destroy_subscription(self.lang_subscription)
            self.lang_subscription = None
        if self._music:
            self._music.quit()
            self._music = None
        cv2.destroyAllWindows()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self._active = False
        if self._music:
            self._music.quit()
            self._music = None
        cv2.destroyAllWindows()
        return TransitionCallbackReturn.SUCCESS

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def language_callback(self, msg):
        self.is_english = msg.data
        self.get_logger().info(
            f"Idioma: {'English' if self.is_english else 'Español'}")

    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.get_logger().info('Clic detectado. Cerrando modo emociones...')
            idle_pub = self.create_publisher(String, '/yaren_mode', 1)
            idle_pub.publish(String(data='idle'))
            self.destroy_publisher(idle_pub)

    # ── Lluvia de emojis ──────────────────────────────────────────────────────

    def _rebuild_particles(self, emotion: str, W: int, H: int):
        images_list  = self.loaded_emojis.get(emotion, [])
        new_particles = []
        if images_list:
            for _ in range(self.PARTICLE_COUNT):
                base_img = random.choice(images_list)
                p = ImageParticle(W, H, base_img)
                p.y = random.randint(-H, H)
                new_particles.append(p)
        with self._rain_lock:
            self._particles       = new_particles
            self._current_emotion = emotion

    def _draw_emoji_rain(self, frame: np.ndarray):
        with self._rain_lock:
            particles = list(self._particles)
        for p in particles:
            p.update()
            p.draw(frame)

    # ── Procesamiento de imagen ───────────────────────────────────────────────

    def image_callback(self, msg):
        if not self._active or not rclpy.ok():
            return

        self._frame_count += 1
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Error cv_bridge: {e}')
            return

        frame = cv2.flip(frame, 0)

        if self._frame_count % self._INFER_EVERY == 0:
            with self._lock:
                self._latest_frame = frame.copy()

        vis = frame.copy()

        with self._lock:
            label = self._result_label
            box   = self._result_box

        if box is not None:
            x_min, y_min, x_max, y_max = box
            accent     = EMOTION_COLORS.get(label, (0, 255, 0))
            corner_len = 20
            cv2.rectangle(vis, (x_min, y_min), (x_max, y_max), accent, 2)
            for px, py in [(x_min, y_min), (x_max, y_min),
                           (x_min, y_max), (x_max, y_max)]:
                dx = corner_len if px == x_min else -corner_len
                dy = corner_len if py == y_min else -corner_len
                cv2.line(vis, (px, py), (px + dx, py), accent, 4)
                cv2.line(vis, (px, py), (px, py + dy), accent, 4)

            (tw, th), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_DUPLEX, 1.1, 2)
            lx, ly = x_min, max(y_min - 12, 30)
            ov = vis.copy()
            cv2.rectangle(ov, (lx - 4, ly - th - 8),
                          (lx + tw + 8, ly + 6), accent, cv2.FILLED)
            cv2.addWeighted(ov, 0.55, vis, 0.45, 0, vis)
            cv2.putText(vis, label, (lx, ly),
                        cv2.FONT_HERSHEY_DUPLEX, 1.1,
                        (255, 255, 255), 2, cv2.LINE_AA)

        if label != "..." and label != self._current_emotion:
            self._rebuild_particles(label, vis.shape[1], vis.shape[0])

        if self._current_emotion:
            self._draw_emoji_rain(vis)

        if label and label != "...":
            accent   = EMOTION_COLORS.get(label, (150, 150, 150))
            big_text = label.upper()
            (bw, bh), _ = cv2.getTextSize(
                big_text, cv2.FONT_HERSHEY_DUPLEX, 1.6, 3)
            bx = (vis.shape[1] - bw) // 2
            by = vis.shape[0] - 18
            cv2.putText(vis, big_text, (bx + 2, by + 2),
                        cv2.FONT_HERSHEY_DUPLEX, 1.6,
                        (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(vis, big_text, (bx, by),
                        cv2.FONT_HERSHEY_DUPLEX, 1.6,
                        accent, 3, cv2.LINE_AA)

        vis = cv2.resize(vis, (800, 480))

        if self._active and rclpy.ok():
            cv2.imshow(self.window_name, vis)
            cv2.waitKey(1)

    # ── Hilo de inferencia ────────────────────────────────────────────────────

    def _infer_loop(self):
        while self._active and rclpy.ok():
            frame = None
            with self._lock:
                if self._latest_frame is not None:
                    frame          = self._latest_frame
                    self._latest_frame = None
            if frame is None:
                time.sleep(0.01)
                continue

            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results   = self.face_mesh.process(rgb_frame)

            if results.multi_face_landmarks:
                h, w = frame.shape[:2]
                for face_landmarks in results.multi_face_landmarks:
                    x_coords = [lm.x * w for lm in face_landmarks.landmark]
                    y_coords = [lm.y * h for lm in face_landmarks.landmark]
                    expand = 40
                    x_min  = max(0, int(min(x_coords)) - expand)
                    y_min  = max(0, int(min(y_coords)) - expand)
                    x_max  = min(w, int(max(x_coords)) + expand)
                    y_max  = min(h, int(max(y_coords)) + expand)
                    fc = rgb_frame[y_min:y_max, x_min:x_max]
                    if fc.size == 0:
                        continue
                    roi = cv2.resize(fc, (48, 48)).astype(np.float32) / 255.0
                    roi = np.expand_dims(roi, axis=0)
                    if not self._active:
                        break

                    preds       = self.model(roi, training=False)
                    preds_array = np.array(preds[0])
                    preds_array[4] *= 4.0  # boost Sad/Triste

                    emotion_list = EMOTIONS_EN if self.is_english else EMOTIONS_ES
                    probs_str = " | ".join(
                        f"{emotion_list[i]}:{preds_array[i]:.2f}"
                        for i in range(len(emotion_list)))
                    self.get_logger().info(f'Probs: {probs_str}')

                    idx = int(np.argmax(preds_array))
                    with self._lock:
                        self._result_label = emotion_list[idx]
                        self._result_box   = (x_min, y_min, x_max, y_max)
                    self.publisher.publish(Int16(data=idx))
            else:
                with self._lock:
                    self._result_box = None


# =============================================================================
#  MAIN
# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = EmotionDetectionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()