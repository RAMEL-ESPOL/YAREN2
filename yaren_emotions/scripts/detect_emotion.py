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

# ─────────────────────────────────────────────────────────────────────────────
#  Configuración de Emojis PNG
# ─────────────────────────────────────────────────────────────────────────────
# Mapea las emociones a los nombres de tus archivos PNG con fondo transparente.
# Los archivos deben estar en la carpeta 'emojis' de tu paquete.
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

# Color de acento por emoción (BGR)
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

# ─────────────────────────────────────────────────────────────────────────────
#  Partícula de Imagen PNG
# ─────────────────────────────────────────────────────────────────────────────

class ImageParticle:
    """Una partícula que renderiza una imagen PNG con canal Alpha (transparencia)."""
    def __init__(self, W: int, H: int, img_array: np.ndarray):
        self.W = W
        self.H = H
        self.base_img = img_array  # Imagen original cargada (BGRA)
        self.reset_random()

    def reset_random(self):
        # Margen ajustado para que no aparezcan pegadas al borde
        self.x = random.randint(20, self.W - 40)
        self.y = random.randint(-150, -30)          
        self.vy = random.uniform(2.5, 6.0)          
        self.vx_amp = random.uniform(0.3, 1.2)      
        self.phase = random.uniform(0, 2 * math.pi) 
        self.freq = random.uniform(0.04, 0.10)      

        # ── AJUSTE DE TAMAÑO FIJO ──
        # Aquí forzamos el tamaño a ~40 píxeles. 
        # Usamos un random muy leve (35 a 45) para que la lluvia se vea un poco más natural, 
        # pero puedes poner un número fijo como `target_size = 40` si las quieres exactamente idénticas.
        target_size = random.randint(35, 45)
        
        self.img = cv2.resize(self.base_img, (target_size, target_size), interpolation=cv2.INTER_AREA)
        self.t = 0.0

    def update(self):
        self.y += self.vy
        self.x += self.vx_amp * math.sin(self.phase + self.t * self.freq * 60)
        self.t += 1
        # Reciclar cuando sale por abajo
        if self.y > self.H + 20:
            self.reset_random()

    def draw(self, frame: np.ndarray):
        x, y = int(self.x), int(self.y)
        h, w = self.img.shape[:2]

        # Evitar crashes si la imagen se sale de los límites de la pantalla
        y1, y2 = max(0, y), min(self.H, y + h)
        x1, x2 = max(0, x), min(self.W, x + w)

        img_y1 = max(0, -y)
        img_y2 = h - max(0, (y + h) - self.H)
        img_x1 = max(0, -x)
        img_x2 = w - max(0, (x + w) - self.W)

        # Si está totalmente fuera de la pantalla, no dibujar nada
        if y1 >= y2 or x1 >= x2:
            return

        # Extraer región del PNG y del frame actual
        emoji_crop = self.img[img_y1:img_y2, img_x1:img_x2]
        frame_crop = frame[y1:y2, x1:x2]

        # Comprobar que el recorte del emoji tenga 4 canales (BGR + Alpha)
        if emoji_crop.shape[2] == 4:
            alpha_mask = emoji_crop[:, :, 3] / 255.0
            alpha_inv = 1.0 - alpha_mask

            # Mezclar colores usando la máscara de transparencia
            for c in range(3):  # B, G, R
                frame_crop[:, :, c] = (alpha_mask * emoji_crop[:, :, c] + alpha_inv * frame_crop[:, :, c])

            frame[y1:y2, x1:x2] = frame_crop

# ─────────────────────────────────────────────────────────────────────────────
#  Nodo principal
# ─────────────────────────────────────────────────────────────────────────────

class EmotionDetectionNode(LifecycleNode):

    # Número de partículas simultáneas en pantalla
    PARTICLE_COUNT = 40

    def __init__(self):
        super().__init__('detector')
        self.is_english      = False
        self.model           = None
        self.face_mesh       = None
        self.bridge          = None
        self.publisher       = None
        self.subscription    = None
        self.lang_subscription = None
        self._lock           = threading.Lock()
        self._latest_frame   = None
        self._result_label   = "..."
        self._result_box     = None
        self._frame_count    = 0
        self._INFER_EVERY    = 3
        self._active         = False
        self._infer_thread   = None
        self.window_name     = "YAREN2 - Emotion Detector"

        # Estado de lluvia de emojis
        self._particles: list[ImageParticle] = []
        self._current_emotion = ""
        self._rain_lock = threading.Lock()
        
        # Diccionario para almacenar los PNGs cargados en memoria RAM
        self.loaded_emojis = {}

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def on_configure(self, state):
        self.get_logger().info('Cargando modelo TF, MediaPipe y emojis PNG...')
        try:
            pkg_path   = get_package_share_directory('yaren_emotions')
            
            # 1. Cargar modelo IA
            model_path = os.path.join(pkg_path, 'models', 'model_mbn_1.h5')
            self.model = tf.keras.models.load_model(model_path)
            dummy = np.zeros((1, 48, 48, 3), dtype=np.float32)
            self.model(dummy, training=False)

            # 2. Cargar imágenes PNG en memoria
            emojis_dir = os.path.join(pkg_path, 'emojis')
            for emotion, files in EMOTION_FILES.items():
                self.loaded_emojis[emotion] = []
                for file in files:
                    img_path = os.path.join(emojis_dir, file)
                    # IMREAD_UNCHANGED preserva la transparencia (canal Alpha)
                    img = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
                    if img is not None:
                        # Verificar que realmente tenga 4 canales (BGRA)
                        if img.shape[2] == 4:
                            self.loaded_emojis[emotion].append(img)
                        else:
                            self.get_logger().warning(f"La imagen {file} no tiene fondo transparente (Alpha).")
                    else:
                        self.get_logger().warning(f"No se encontró el emoji: {img_path}")

            # 3. Iniciar MediaPipe y ROS
            mp_face_mesh   = mp.solutions.face_mesh
            self.face_mesh = mp_face_mesh.FaceMesh(
                max_num_faces=1, refine_landmarks=False,
                min_detection_confidence=0.5, min_tracking_confidence=0.5)

            self.bridge    = CvBridge()
            self.publisher = self.create_publisher(Int16, '/emotion', 10)

            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.lang_subscription = self.create_subscription(
                Bool, '/yaren/is_english', self.language_callback, qos)

            self.get_logger().info('Modelo TF + MediaPipe + Emojis listos ✓')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Error en configure: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        self.get_logger().info('EmotionDetector ACTIVO')
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.setWindowProperty(self.window_name, cv2.WND_PROP_TOPMOST, 1)
        cv2.setMouseCallback(self.window_name, self.on_mouse_click)

        def force_focus():
            time.sleep(0.3)
            cmd = f"xdotool search --sync --name '{self.window_name}' windowactivate --sync windowraise 2>/dev/null"
            os.system(cmd)
        threading.Thread(target=force_focus, daemon=True).start()

        self._active      = True
        self._frame_count = 0
        self._latest_frame = None
        self._particles   = []
        self._current_emotion = ""

        self.subscription = self.create_subscription(
            Image, '/csi_camera/image_raw', self.image_callback, 10)
        self._infer_thread = threading.Thread(target=self._infer_loop, daemon=True)
        self._infer_thread.start()
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
        cv2.destroyAllWindows()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self._active = False
        cv2.destroyAllWindows()
        return TransitionCallbackReturn.SUCCESS

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def language_callback(self, msg):
        self.is_english = msg.data
        self.get_logger().info(f"Idioma: {'English' if self.is_english else 'Español'}")

    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.get_logger().info('Clic detectado. Cerrando modo emociones...')
            idle_pub = self.create_publisher(String, '/yaren_mode', 1)
            idle_pub.publish(String(data='idle'))
            self.destroy_publisher(idle_pub)

    # ── Lluvia de emojis ──────────────────────────────────────────────────────

    def _rebuild_particles(self, emotion: str, W: int, H: int):
        """Recrea las partículas PNG cuando cambia la emoción detectada."""
        images_list = self.loaded_emojis.get(emotion, [])
        new_particles = []
        
        if images_list:
            for _ in range(self.PARTICLE_COUNT):
                base_img = random.choice(images_list)
                p = ImageParticle(W, H, base_img)
                # Distribuir inicialmente en distintas alturas
                p.y = random.randint(-H, H)
                new_particles.append(p)
                
        with self._rain_lock:
            self._particles = new_particles
            self._current_emotion = emotion

    def _draw_emoji_rain(self, frame: np.ndarray):
        """Actualiza y dibuja todas las partículas sobre el frame."""
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

        # Recuperar resultados
        with self._lock:
            label = self._result_label
            box   = self._result_box

        # ── Bounding box y etiqueta ────────────────────────────────────────
        if box is not None:
            x_min, y_min, x_max, y_max = box
            accent = EMOTION_COLORS.get(label, (0, 255, 0))

            # Marco con esquinas decorativas
            corner_len = 20
            thick_box  = 2
            cv2.rectangle(vis, (x_min, y_min), (x_max, y_max), accent, thick_box)
            # Esquinas más gruesas
            for px, py in [(x_min, y_min), (x_max, y_min),
                           (x_min, y_max), (x_max, y_max)]:
                dx = corner_len if px == x_min else -corner_len
                dy = corner_len if py == y_min else -corner_len
                cv2.line(vis, (px, py), (px + dx, py), accent, 4)
                cv2.line(vis, (px, py), (px, py + dy), accent, 4)

            # Etiqueta con fondo semitransparente
            label_display = label
            (tw, th), _ = cv2.getTextSize(
                label_display, cv2.FONT_HERSHEY_DUPLEX, 1.1, 2)
            lx, ly = x_min, max(y_min - 12, 30)
            ov = vis.copy()
            cv2.rectangle(ov, (lx - 4, ly - th - 8),
                          (lx + tw + 8, ly + 6), accent, cv2.FILLED)
            cv2.addWeighted(ov, 0.55, vis, 0.45, 0, vis)
            cv2.putText(vis, label_display, (lx, ly),
                        cv2.FONT_HERSHEY_DUPLEX, 1.1,
                        (255, 255, 255), 2, cv2.LINE_AA)

        # ── Reconstruir partículas si cambió la emoción ────────────────────
        if label != "..." and label != self._current_emotion:
            self._rebuild_particles(label, vis.shape[1], vis.shape[0])

        # ── Dibujar lluvia de emojis ───────────────────────────────────────
        if self._current_emotion:
            self._draw_emoji_rain(vis)

        # ── Nombre de emoción grande en la parte inferior ─────────────────
        if label and label != "...":
            accent = EMOTION_COLORS.get(label, (150, 150, 150))
            big_text = label.upper()
            (bw, bh), _ = cv2.getTextSize(
                big_text, cv2.FONT_HERSHEY_DUPLEX, 1.6, 3)
            bx = (vis.shape[1] - bw) // 2
            by = vis.shape[0] - 18
            # Sombra
            cv2.putText(vis, big_text, (bx + 2, by + 2),
                        cv2.FONT_HERSHEY_DUPLEX, 1.6,
                        (0, 0, 0), 4, cv2.LINE_AA)
            # Texto con color
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
                    frame = self._latest_frame
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
                    x_min = max(0, int(min(x_coords)) - expand)
                    y_min = max(0, int(min(y_coords)) - expand)
                    x_max = min(w, int(max(x_coords)) + expand)
                    y_max = min(h, int(max(y_coords)) + expand)
                    fc = rgb_frame[y_min:y_max, x_min:x_max]
                    if fc.size == 0:
                        continue
                    roi = cv2.resize(fc, (48, 48)).astype(np.float32) / 255.0
                    roi = np.expand_dims(roi, axis=0)
                    if not self._active:
                        break
                    
                    # 1. Obtener predicciones
                    preds = self.model(roi, training=False)
                    preds_array = np.array(preds[0])

                    
                    # 2. HACK MATEMÁTICO: Multiplicar probabilidad de "Triste" (índice 4)
                    preds_array[4] *= 4.0
                    emotion_list = EMOTIONS_EN if self.is_english else EMOTIONS_ES
                    probs_str = " | ".join(f"{emotion_list[i]}:{preds_array[i]:.2f}" for i in range(len(emotion_list)))
                    self.get_logger().info(f'Probs: {probs_str}')

                    idx = int(np.argmax(preds_array))
                    
                    # 3. Elegir la emoción ganadora
                    idx = int(np.argmax(preds_array))
                    
                    emotion_list = EMOTIONS_EN if self.is_english else EMOTIONS_ES
                    with self._lock:
                        self._result_label = emotion_list[idx]
                        self._result_box   = (x_min, y_min, x_max, y_max)
                    self.publisher.publish(Int16(data=idx))
            else:
                with self._lock:
                    self._result_box = None


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