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
from ament_index_python.packages import get_package_share_directory

EMOTIONS_EN = ["Angry", "Disgust", "Fear", "Happy", "Sad", "Surprise", "Neutral"]
EMOTIONS_ES = ["Enojado", "Disgusto", "Miedo", "Feliz", "Triste", "Sorpresa", "Neutral"]


class EmotionDetectionNode(LifecycleNode):

    def __init__(self):
        super().__init__('detector')
        # Solo variables — sin cargar nada pesado
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

    # ──────────────────────────────────────────────
    # Lifecycle callbacks
    # ──────────────────────────────────────────────

    def on_configure(self, state):
        """Carga TF + MediaPipe una sola vez al arrancar el robot."""
        self.get_logger().info('Cargando modelo TF y MediaPipe...')
        try:
            pkg_path   = get_package_share_directory('yaren_emotions')
            model_path = os.path.join(pkg_path, 'models', 'model_mbn_1.h5')
            self.model = tf.keras.models.load_model(model_path)
            # Warmup para que la primera inferencia real no sea lenta
            dummy = np.zeros((1, 48, 48, 3), dtype=np.float32)
            self.model(dummy, training=False)

            mp_face_mesh   = mp.solutions.face_mesh
            self.face_mesh = mp_face_mesh.FaceMesh(
                max_num_faces=1, refine_landmarks=False,
                min_detection_confidence=0.5, min_tracking_confidence=0.5)

            self.bridge    = CvBridge()
            self.publisher = self.create_publisher(Int16, '/emotion', 10)

            # Idioma: escuchar siempre aunque el nodo esté en pausa
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.lang_subscription = self.create_subscription(
                Bool, '/yaren/is_english', self.language_callback, qos)

            self.get_logger().info('Modelo TF + MediaPipe listos ✓')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Error en configure: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        """Inicia suscripción de cámara e hilo de inferencia."""
        self.get_logger().info('EmotionDetector ACTIVO')
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.setWindowProperty(self.window_name, cv2.WND_PROP_TOPMOST, 1)
        cv2.setMouseCallback(self.window_name, self.on_mouse_click)
        
        # ──────────────────────────────────────────────
        # Truco de xdotool (Idéntico a tu C++)
        # ──────────────────────────────────────────────
        def force_focus():
            time.sleep(0.3)  # Esperar 300ms a que la ventana se cree
            cmd = f"xdotool search --sync --name '{self.window_name}' windowactivate --sync windowraise 2>/dev/null"
            os.system(cmd)
            
        # Lanzar el hilo como daemon (equivalente al .detach() en C++)
        threading.Thread(target=force_focus, daemon=True).start()
        # ──────────────────────────────────────────────

        self._active      = True
        self._frame_count = 0
        self._latest_frame = None
        self.subscription = self.create_subscription(
            Image, '/csi_camera/image_raw', self.image_callback, 10)
        self._infer_thread = threading.Thread(
            target=self._infer_loop, daemon=True)
        self._infer_thread.start()
        return super().on_activate(state)

    def on_deactivate(self, state):
        """Pausa — modelo y MediaPipe siguen en RAM."""
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

    # ──────────────────────────────────────────────
    # Callbacks (lógica idéntica al original)
    # ──────────────────────────────────────────────

    def language_callback(self, msg):
        self.is_english = msg.data
        self.get_logger().info(
            f"Idioma: {'English' if self.is_english else 'Español'}")

    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.get_logger().info('Clic detectado. Cerrando modo emociones...')
            
            # 1. Avisar al sistema central (face_screen) que se vuelve al menú
            idle_pub = self.create_publisher(String, '/yaren_mode', 1)
            idle_pub.publish(String(data='idle'))
            
            # 2. Destruir el publicador temporal para liberar memoria
            self.destroy_publisher(idle_pub)
            
            # Nota: No se requiere cv2.destroyAllWindows() ni rclpy.shutdown() aquí.
            # face_screen interceptará 'idle' y llamará a on_deactivate() automáticamente.

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
        
        # Extraer frame para que el hilo de inferencia lo procese
        if self._frame_count % self._INFER_EVERY == 0:
            with self._lock:
                self._latest_frame = frame.copy()
                
        vis = frame.copy()
        
        # Recuperar resultados de la inferencia desde el lock
        with self._lock:
            label = self._result_label
            box   = self._result_box
            
        # Dibujar bounding box y etiqueta si el modelo detectó un rostro
        if box is not None:
            x_min, y_min, x_max, y_max = box
            cv2.rectangle(vis, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            cv2.putText(vis, label, (x_min, max(y_min - 10, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
                        
        vis = cv2.resize(vis, (800, 480))
        
        # Mostrar ventana de OpenCV únicamente si el nodo sigue en estado activo
        if self._active and rclpy.ok():
            cv2.imshow(self.window_name, vis)
            cv2.waitKey(1)
            
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
                    preds = self.model(roi, training=False)
                    idx   = int(np.argmax(preds))
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
