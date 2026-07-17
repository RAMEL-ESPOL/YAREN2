#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from yaren_interfaces.msg import Landmarks
from ultralytics import YOLO
import os
import threading
import cv2
import time
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String

WIN = 'Yaren Dice'

class BodyPointsDetectorNode(LifecycleNode):

    def __init__(self):
        super().__init__('body_points_detector_node_visual')
        self.bridge = CvBridge()
        self.model          = None
        self.subscription   = None
        self.publisher      = None
        self._lock          = threading.Lock()
        self._latest_frame  = None
        self._latest_header = None
        self._result_pts    = []   # últimos keypoints para dibujar
        self._frame_count   = 0
        self._INFER_EVERY   = 2
        self._active        = False
        self._show_window   = False # Bandera para controlar la ventana
        self._infer_thread  = None
        self.INFER_SIZE     = 320
        self.mode_publisher = self.create_publisher(String, '/yaren_mode', 10)

    # ──────────────────────────────────────────────
    # Lifecycle callbacks
    # ──────────────────────────────────────────────

    def on_configure(self, state):
        self.get_logger().info('Cargando modelo YOLO...')
        try:
            pkg_share_dir = get_package_share_directory('yaren_dice')
            model_path = os.path.join(pkg_share_dir, 'models', 'yolov8s-pose.pt')
            self.model = YOLO(model_path)
            self.publisher = self.create_publisher(Landmarks, 'pose_landmarks', 10)
            self.get_logger().info('Modelo YOLO listo ✓')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Error en configure: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        self.get_logger().info('BodyPointsDetector ACTIVO')
        self._active = True
        self._show_window = True
        self._frame_count  = 0
        self._latest_frame = None
        self._result_pts   = []

        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(WIN, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        
        # Asignar la función para detectar el clic en la ventana
        cv2.setMouseCallback(WIN, self._mouse_callback)

        self.subscription = self.create_subscription(
            Image, '/csi_camera/image_raw', self.image_callback, 10)
        self._infer_thread = threading.Thread(target=self._infer_loop, daemon=True)
        self._infer_thread.start()
        return super().on_activate(state)

    def on_deactivate(self, state):
        self.get_logger().info('BodyPointsDetector en PAUSA')
        self._active = False
        self._show_window = False
        if self._infer_thread is not None:
            self._infer_thread.join(timeout=2.0)
            self._infer_thread = None
        if self.subscription is not None:
            self.destroy_subscription(self.subscription)
            self.subscription = None
        cv2.destroyAllWindows()
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self.get_logger().info('BodyPointsDetector cleanup')
        self.model = None
        if self.publisher is not None:
            self.destroy_publisher(self.publisher)
            self.publisher = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self._active = False
        self._show_window = False
        cv2.destroyAllWindows()
        return TransitionCallbackReturn.SUCCESS

    # ──────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────

    def _mouse_callback(self, event, x, y, flags, param):
        """Detecta clics en la ventana de OpenCV para cerrarla y detener el juego."""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.get_logger().info('Clic detectado. Apagando Simon Dice y cerrando ayuda...')
            
            # 1. Enviar comando de parada
            msg = String()
            msg.data = 'idle'
            self.mode_publisher.publish(msg)
            
            # 2. Cerrar la ventana
            self._show_window = False
            cv2.destroyAllWindows()

    def image_callback(self, msg):
        if not self._active:
            return
        self._frame_count += 1
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        # Guardar frame para inferencia
        if self._frame_count % self._INFER_EVERY == 0:
            with self._lock:
                self._latest_frame  = frame.copy()
                self._latest_header = msg.header

        # Si la ventana sigue activa, dibujamos los puntos y la mostramos
        if self._show_window:
            with self._lock:
                pts = list(self._result_pts)

            vis = cv2.resize(frame, (800, 480))
            for x, y in pts:
                cx, cy = int(x), int(y)
                if (cx, cy) != (0, 0):
                    cv2.circle(vis, (cx, cy), 5, (0, 255, 0), -1, cv2.LINE_AA)

            try:
                cv2.imshow(WIN, vis)
                cv2.waitKey(1)
            except cv2.error:
                # Evita un crash si la ventana fue destruida asíncronamente
                pass

    def _infer_loop(self):
        while self._active and rclpy.ok():
            frame = header = None
            with self._lock:
                if self._latest_frame is not None:
                    frame  = self._latest_frame
                    header = self._latest_header
                    self._latest_frame = None
            if frame is None:
                time.sleep(0.005)
                continue

            results   = self.model(frame, imgsz=self.INFER_SIZE, verbose=False)
            keypoints = results[0].keypoints

            if keypoints is not None and len(keypoints.xy) > 0:
                pts = keypoints.xy[0].cpu().numpy()

                # Escalar puntos al tamaño de visualización (800x480)
                h, w = frame.shape[:2]
                sx, sy = 800 / w, 480 / h
                with self._lock:
                    self._result_pts = [(float(x) * sx, float(y) * sy) for x, y in pts]

                # Publicar landmarks en coordenadas originales
                landmarks_msg = Landmarks()
                landmarks_msg.header = header
                for x, y in pts:
                    landmarks_msg.landmarks.append(Point(x=float(x), y=float(y), z=0.0))
                self.publisher.publish(landmarks_msg)
            else:
                with self._lock:
                    self._result_pts = []

def main(args=None):
    rclpy.init(args=args)
    node = BodyPointsDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()