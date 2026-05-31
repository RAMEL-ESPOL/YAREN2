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


class BodyPointsDetectorNode(LifecycleNode):

    def __init__(self):
        super().__init__('body_points_detector_node')
        self.bridge = CvBridge()
        # Variables de estado — sin inicializar recursos pesados aquí
        self.model        = None
        self.subscription = None
        self.publisher    = None
        self._lock        = threading.Lock()
        self._latest_frame  = None
        self._latest_header = None
        self._frame_count   = 0
        self._INFER_EVERY   = 2
        self._active        = False
        self._infer_thread  = None
        self.INFER_SIZE     = 320

    # ──────────────────────────────────────────────
    # Lifecycle callbacks
    # ──────────────────────────────────────────────

    def on_configure(self, state):
        """Carga el modelo YOLO una sola vez al arrancar el robot."""
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
        """Modo activo: suscribe cámara y arranca thread de inferencia."""
        self.get_logger().info('BodyPointsDetector ACTIVO')
        self._active = True
        self._frame_count  = 0
        self._latest_frame = None
        self.subscription = self.create_subscription(
            Image, '/csi_camera/image_raw', self.image_callback, 10)
        self._infer_thread = threading.Thread(target=self._infer_loop, daemon=True)
        self._infer_thread.start()
        return super().on_activate(state)

    def on_deactivate(self, state):
        """Pausa sin descargar el modelo de RAM."""
        self.get_logger().info('BodyPointsDetector en PAUSA')
        self._active = False
        if self._infer_thread is not None:
            self._infer_thread.join(timeout=2.0)
            self._infer_thread = None
        if self.subscription is not None:
            self.destroy_subscription(self.subscription)
            self.subscription = None
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        """Libera el modelo de memoria (solo si se llama cleanup explícito)."""
        self.get_logger().info('BodyPointsDetector cleanup')
        self.model = None
        if self.publisher is not None:
            self.destroy_publisher(self.publisher)
            self.publisher = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self._active = False
        return TransitionCallbackReturn.SUCCESS

    # ──────────────────────────────────────────────
    # Lógica (sin cambios respecto al original)
    # ──────────────────────────────────────────────

    def image_callback(self, msg):
        if not self._active:
            return
        self._frame_count += 1
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        if self._frame_count % self._INFER_EVERY == 0:
            with self._lock:
                self._latest_frame  = frame.copy()
                self._latest_header = msg.header

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
                landmarks_msg = Landmarks()
                landmarks_msg.header = header
                for x, y in pts:
                    point = Point(x=float(x), y=float(y), z=0.0)
                    landmarks_msg.landmarks.append(point)
                self.publisher.publish(landmarks_msg)


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