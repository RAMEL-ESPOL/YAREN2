#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from yaren_interfaces.msg import Landmarks
from ultralytics import YOLO
import os
import threading
import cv2
from ament_index_python.packages import get_package_share_directory

class BodyPointsDetectorNode(Node):
    def __init__(self):
        super().__init__('body_points_detector_node')
        self.bridge = CvBridge()
        self.window_name = "Camara Yaren Dice"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        pkg_share_dir = get_package_share_directory('yaren_dice')
        model_path = os.path.join(pkg_share_dir, 'models', 'yolov8s-pose.pt')

        # Reducir resolución de inferencia (más rápido, suficiente para pose)
        self.model = YOLO(model_path)
        self.INFER_SIZE = 320  # default es 640, reducir a 320 duplica velocidad

        self.subscription = self.create_subscription(
            Image, 'image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Landmarks, 'pose_landmarks', 10)

        # Estado compartido
        self._lock = threading.Lock()
        self._latest_frame = None
        self._latest_header = None
        self._result_pts = []        # últimos keypoints detectados
        self._frame_count = 0
        self._INFER_EVERY = 2        # inferir 1 de cada 2 frames (~30 detecciones/s)

        # Thread de inferencia
        self._infer_thread = threading.Thread(target=self._infer_loop, daemon=True)
        self._infer_thread.start()

        self.get_logger().info('Body Points Detector started')

    def image_callback(self, msg):
        self._frame_count += 1

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Actualizar frame para inferencia cada N frames
        if self._frame_count % self._INFER_EVERY == 0:
            with self._lock:
                self._latest_frame = frame.copy()
                self._latest_header = msg.header

        # Dibujar resultado anterior sin esperar inferencia
        with self._lock:
            pts = list(self._result_pts)
        vis = frame.copy()
        for j, (x, y) in enumerate(pts):
            cv2.circle(vis, (int(x), int(y)), 5, (0, 255, 0), -1)
            cv2.putText(vis, str(j), (int(x), int(y)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.putText(vis, "Yaren Vision - Debug Mode", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        vis = cv2.resize(vis, (800, 480))
        cv2.imshow(self.window_name, vis)
        cv2.waitKey(1)

    def _infer_loop(self):
        """Corre en su propio thread — no bloquea ROS."""
        import time
        while rclpy.ok():
            frame = None
            header = None
            with self._lock:
                if self._latest_frame is not None:
                    frame = self._latest_frame
                    header = self._latest_header
                    self._latest_frame = None

            if frame is None:
                time.sleep(0.005)
                continue

            # Inferencia con resolución reducida
            results = self.model(frame, imgsz=self.INFER_SIZE, verbose=False)
            keypoints = results[0].keypoints

            if keypoints is not None and len(keypoints.xy) > 0:
                pts = keypoints.xy[0].cpu().numpy()

                with self._lock:
                    self._result_pts = [(float(x), float(y)) for x, y in pts]

                # Publicar landmarks
                landmarks_msg = Landmarks()
                landmarks_msg.header = header
                for x, y in pts:
                    point = Point()
                    point.x = float(x)
                    point.y = float(y)
                    landmarks_msg.landmarks.append(point)
                self.publisher.publish(landmarks_msg)
            else:
                with self._lock:
                    self._result_pts = []

def main(args=None):
    rclpy.init(args=args)
    node = BodyPointsDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()