#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int16
import numpy as np
import tensorflow as tf
import cv2
import mediapipe as mp
import os
import threading
from ament_index_python.packages import get_package_share_directory

EMOTIONS_LIST = ["Angry", "Disgust", "Fear", "Happy", "Sad", "Surprise", "Neutral"]

class EmotionDetectionNode(Node):
    def __init__(self):
        super().__init__('detector')

        self.window_name = "YAREN2 - Emotion Detector"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        pkg_path = get_package_share_directory('yaren_emotions')
        model_path = os.path.join(pkg_path, 'models', 'model_mbn_1.h5')
        self.model = tf.keras.models.load_model(model_path)
        
        dummy = np.zeros((1, 48, 48, 3), dtype=np.float32)
        self.model(dummy, training=False)

        mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Int16, '/emotion', 10)
        self.subscription = self.create_subscription(
    	Image, '/csi_camera/image_raw', self.image_callback, 10)


        self._lock = threading.Lock()
        self._latest_frame = None      
        self._result_label = "..."     
        self._result_box = None        
        self._frame_count = 0
        self._INFER_EVERY = 3

        self._infer_thread = threading.Thread(target=self._infer_loop, daemon=True)
        self._infer_thread.start()

        self.get_logger().info('Emotion Node: PANTALLA COMPLETA activada ✓')

    def image_callback(self, msg):
        self._frame_count += 1
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Error cv_bridge: {e}')
            return

        frame = cv2.flip(frame, -1)
        
        if self._frame_count % self._INFER_EVERY == 0:
            with self._lock:
                self._latest_frame = frame.copy()

        # Frame completo para visualización
        vis = frame.copy()
        
        with self._lock:
            label = self._result_label
            box = self._result_box

        # Dibujar solo el rectángulo y etiqueta, sin recortar
        if box is not None:
            x_min, y_min, x_max, y_max = box
            cv2.rectangle(vis, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            cv2.putText(vis, label, (x_min, max(y_min - 10, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)

        # Redimensionar al tamaño real de la pantalla
        vis = cv2.resize(vis, (800, 480))
        cv2.imshow(self.window_name, vis)
        cv2.waitKey(1)

    def _infer_loop(self):
        while rclpy.ok():
            frame = None
            with self._lock:
                if self._latest_frame is not None:
                    frame = self._latest_frame
                    self._latest_frame = None 

            if frame is None:
                import time; time.sleep(0.01)
                continue

            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.face_mesh.process(rgb_frame)

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

                    # Recorte SOLO para inferencia de IA
                    fc = rgb_frame[y_min:y_max, x_min:x_max]
                    if fc.size == 0: continue

                    roi = cv2.resize(fc, (48, 48)).astype(np.float32) / 255.0
                    roi = np.expand_dims(roi, axis=0)

                    preds = self.model(roi, training=False)
                    idx = int(np.argmax(preds))
                    
                    with self._lock:
                        self._result_label = EMOTIONS_LIST[idx]
                        self._result_box = (x_min, y_min, x_max, y_max)

                    self.publisher.publish(Int16(data=idx))
            else:
                with self._lock:
                    self._result_box = None

def main(args=None):
    rclpy.init(args=args)
    node = EmotionDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': 
    main()
