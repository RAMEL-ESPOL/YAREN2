#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from yaren_interfaces.msg import Landmarks
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory
import cv2

class BodyPointsDetectorNode(Node):
    def __init__(self):
        super().__init__('body_points_detector_node')
        self.bridge = CvBridge()
        pkg_share_dir = get_package_share_directory('yaren_dice')
        model_path = os.path.join(pkg_share_dir, 'models', 'yolov8s-pose.pt')
        self.model = YOLO(model_path)
        
        self.subscription = self.create_subscription(
            Image, 'image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Landmarks, 'pose_landmarks', 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Realizar la detección
        results = self.model(frame, verbose=False)

        keypoints = results[0].keypoints
        if keypoints is not None and len(keypoints.xy) > 0:
            landmarks_msg = Landmarks()
            landmarks_msg.header = msg.header
            
            # Obtener los puntos para ROS y para dibujar
            pts = keypoints.xy[0].cpu().numpy()
            
            for j, (x, y) in enumerate(pts):
                point = Point()
                point.x = float(x)
                point.y = float(y)
                landmarks_msg.landmarks.append(point)
                
                # --- DIBUJAR EN EL FRAME ---
                # Dibujamos un círculo pequeño en cada punto detectado
                cv2.circle(frame, (int(x), int(y)), 5, (0, 255, 0), -1)
                # Ponemos el número del punto (opcional, ayuda a debuguear)
                cv2.putText(frame, str(j), (int(x), int(y)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            self.publisher.publish(landmarks_msg)

        # --- MOSTRAR LA VENTANA ---
        # Añadimos un título a la imagen para saber qué estamos viendo
        cv2.putText(frame, "Yaren Vision - Debug Mode", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        cv2.imshow("Camara Yaren Dice", frame)
        cv2.waitKey(1) # Importante para que la ventana se actualice

def main(args=None):
    rclpy.init(args=args)
    node = BodyPointsDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()