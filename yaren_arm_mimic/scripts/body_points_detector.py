#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
from yaren_interfaces.msg import BodyPoints
from geometry_msgs.msg import Point32
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

LANDMARK_GROUPS = [
    [11, 13, 15], 
    [12, 14, 16], 
]

def plot_world_landmarks(ax, landmarks, landmark_groups=LANDMARK_GROUPS):
    if landmarks is None:
        return

    ax.cla()

    ax.set_xlim3d(-1, 1)
    ax.set_ylim3d(-1, 1)
    ax.set_zlim3d(1, -1)

    for group in landmark_groups:
        plotX = [landmarks.landmark[i].x for i in group]
        plotY = [landmarks.landmark[i].y for i in group]
        plotZ = [landmarks.landmark[i].z for i in group]

        ax.plot(plotX, plotZ, plotY)  

    plt.pause(.001)

mp_pose = mp.solutions.pose

class BodyPointsDetectorNode(Node):
    def __init__(self):
        super().__init__('body_points_detector_node')
        self.bridge = CvBridge()
        self.pose = mp_pose.Pose(
            min_detection_confidence=0.5, 
            min_tracking_confidence=0.5,
            model_complexity=1, 
            smooth_landmarks=True
        )
        
        # CORRECCIÓN 1: Suscribirse al topic correcto
        self.subscription = self.create_subscription(
            Image, '/csi_camera/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(BodyPoints, 'body_points', 10)
        
        self.get_logger().info("BodyPointsDetectorNode iniciado - esperando imágenes de /csi_camera/image_raw")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.pose.process(image)

            points_msg = BodyPoints()
            points_msg.is_detected = False
            
            if results.pose_landmarks and results.pose_world_landmarks:
                world_landmarks = results.pose_world_landmarks.landmark
                
                # Verificar confianza de los landmarks de brazos
                right_shoulder_conf = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].visibility
                right_elbow_conf = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW].visibility
                right_wrist_conf = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST].visibility
                
                left_shoulder_conf = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER].visibility
                left_elbow_conf = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW].visibility
                left_wrist_conf = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST].visibility
                
                # Solo considerar detectado si los brazos tienen buena visibilidad
                MIN_VISIBILITY = 0.5  # Threshold de confianza
                
                arms_visible = (
                    right_wrist_conf > MIN_VISIBILITY and
                    left_wrist_conf > MIN_VISIBILITY
                )
                
                if not arms_visible:
                    self.get_logger().debug(
                        f"Brazos con baja visibilidad - R: {right_wrist_conf:.2f}, L: {left_wrist_conf:.2f}"
                    )
                    # No publicar si no se ven los brazos
                    return
                
                # Coordenadas de hombros para referencia
                points_msg.right_shoulder = Point32(
                    x=world_landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].x,
                    y=world_landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].y,
                    z=world_landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].z
                )
                
                points_msg.right_elbow = Point32(
                    x=world_landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].x,
                    y=world_landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].y,
                    z=world_landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].z
                )
                
                points_msg.right_wrist = Point32(
                    x=world_landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].x,
                    y=world_landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].y,
                    z=world_landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].z
                )
                
                points_msg.left_shoulder = Point32(
                    x=world_landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].x,
                    y=world_landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].y,
                    z=world_landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].z
                )
                
                points_msg.left_elbow = Point32(
                    x=world_landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].x,
                    y=world_landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].y,
                    z=world_landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].z
                )
                
                points_msg.left_wrist = Point32(
                    x=world_landmarks[mp_pose.PoseLandmark.LEFT_WRIST].x,
                    y=world_landmarks[mp_pose.PoseLandmark.LEFT_WRIST].y,
                    z=world_landmarks[mp_pose.PoseLandmark.LEFT_WRIST].z
                )
                
                points_msg.right_palm_rotation = 0.0
                points_msg.left_palm_rotation = 0.0
                
                points_msg.is_detected = True
                
                self.publisher.publish(points_msg)
                self.get_logger().debug(f"Persona detectada con brazos visibles")
            else:
                self.get_logger().debug("No se detectó persona en el frame")
                
        except Exception as e:
            self.get_logger().error(f"Error en image_callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BodyPointsDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()