#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from yaren_interfaces.msg import Landmarks
import mediapipe as mp
from cv_bridge import CvBridge
import cv2

class FaceLandmarkPublisher(Node):
    def __init__(self):
        super().__init__('face_landmark_publisher')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/csi_camera/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(Landmarks, 'face_landmarks', 10)
        
        self.face_mesh = mp.solutions.face_mesh.FaceMesh(
            static_image_mode=False,
            max_num_faces=2,
            min_detection_confidence=0.5
        )

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.flip(cv_image, 1)
            results = self.face_mesh.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            
            if results.multi_face_landmarks:
                landmarks_msg = Landmarks()
                landmarks_msg.header = msg.header
                for landmark in results.multi_face_landmarks[0].landmark:
                    point = Point()
                    point.x = landmark.x
                    point.y = landmark.y
                    landmarks_msg.landmarks.append(point)
                
                self.pub.publish(landmarks_msg)

        except Exception as e:
            self.get_logger().error(f"Error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = FaceLandmarkPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
