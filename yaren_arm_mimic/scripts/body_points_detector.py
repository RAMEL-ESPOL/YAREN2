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
import time

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
            min_detection_confidence=0.5, min_tracking_confidence=0.5,
            model_complexity=1, smooth_landmarks=True )
        
        self.subscription = self.create_subscription(
            Image, 'image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(BodyPoints, 'body_points', 10)

        self.was_centered = False
        self.centered_time = None
        self.max_waiting_time=5.0

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image)

        points_msg = BodyPoints()
        points_msg.is_detected = False
        
        if results.pose_landmarks and results.pose_world_landmarks:
            world_landmarks = results.pose_world_landmarks.landmark
            img_landmarks   = results.pose_landmarks.landmark
            plot_world_landmarks(ax, results.pose_world_landmarks)

            points_msg.is_detected = True

            right_shoulder = Point32()
            right_shoulder.x = world_landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].x
            right_shoulder.y = world_landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].y
            right_shoulder.z = world_landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].z
            points_msg.right_shoulder = right_shoulder

            left_shoulder = Point32()
            left_shoulder.x = world_landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].x
            left_shoulder.y = world_landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].y
            left_shoulder.z = world_landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].z
            points_msg.left_shoulder = left_shoulder

            right_elbow = Point32()
            right_elbow.x = world_landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].x
            right_elbow.y = world_landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].y
            right_elbow.z = world_landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].z
            points_msg.right_elbow = right_elbow

            left_elbow = Point32()
            left_elbow.x = world_landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].x
            left_elbow.y = world_landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].y
            left_elbow.z = world_landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].z
            points_msg.left_elbow = left_elbow

            right_wrist = Point32()
            right_wrist.x = world_landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].x
            right_wrist.y = world_landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].y
            right_wrist.z = world_landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].z
            points_msg.right_wrist = right_wrist

            left_wrist = Point32()
            left_wrist.x = world_landmarks[mp_pose.PoseLandmark.LEFT_WRIST].x
            left_wrist.y = world_landmarks[mp_pose.PoseLandmark.LEFT_WRIST].y
            left_wrist.z = world_landmarks[mp_pose.PoseLandmark.LEFT_WRIST].z
            points_msg.left_wrist = left_wrist
        
            l2 = img_landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER]
            r2 = img_landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER]
            min_x, max_x = min(l2.x, r2.x), max(l2.x, r2.x)

            outside_center = min_x < 0.3 or max_x > 0.7

            if outside_center:
                if self.was_centered:
                    self.get_logger().info('Persona fuera de centro, reiniciando timer de centrado.')
                self.was_centered = False
                self.centered_time = None
                self.get_logger().info(
                    f"Off-center image (x-range={min_x:.2f}–{max_x:.2f}), skipping BodyPoints")
                return

            if not self.was_centered:
                current = time.time()
                if self.centered_time is None:
                    self.centered_time = current
                    self.get_logger().info('Centrado detectado. Esperando 5 segundos antes de publicar.')
                    return
                elif (current-self.centered_time)<self.max_waiting_time:
                    return
                else:
                    self.was_centered = True
                    self.centered_time = None

            self.publisher.publish(points_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BodyPointsDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()