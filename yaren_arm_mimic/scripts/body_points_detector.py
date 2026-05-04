#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
from yaren_interfaces.msg import BodyPoints
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
            min_detection_confidence=0.5, min_tracking_confidence=0.5,
            model_complexity=1, smooth_landmarks=True )
        
        self.subscription = self.create_subscription(
            Image, 'image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(BodyPoints, 'body_points', 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image)

        points_msg = BodyPoints()
        points_msg.is_detected = False
        
        if results.pose_landmarks and results.pose_world_landmarks:
            world_landmarks = results.pose_world_landmarks.landmark

            plot_world_landmarks(ax, results.pose_world_landmarks)

    
            points_msg.is_detected = True
            
            points_msg.right_shoulder_x = world_landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].x
            points_msg.right_elbow_x = world_landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].x
            points_msg.right_wrist_x = world_landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].x

            points_msg.left_shoulder_x = world_landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].x
            points_msg.left_elbow_x = world_landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].x
            points_msg.left_wrist_x = world_landmarks[mp_pose.PoseLandmark.LEFT_WRIST].x
            

            points_msg.right_shoulder_y = world_landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].y
            points_msg.right_elbow_y = world_landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].y
            points_msg.right_wrist_y = world_landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].y

            points_msg.left_shoulder_y = world_landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].y
            points_msg.left_elbow_y = world_landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].y
            points_msg.left_wrist_y = world_landmarks[mp_pose.PoseLandmark.LEFT_WRIST].y
            
            points_msg.right_shoulder_z = world_landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].z
            points_msg.right_elbow_z = world_landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].z
            points_msg.right_wrist_z = world_landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].z
            
            points_msg.left_shoulder_z = world_landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].z
            points_msg.left_elbow_z = world_landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].z
            points_msg.left_wrist_z = world_landmarks[mp_pose.PoseLandmark.LEFT_WRIST].z
            
            self.publisher.publish(points_msg)        

def main(args=None):
    rclpy.init(args=args)
    node = BodyPointsDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()