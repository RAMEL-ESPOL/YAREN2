#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

PIPE = ("nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
        "nvvidconv ! video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink")

class CsiCamPub(Node):
    def __init__(self):
        super().__init__('csi_cam_pub')
        self.pub = self.create_publisher(Image, '/image_raw', 10)
        self.bridge = CvBridge()
        # Para usar la cámara USB por defecto en PC
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('No se pudo abrir la cámara CSI')
            raise RuntimeError('CSI camera not opened')
        self.timer = self.create_timer(1/30, self.loop)

    def loop(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('Frame no válido')
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub.publish(msg)
        

def main():
    rclpy.init()
    node = CsiCamPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
