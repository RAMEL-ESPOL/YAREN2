#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CsiCamPub(Node):
    def __init__(self):
        super().__init__('camara')
        self.pub = self.create_publisher(Image, '/image_raw', 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(1, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            self.get_logger().error('No se pudo abrir la cámara USB')
            raise RuntimeError('USB camera not opened')

        # CRÍTICO: forzar MJPG antes de setear resolución/fps
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 60)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        fps_real = self.cap.get(cv2.CAP_PROP_FPS)
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f'Cámara: {w}x{h} @ {fps_real:.1f}fps')

        self.timer = self.create_timer(1/60, self.loop)

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