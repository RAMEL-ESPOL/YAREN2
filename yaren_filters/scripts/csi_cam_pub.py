#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def gstreamer_pipeline(
    sensor_id=0, capture_width=1920, capture_height=1080,
    display_width=1920, display_height=1080, framerate=30, flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! video/x-raw, format=(string)BGR ! appsink"
        % (sensor_id, capture_width, capture_height, framerate,
           flip_method, display_width, display_height)
    )

class CSICameraPublisher(Node):
    def __init__(self):
        super().__init__('camara')
        self.declare_parameter('sensor_id',      0)
        self.declare_parameter('capture_width',  1920)
        self.declare_parameter('capture_height', 1080)
        self.declare_parameter('display_width',  1920)
        self.declare_parameter('display_height', 1080)
        self.declare_parameter('framerate',      30)
        self.declare_parameter('flip_method',    0)

        sensor_id      = self.get_parameter('sensor_id').value
        capture_width  = self.get_parameter('capture_width').value
        capture_height = self.get_parameter('capture_height').value
        display_width  = self.get_parameter('display_width').value
        display_height = self.get_parameter('display_height').value
        framerate      = self.get_parameter('framerate').value
        flip_method    = self.get_parameter('flip_method').value

        self.publisher_ = self.create_publisher(Image, 'csi_camera/image_raw', 10)
        self.bridge = CvBridge()

        pipeline = gstreamer_pipeline(
            sensor_id=sensor_id, capture_width=capture_width,
            capture_height=capture_height, display_width=display_width,
            display_height=display_height, framerate=framerate,
            flip_method=flip_method,
        )
        self.get_logger().info(f'GStreamer pipeline:\n{pipeline}')
        #CON CAMARA CSI:
        # self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        # CON CAMARA USB:
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Unable to open CSI camera!')
            raise RuntimeError('Camera not available')
        self.create_timer(1.0 / framerate, self.timer_callback)
        self.get_logger().info('CSI Camera Publisher started.')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Failed to capture frame.')
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'csi_camera'
        self.publisher_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CSICameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
