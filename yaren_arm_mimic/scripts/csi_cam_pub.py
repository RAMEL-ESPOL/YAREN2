#!/usr/bin/env python3
import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def gstreamer_pipeline(
    sensor_id=0, capture_width=640, capture_height=480,
    display_width=640, display_height=480, framerate=30, flip_method=0,
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


def is_usb_camera(device_id: int) -> bool:
    """Verifica via sysfs si /dev/video{device_id} corresponde a hardware USB real."""
    device_path = f"/dev/video{device_id}"
    if not os.path.exists(device_path):
        return False
    sys_path = f"/sys/class/video4linux/video{device_id}/device/driver"
    try:
        driver = os.path.realpath(sys_path)
        return "usb" in driver.lower()
    except Exception:
        return False


class CSICameraPublisher(Node):
    def __init__(self):
        super().__init__('csi_cam_pub')

        # ── Parámetros ──
        self.declare_parameter('sensor_id',      0)
        self.declare_parameter('capture_width',  640)
        self.declare_parameter('capture_height', 480)
        self.declare_parameter('display_width',  640)
        self.declare_parameter('display_height', 480)
        self.declare_parameter('framerate',      30)
        self.declare_parameter('flip_method',    0)

        sensor_id      = self.get_parameter('sensor_id').value
        capture_width  = self.get_parameter('capture_width').value
        capture_height = self.get_parameter('capture_height').value
        display_width  = self.get_parameter('display_width').value
        display_height = self.get_parameter('display_height').value
        framerate      = self.get_parameter('framerate').value
        flip_method    = self.get_parameter('flip_method').value

        self.publisher_ = self.create_publisher(Image, '/csi_camera/image_raw', 10)
        self.bridge     = CvBridge()
        self.cap        = None
        self.frame_count = 0

        # ── 1. Intentar USB ──
        if is_usb_camera(0):
            self.get_logger().info('Hardware USB detectado en /dev/video0. Intentando abrir...')
            cap_usb = cv2.VideoCapture(0, cv2.CAP_V4L2)
            if cap_usb.isOpened():
                cap_usb.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                cap_usb.set(cv2.CAP_PROP_FRAME_WIDTH,  capture_width)
                cap_usb.set(cv2.CAP_PROP_FRAME_HEIGHT, capture_height)
                cap_usb.set(cv2.CAP_PROP_FPS,          framerate)
                cap_usb.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                ok, _ = cap_usb.read()
                if ok:
                    self.cap = cap_usb
                    self.get_logger().info('✅ Cámara USB detectada y en uso.')
                else:
                    cap_usb.release()
                    self.get_logger().warn('USB abre pero no entrega frames. Descartando.')
        else:
            self.get_logger().info('No se detectó hardware USB en /dev/video0.')

        # ── 2. Intentar CSI (GStreamer) ──
        if self.cap is None:
            self.get_logger().info('Intentando pipeline CSI GStreamer...')
            pipeline = gstreamer_pipeline(
                sensor_id=sensor_id,
                capture_width=capture_width,
                capture_height=capture_height,
                display_width=display_width,
                display_height=display_height,
                framerate=framerate,
                flip_method=flip_method,
            )
            self.get_logger().info(f'Pipeline: {pipeline}')
            self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

            if self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret and frame is not None and frame.size > 0:
                    self.get_logger().info('✅ Cámara CSI detectada y en uso.')
                else:
                    self.cap.release()
                    self.cap = None
                    self.get_logger().error('CSI abre pero no produce frames.')
            else:
                self.get_logger().error('No se pudo abrir cámara CSI.')

        # ── 3. Si nada funciona ──
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().error('❌ No se encontró ninguna cámara (USB ni CSI)')
            raise RuntimeError('No camera available')

        self.timer = self.create_timer(1.0 / framerate, self.timer_callback)
        self.get_logger().info('📷 Cámara publicando en /csi_camera/image_raw')

    def timer_callback(self):
        if self.cap is None or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            if self.frame_count % 30 == 0:
                self.get_logger().warning('Fallo al capturar frame.')
            self.frame_count += 1
            return

        self.frame_count += 1
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'csi_camera'
        self.publisher_.publish(msg)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CSICameraPublisher()
    except RuntimeError as e:
        print(f"❌ Error: {e}")
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
