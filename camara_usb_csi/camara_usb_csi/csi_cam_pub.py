#!/usr/bin/env python3
import os
import cv2
import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
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


class CSICameraLifecycle(LifecycleNode):
    def __init__(self):
        super().__init__('csi_cam_node')

        self.declare_parameter('sensor_id',      0)
        self.declare_parameter('capture_width',  1920)
        self.declare_parameter('capture_height', 1080)
        self.declare_parameter('display_width',  1920)
        self.declare_parameter('display_height', 1080)
        self.declare_parameter('framerate',      30)
        self.declare_parameter('flip_method',    0)

        self.publisher_ = None
        self.timer      = None
        self.cap        = None
        self.bridge     = CvBridge()

        self.get_logger().info('Nodo Lifecycle de Cámara creado. En espera de configuración...')

    # ------------------------------------------------------------------ #
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.publisher_ = self.create_lifecycle_publisher(Image, 'csi_camera/image_raw', 10)
        self.get_logger().info('Cámara CONFIGURADA.')
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------ #
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        sensor_id      = self.get_parameter('sensor_id').value
        capture_width  = self.get_parameter('capture_width').value
        capture_height = self.get_parameter('capture_height').value
        display_width  = self.get_parameter('display_width').value
        display_height = self.get_parameter('display_height').value
        framerate      = self.get_parameter('framerate').value
        flip_method    = self.get_parameter('flip_method').value

        # --- Intento 1: Cámara USB real (verificada via sysfs) -----------
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
                    self.get_logger().info('Cámara USB detectada y en uso.')
                else:
                    cap_usb.release()
                    self.get_logger().warn('VideoCapture USB abre pero no entrega frames. Descartando.')
        else:
            self.get_logger().info('No se detectó hardware USB en /dev/video0.')

        # --- Intento 2: Pipeline CSI GStreamer (Jetson) ------------------
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
            self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if self.cap is None or not self.cap.isOpened():
            self.get_logger().error('Fallo total al abrir el hardware de la cámara.')
            return TransitionCallbackReturn.FAILURE

        self.timer = self.create_timer(1.0 / framerate, self.timer_callback)
        super().on_activate(state)
        self.get_logger().info('Cámara ACTIVADA. Capturando y publicando imágenes...')
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------ #
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Desactivando cámara...')

        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None

        if self.cap is not None:
            self.cap.release()
            self.cap = None

        super().on_deactivate(state)
        self.get_logger().info('Cámara DESACTIVADA. Hardware liberado correctamente.')
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------ #
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        if self.publisher_ is not None:
            self.destroy_publisher(self.publisher_)
            self.publisher_ = None
        self.get_logger().info('Cámara LIMPIADA (Unconfigured).')
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------ #
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
        if self.cap is not None:
            self.cap.release()
        self.get_logger().info('Cámara APAGADA por completo.')
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------ #
    def timer_callback(self):
        if self.cap is None or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Fallo al capturar frame.')
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'csi_camera'
        self.publisher_.publish(msg)


# ------------------------------------------------------------------ #
def main(args=None):
    rclpy.init(args=args)
    node = CSICameraLifecycle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
