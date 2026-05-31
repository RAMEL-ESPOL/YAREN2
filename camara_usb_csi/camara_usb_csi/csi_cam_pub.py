#!/usr/bin/env python3
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

class CSICameraLifecycle(LifecycleNode):
    def __init__(self):
        # NOTA: El nombre del nodo debe coincidir con lo que C++ espera configurar
        super().__init__('csi_cam_node')
        
        self.declare_parameter('sensor_id',      0)
        self.declare_parameter('capture_width',  1920)
        self.declare_parameter('capture_height', 1080)
        self.declare_parameter('display_width',  1920)
        self.declare_parameter('display_height', 1080)
        self.declare_parameter('framerate',      30)
        self.declare_parameter('flip_method',    0)

        # Variables de estado
        self.publisher_ = None
        self.timer = None
        self.cap = None
        self.bridge = CvBridge()
        
        self.get_logger().info('Nodo Lifecycle de Cámara creado. En espera de configuración...')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        # Se crea el publisher, pero aún no transmite nada
        self.publisher_ = self.create_lifecycle_publisher(Image, 'csi_camera/image_raw', 10)
        self.get_logger().info('Cámara CONFIGURADA.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        sensor_id      = self.get_parameter('sensor_id').value
        capture_width  = self.get_parameter('capture_width').value
        capture_height = self.get_parameter('capture_height').value
        display_width  = self.get_parameter('display_width').value
        display_height = self.get_parameter('display_height').value
        framerate      = self.get_parameter('framerate').value
        flip_method    = self.get_parameter('flip_method').value

        pipeline = gstreamer_pipeline(
            sensor_id=sensor_id, capture_width=capture_width,
            capture_height=capture_height, display_width=display_width,
            display_height=display_height, framerate=framerate,
            flip_method=flip_method,
        )

        # Intento 1: Cámara USB (para pruebas en PC)
        self.cap = cv2.VideoCapture(0)
        
        # Intento 2: Fallback a GStreamer CSI (para la Jetson)
        if not self.cap.isOpened():
            self.get_logger().info('Cámara USB no detectada. Intentando pipeline CSI GStreamer...')
            self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not self.cap or not self.cap.isOpened():
            self.get_logger().error('Fallo total al abrir el hardware de la cámara.')
            return TransitionCallbackReturn.FAILURE

        # Iniciar el timer que lee los frames
        self.timer = self.create_timer(1.0 / framerate, self.timer_callback)
        
        # Habilitar el publisher subyacente
        super().on_activate(state)
        self.get_logger().info('Cámara ACTIVADA. Capturando y publicando imágenes...')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Desactivando cámara...')
        
        # 1. Apagar el temporizador para dejar de leer
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None
        
        # 2. Liberar físicamente la cámara
        if self.cap is not None:
            self.cap.release()
            self.cap = None

        super().on_deactivate(state)
        self.get_logger().info('Cámara DESACTIVADA. Hardware liberado correctamente.')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        if self.publisher_ is not None:
            self.destroy_publisher(self.publisher_)
            self.publisher_ = None
        self.get_logger().info('Cámara LIMPIADA (Unconfigured).')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
        if self.cap is not None:
            self.cap.release()
        self.get_logger().info('Cámara APAGADA por completo.')
        return TransitionCallbackReturn.SUCCESS

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