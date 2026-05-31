#!/usr/bin/env python3
"""
face_landmark_publisher.py
──────────────────────────
Nodo ROS2 Lifecycle que publica landmarks faciales desde la cámara CSI.

Estados:
  inactive  → suscrito a la cámara pero NO procesa ni publica (CPU mínima)
  active    → procesa frames con MediaPipe y publica en face_landmarks

face_screen activa este nodo cuando cualquier filtro facial arranca
y lo desactiva cuando se regresa a idle.

Parámetros ROS2 (declarados, configurables desde launch):
  max_num_faces         (int,   default 2)
  min_detection_conf    (float, default 0.5)
  min_tracking_conf     (float, default 0.5)
  flip_frame            (bool,  default True)  -- CSI camera orientación
"""

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.lifecycle.node import LifecycleState
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from yaren_interfaces.msg import Landmarks
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
import cv2
import mediapipe as mp


class FaceLandmarkPublisher(LifecycleNode):

    def __init__(self):
        super().__init__('face_landmark_publisher')

        # Parámetros configurables sin recompilar
        self.declare_parameter('max_num_faces',      2)
        self.declare_parameter('min_detection_conf', 0.5)
        self.declare_parameter('min_tracking_conf',  0.5)
        self.declare_parameter('flip_frame',         True)

        self._bridge    = CvBridge()
        self._face_mesh = None   # se crea en on_activate
        self._active    = False  # flag interno para el callback

        self._sub = None
        self._pub = None

        self.get_logger().info('face_landmark_publisher creado (unconfigured).')

    # ── Lifecycle callbacks ────────────────────────────────────────────

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._pub = self.create_lifecycle_publisher(
            Landmarks, '/face_landmarks', 10)

        self._sub = self.create_subscription(
            Image, '/csi_camera/image_raw',
            self._image_callback, 10)

        self.get_logger().info('Configurado: suscrito a /csi_camera/image_raw.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # ── FIX: activar el lifecycle publisher ──
        super().on_activate(state)

        max_faces  = self.get_parameter('max_num_faces').value
        det_conf   = self.get_parameter('min_detection_conf').value
        track_conf = self.get_parameter('min_tracking_conf').value

        if self._face_mesh is None:
            self._face_mesh = mp.solutions.face_mesh.FaceMesh(
                static_image_mode=False,
                max_num_faces=max_faces,
                min_detection_confidence=det_conf,
                min_tracking_confidence=track_conf,
            )
            self.get_logger().info(
                f'MediaPipe FaceMesh inicializado '
                f'(caras={max_faces}, det={det_conf}, track={track_conf}).'
            )

        self._active = True
        self.get_logger().info('ACTIVADO — publicando face_landmarks.')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # ── FIX: desactivar el lifecycle publisher ──
        super().on_deactivate(state)

        self._active = False
        self.get_logger().info('DESACTIVADO — pausando publicación de landmarks.')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._release_mediapipe()
        if self._sub:
            self.destroy_subscription(self._sub)
            self._sub = None
        if self._pub:
            self.destroy_publisher(self._pub)
            self._pub = None
        self.get_logger().info('Limpieza completada.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._release_mediapipe()
        self.get_logger().info('Apagando face_landmark_publisher.')
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().error(f'Error en estado: {state.label}')
        self._release_mediapipe()
        return TransitionCallbackReturn.SUCCESS

    # ── Callback de imagen ─────────────────────────────────────────────

    def _image_callback(self, msg: Image):
        if not self._active:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Igual que el nodo original: flip ANTES de MediaPipe
            if self.get_parameter('flip_frame').value:
                frame = cv2.flip(frame, 0)

            rgb     = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self._face_mesh.process(rgb)

            if not results.multi_face_landmarks:
                return

            out = Landmarks()
            out.header = msg.header

            for lm in results.multi_face_landmarks[0].landmark:
                p   = Point()
                p.x = lm.x
                p.y = lm.y  # coordenadas directas, sin invertir
                out.landmarks.append(p)

            self._pub.publish(out)

        except Exception as e:
            self.get_logger().error(f'Error en callback: {e}')

    # ── Helpers ────────────────────────────────────────────────────────

    def _release_mediapipe(self):
        if self._face_mesh is not None:
            try:
                self._face_mesh.close()
            except Exception:
                pass
            self._face_mesh = None
            self.get_logger().info('MediaPipe liberado.')


# ── Entrypoint ─────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = FaceLandmarkPublisher()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()