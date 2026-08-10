#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import mediapipe as mp
from yaren_interfaces.msg import BodyPoints, BodyPosition
from geometry_msgs.msg import Point32

LANDMARK_GROUPS = [
    [11, 13, 15],
    [12, 14, 16],
]

mp_pose = mp.solutions.pose

# ── Colores BGR ───────────────────────────────────────────────────────────────
BLUE   = (255, 100,   0)
ORANGE = (  0, 165, 255)
CYAN   = (255, 255,   0)
YELLOW = (  0, 220, 255)
WHITE  = (255, 255, 255)
BLACK  = (  0,   0,   0)
GREEN  = (  0, 255,   0)
RED    = (  0,   0, 255)


def draw_bar(img, x, y, value, lo, hi, label, color, bw=110, bh=13):
    pct  = max(0.0, min(1.0, (value - lo) / (hi - lo)))
    fill = int(pct * bw)
    cv2.rectangle(img, (x, y), (x + bw, y + bh), (50, 50, 50), -1)
    cv2.rectangle(img, (x, y), (x + fill, y + bh), color, -1)
    cv2.rectangle(img, (x, y), (x + bw, y + bh), WHITE, 1)
    cv2.putText(img, f"{label}:{value:+.1f}", (x + bw + 5, y + bh - 1),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, WHITE, 1, cv2.LINE_AA)


def dir_label(yx, zy):
    h = "→Der" if yx >  10 else ("←Izq" if yx < -10 else "·Cen")
    v = "↑Arr" if zy < -10 else ("↓Aba" if zy >  10 else "·Cen")
    return h, v


class BodyPointsDetectorNode(Node):
    def __init__(self):
        super().__init__('body_points_detector_node')
        self.bridge = CvBridge()
        self.pose = mp_pose.Pose(
            min_detection_confidence=0.2,
            min_tracking_confidence=0.2,
            model_complexity=1,
            smooth_landmarks=True
        )

        self.latest_pos = None  # BodyPosition más reciente

        self.sub_img = self.create_subscription(
            Image, '/csi_camera/image_raw', self.image_callback, 10)
        self.sub_pos = self.create_subscription(
            BodyPosition, 'body_tracker', self.pos_callback, 10)

        self.pub_points = self.create_publisher(BodyPoints,  'body_points',      10)
        self.pub_debug  = self.create_publisher(Image,       'arm_debug/image',  10)

        self.get_logger().info("BodyPointsDetectorNode iniciado")

    def pos_callback(self, msg):
        self.latest_pos = msg

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = cv2.flip(frame, 0)
            h, w  = frame.shape[:2]

            image   = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.pose.process(image)

            points_msg         = BodyPoints()
            points_msg.is_detected = False

            lm_image = None  # landmarks en coordenadas de imagen (normalizado)

            if results.pose_landmarks and results.pose_world_landmarks:
                world = results.pose_world_landmarks.landmark
                img_lm = results.pose_landmarks.landmark

                r_w_vis = img_lm[mp_pose.PoseLandmark.RIGHT_WRIST].visibility
                l_w_vis = img_lm[mp_pose.PoseLandmark.LEFT_WRIST].visibility

                if r_w_vis > 0.1 and l_w_vis > 0.1:
                    points_msg.right_shoulder = Point32(
                        x=world[mp_pose.PoseLandmark.RIGHT_SHOULDER].x,
                        y=world[mp_pose.PoseLandmark.RIGHT_SHOULDER].y,
                        z=world[mp_pose.PoseLandmark.RIGHT_SHOULDER].z)
                    points_msg.right_elbow = Point32(
                        x=world[mp_pose.PoseLandmark.RIGHT_ELBOW].x,
                        y=world[mp_pose.PoseLandmark.RIGHT_ELBOW].y,
                        z=world[mp_pose.PoseLandmark.RIGHT_ELBOW].z)
                    points_msg.right_wrist = Point32(
                        x=world[mp_pose.PoseLandmark.RIGHT_WRIST].x,
                        y=world[mp_pose.PoseLandmark.RIGHT_WRIST].y,
                        z=world[mp_pose.PoseLandmark.RIGHT_WRIST].z)
                    points_msg.left_shoulder = Point32(
                        x=world[mp_pose.PoseLandmark.LEFT_SHOULDER].x,
                        y=world[mp_pose.PoseLandmark.LEFT_SHOULDER].y,
                        z=world[mp_pose.PoseLandmark.LEFT_SHOULDER].z)
                    points_msg.left_elbow = Point32(
                        x=world[mp_pose.PoseLandmark.LEFT_ELBOW].x,
                        y=world[mp_pose.PoseLandmark.LEFT_ELBOW].y,
                        z=world[mp_pose.PoseLandmark.LEFT_ELBOW].z)
                    points_msg.left_wrist = Point32(
                        x=world[mp_pose.PoseLandmark.LEFT_WRIST].x,
                        y=world[mp_pose.PoseLandmark.LEFT_WRIST].y,
                        z=world[mp_pose.PoseLandmark.LEFT_WRIST].z)
                    points_msg.right_palm_rotation = 0.0
                    points_msg.left_palm_rotation  = 0.0
                    points_msg.is_detected = True
                    lm_image = img_lm

                    self.pub_points.publish(points_msg)

            # ── Construir frame de debug ──────────────────────────────────
            debug = self._build_debug(frame, w, h, lm_image, points_msg.is_detected)

            # Publicar como topic Y mostrar ventana local
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))
            cv2.imshow("YAREN Arm Debug", debug)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    # ── Frame de debug ────────────────────────────────────────────────────────
    def _build_debug(self, frame, w, h, lm_image, detected):
        canvas = frame.copy()

        # Paneles laterales semitransparentes
        overlay = canvas.copy()
        cv2.rectangle(overlay, (0,      0), (295,  h), BLACK, -1)
        cv2.rectangle(overlay, (w-295,  0), (w,    h), BLACK, -1)
        cv2.addWeighted(overlay, 0.6, canvas, 0.4, 0, canvas)

        # ── Esqueleto en imagen ───────────────────────────────────────────
        if lm_image is not None:
            def px(lm_idx):
                l = lm_image[lm_idx]
                return (int(l.x * w), int(l.y * h))

            rs = px(mp_pose.PoseLandmark.RIGHT_SHOULDER)
            re = px(mp_pose.PoseLandmark.RIGHT_ELBOW)
            rw = px(mp_pose.PoseLandmark.RIGHT_WRIST)
            ls = px(mp_pose.PoseLandmark.LEFT_SHOULDER)
            le = px(mp_pose.PoseLandmark.LEFT_ELBOW)
            lw = px(mp_pose.PoseLandmark.LEFT_WRIST)

            cv2.line(canvas, rs, re, BLUE,   3, cv2.LINE_AA)
            cv2.line(canvas, re, rw, CYAN,   3, cv2.LINE_AA)
            cv2.line(canvas, ls, le, ORANGE, 3, cv2.LINE_AA)
            cv2.line(canvas, le, lw, YELLOW, 3, cv2.LINE_AA)

            for pt, col, lbl in [
                (rs, BLUE,   "H.Der"), (re, CYAN,   "Codo.D"), (rw, WHITE, "Mano.D"),
                (ls, ORANGE, "H.Izq"), (le, YELLOW, "Codo.I"), (lw, WHITE, "Mano.I"),
            ]:
                cv2.circle(canvas, pt, 8, col, -1)
                cv2.circle(canvas, pt, 8, WHITE, 1)
                cv2.putText(canvas, lbl, (pt[0]+10, pt[1]-8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, col, 1, cv2.LINE_AA)

        # ── Banner estado ─────────────────────────────────────────────────
        status = "✓ DETECCION VALIDA" if detected else "✗ SIN DETECCION"
        s_col  = GREEN if detected else RED
        cv2.rectangle(canvas, (w//2-150, 4), (w//2+150, 28), BLACK, -1)
        cv2.putText(canvas, status, (w//2-140, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, s_col, 2, cv2.LINE_AA)

        pos = self.latest_pos

        # ── Paneles de ángulos ────────────────────────────────────────────
        self._arm_panel(canvas, pos, side='right', x0=6)
        self._arm_panel(canvas, pos, side='left',  x0=w-293)

        return canvas

    def _arm_panel(self, canvas, pos, side, x0):
        is_right = (side == 'right')
        title    = "BRAZO DERECHO" if is_right else "BRAZO IZQUIERDO"
        col      = BLUE if is_right else ORANGE

        y = 44
        cv2.putText(canvas, title, (x0, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, col, 2, cv2.LINE_AA)
        y += 6
        cv2.line(canvas, (x0, y), (x0+282, y), col, 1)
        y += 20

        if pos is None or not pos.is_valid:
            cv2.putText(canvas, "Sin datos del tracker", (x0, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, RED, 1, cv2.LINE_AA)
            return

        if is_right:
            sh_zy, sh_yx = pos.right_shoulder_elbow_zy, pos.right_shoulder_elbow_yx
            el_zy, el_yx = pos.right_elbow_wrist_zy,    pos.right_elbow_wrist_yx
        else:
            sh_zy, sh_yx = pos.left_shoulder_elbow_zy,  pos.left_shoulder_elbow_yx
            el_zy, el_yx = pos.left_elbow_wrist_zy,     pos.left_elbow_wrist_yx

        bar_col  = col
        bar_col2 = CYAN if is_right else YELLOW

        # ── Hombro ────────────────────────────────────────────────────────
        cv2.putText(canvas, "HOMBRO", (x0, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, WHITE, 1, cv2.LINE_AA)
        y += 17
        draw_bar(canvas, x0, y, sh_zy, -90, 90, "ZY", bar_col);  y += 20
        draw_bar(canvas, x0, y, sh_yx, -90, 90, "YX", bar_col);  y += 20
        h1, v1 = dir_label(sh_yx, sh_zy)
        cv2.putText(canvas, f"  {v1}  {h1}", (x0, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, YELLOW, 1, cv2.LINE_AA)
        y += 26

        # ── Codo ──────────────────────────────────────────────────────────
        cv2.putText(canvas, "CODO", (x0, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, WHITE, 1, cv2.LINE_AA)
        y += 17
        draw_bar(canvas, x0, y, el_zy, -90, 90, "ZY", bar_col2); y += 20
        draw_bar(canvas, x0, y, el_yx, -90, 90, "YX", bar_col2); y += 20
        h2, v2 = dir_label(el_yx, el_zy)
        cv2.putText(canvas, f"  {v2}  {h2}", (x0, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, YELLOW, 1, cv2.LINE_AA)
        y += 28

        # ── Valores exactos ───────────────────────────────────────────────
        cv2.putText(canvas, "GRADOS (raw)", (x0, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160, 160, 160), 1, cv2.LINE_AA)
        y += 15
        for txt in [f"  Sh ZY: {sh_zy:+7.2f}°", f"  Sh YX: {sh_yx:+7.2f}°",
                    f"  El ZY: {el_zy:+7.2f}°", f"  El YX: {el_yx:+7.2f}°"]:
            cv2.putText(canvas, txt, (x0, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, WHITE, 1, cv2.LINE_AA)
            y += 15

        # ── Límites del robot ─────────────────────────────────────────────
        y += 4
        cv2.putText(canvas, "LIMITES ROBOT (rad)", (x0, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (160, 160, 160), 1, cv2.LINE_AA)
        y += 14
        jnames = ["j5/9", "j6/10", "j7/11", "j8/12"]
        limits = [(-0.7853, 1.5708), (0.0, 1.0472), (-0.7853, 0.7853), (0.1745, 1.5708)]
        for jn, (lo, hi) in zip(jnames, limits):
            cv2.putText(canvas, f"  {jn}: [{lo:.3f}, {hi:.3f}]", (x0, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.37, (120, 200, 120), 1, cv2.LINE_AA)
            y += 13


def main(args=None):
    rclpy.init(args=args)
    node = BodyPointsDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()