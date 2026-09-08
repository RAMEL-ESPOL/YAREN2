#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import mediapipe as mp
from yaren_interfaces.msg import BodyPoints, BodyPosition
from geometry_msgs.msg import Point32
import time

mp_pose = mp.solutions.pose

C_BG     = (8,  14, 26)
C_BLUE   = (255, 120,  40)
C_ORANGE = ( 40, 160, 255)
C_CYAN   = (255, 220,  60)
C_YELLOW = ( 40, 220, 255)
C_WHITE  = (220, 230, 245)
C_GREEN  = ( 60, 220,  80)
C_RED    = ( 40,  40, 220)
C_ACCENT = (  0, 200, 255)
C_PURPLE = (200,  80, 255)
C_PANEL  = ( 14,  22,  40)
C_GRID   = ( 15,  22,  38)


def rrect(img, x, y, w, h, r, color, fill=True, t=1):
    if fill:
        cv2.rectangle(img, (x+r, y),   (x+w-r, y+h),   color, -1)
        cv2.rectangle(img, (x,   y+r), (x+w,   y+h-r), color, -1)
        for cx, cy in [(x+r, y+r), (x+w-r, y+r), (x+r, y+h-r), (x+w-r, y+h-r)]:
            cv2.circle(img, (cx, cy), r, color, -1)
    else:
        cv2.line(img, (x+r, y), (x+w-r, y), color, t)
        cv2.line(img, (x+r, y+h), (x+w-r, y+h), color, t)
        cv2.line(img, (x, y+r), (x, y+h-r), color, t)
        cv2.line(img, (x+w, y+r), (x+w, y+h-r), color, t)
        cv2.ellipse(img, (x+r, y+r), (r, r), 0, 180, 270, color, t)
        cv2.ellipse(img, (x+w-r, y+r), (r, r), 0, 270, 360, color, t)
        cv2.ellipse(img, (x+w-r, y+h-r), (r, r), 0, 0, 90, color, t)
        cv2.ellipse(img, (x+r, y+h-r), (r, r), 0, 90, 180, color, t)


def bar(img, x, y, val, lo, hi, label, color, bw=130, bh=9):
    pct  = max(0.0, min(1.0, (val - lo) / (hi - lo)))
    fill = int(pct * bw)
    rrect(img, x, y, bw, bh, 3, (22, 32, 52))
    if fill > 6:
        rrect(img, x, y, fill, bh, 3, color)
    rrect(img, x, y, bw, bh, 3, (45, 65, 95), fill=False)

    cv2.putText(img, label, (x - 20, y + bh - 1),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, color, 1, cv2.LINE_AA)

    val_str = f"{val:+.1f}"
    val_x   = x + bw + 4
    val_y   = y + bh - 1
    if val_x + 38 < 195:
        cv2.putText(img, val_str, (val_x, val_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.30, C_WHITE, 1, cv2.LINE_AA)


def corners(img, x, y, w, h, color, size=14, thick=2):
    for px, py, dx, dy in [(x, y, 1, 1), (x+w, y, -1, 1),
                            (x, y+h, 1, -1), (x+w, y+h, -1, -1)]:
        cv2.line(img, (px, py), (px + dx*size, py),          color, thick, cv2.LINE_AA)
        cv2.line(img, (px, py), (px,           py + dy*size), color, thick, cv2.LINE_AA)


def skeleton(cam, lm, w, h, enabled):
    if lm is None:
        return
    def px(i):
        return (int(lm[i].x * w), int(lm[i].y * h))
    rs = px(mp_pose.PoseLandmark.RIGHT_SHOULDER)
    re = px(mp_pose.PoseLandmark.RIGHT_ELBOW)
    rw = px(mp_pose.PoseLandmark.RIGHT_WRIST)
    ls = px(mp_pose.PoseLandmark.LEFT_SHOULDER)
    le = px(mp_pose.PoseLandmark.LEFT_ELBOW)
    lw = px(mp_pose.PoseLandmark.LEFT_WRIST)
    r1 = C_BLUE   if enabled else (70, 75, 95)
    r2 = C_CYAN   if enabled else (55, 65, 85)
    l1 = C_ORANGE if enabled else (70, 75, 95)
    l2 = C_YELLOW if enabled else (55, 65, 85)
    for a, b, col in [(rs,re,r1),(re,rw,r2),(ls,le,l1),(le,lw,l2)]:
        cv2.line(cam, a, b, (0,0,0), 5)
        cv2.line(cam, a, b, col, 3, cv2.LINE_AA)
    for pt, col, r in [(rs,r1,7),(re,r2,6),(rw,C_WHITE,5),
                       (ls,l1,7),(le,l2,6),(lw,C_WHITE,5)]:
        cv2.circle(cam, pt, r+2, (0,0,0), -1)
        cv2.circle(cam, pt, r,   col,      -1, cv2.LINE_AA)
        cv2.circle(cam, pt, r,   C_WHITE,   1, cv2.LINE_AA)


class BodyPointsDetectorNode(Node):
    def __init__(self):
        super().__init__('body_points_detector_node')
        self.bridge        = CvBridge()
        self.pose          = mp_pose.Pose(
            min_detection_confidence=0.2,
            min_tracking_confidence=0.2,
            model_complexity=1,
            smooth_landmarks=True)
        self.latest_pos    = None
        self.mimic_enabled = False
        self._t0           = time.time()
        self.should_exit   = False  # Bandera para cerrar la app

        # Coordenadas del botón Volver
        self.BTN_X, self.BTN_Y, self.BTN_W, self.BTN_H = 680, 455, 110, 20

        self.sub_img  = self.create_subscription(
            Image, '/csi_camera/image_raw', self.image_callback, 10)
        self.sub_pos  = self.create_subscription(
            BodyPosition, 'body_tracker', self.pos_callback, 10)
        from std_msgs.msg import Bool
        self.sub_gate = self.create_subscription(
            Bool, '/mimic/enabled', self._cb_gate, 10)

        self.pub_pts = self.create_publisher(BodyPoints, 'body_points',     10)
        self.pub_dbg = self.create_publisher(Image,      'arm_debug/image', 10)

        cv2.namedWindow("YAREN Mimic", cv2.WINDOW_NORMAL)
        cv2.setWindowProperty("YAREN Mimic", cv2.WND_PROP_FULLSCREEN,
                              cv2.WINDOW_FULLSCREEN)
        
        # Asignar callback del ratón a la ventana
        cv2.setMouseCallback("YAREN Mimic", self.mouse_callback)
        self.get_logger().info("BodyPointsDetectorNode iniciado")

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # Comprobar si el clic está dentro del área del botón
            if self.BTN_X <= x <= self.BTN_X + self.BTN_W and self.BTN_Y <= y <= self.BTN_Y + self.BTN_H:
                self.get_logger().info("Botón Volver presionado.")
                self.should_exit = True

    def _cb_gate(self, msg):
        self.mimic_enabled = msg.data

    def pos_callback(self, msg):
        self.latest_pos = msg

    def image_callback(self, msg):
        try:
            frame  = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame  = cv2.flip(frame, 0)
            h, w   = frame.shape[:2]
            image  = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            result = self.pose.process(image)

            pts             = BodyPoints()
            pts.is_detected = False
            lm_img          = None

            if result.pose_landmarks and result.pose_world_landmarks:
                world  = result.pose_world_landmarks.landmark
                img_lm = result.pose_landmarks.landmark
                rvis   = img_lm[mp_pose.PoseLandmark.RIGHT_WRIST].visibility
                lvis   = img_lm[mp_pose.PoseLandmark.LEFT_WRIST].visibility
                if rvis > 0.1 and lvis > 0.1:
                    def p32(lm): return Point32(x=lm.x, y=lm.y, z=lm.z)
                    pts.right_shoulder      = p32(world[mp_pose.PoseLandmark.RIGHT_SHOULDER])
                    pts.right_elbow         = p32(world[mp_pose.PoseLandmark.RIGHT_ELBOW])
                    pts.right_wrist         = p32(world[mp_pose.PoseLandmark.RIGHT_WRIST])
                    pts.left_shoulder       = p32(world[mp_pose.PoseLandmark.LEFT_SHOULDER])
                    pts.left_elbow          = p32(world[mp_pose.PoseLandmark.LEFT_ELBOW])
                    pts.left_wrist          = p32(world[mp_pose.PoseLandmark.LEFT_WRIST])
                    pts.right_palm_rotation = 0.0
                    pts.left_palm_rotation  = 0.0
                    pts.is_detected         = True
                    lm_img                  = img_lm
                    self.pub_pts.publish(pts)

            debug = self._build(frame, w, h, lm_img, pts.is_detected)
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))
            cv2.imshow("YAREN Mimic", debug)
            
            # Detectar la tecla ESC (27) o la bandera del botón
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or self.should_exit:
                self.get_logger().info("Saliendo de YAREN Mimic...")
                sys.exit(0) # Termina el nodo y devuelve al proceso padre
                
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def _build(self, frame, fw, fh, lm_img, detected):
        W, H = 800, 480
        t    = time.time() - self._t0
        c    = np.full((H, W, 3), C_BG, dtype=np.uint8)

        # Grid
        for gx in range(0, W, 40):
            cv2.line(c, (gx, 0), (gx, H), C_GRID, 1)
        for gy in range(0, H, 40):
            cv2.line(c, (0, gy), (W, gy), C_GRID, 1)

        # ── CAMARA (mitad derecha) ────────────────────────────────────────
        CAM_X, CAM_Y, CAM_W, CAM_H = 210, 20, 582, 422
        cam = cv2.resize(frame, (CAM_W, CAM_H))
        if lm_img is not None:
            skeleton(cam, lm_img, CAM_W, CAM_H, self.mimic_enabled)
        c[CAM_Y:CAM_Y+CAM_H, CAM_X:CAM_X+CAM_W] = cam

        brd = C_GREEN if detected else C_RED
        cv2.rectangle(c, (CAM_X, CAM_Y), (CAM_X+CAM_W, CAM_Y+CAM_H), brd, 2)
        corners(c, CAM_X, CAM_Y, CAM_W, CAM_H, C_ACCENT)

        # ── PANEL IZQUIERDO ───────────────────────────────────────────────
        PW = 202
        rrect(c, 4, 4, PW, H - 12, 8, C_PANEL)
        rrect(c, 4, 4, PW, H - 12, 8, (28, 45, 75), fill=False)

        # Titulo
        cv2.putText(c, "YAREN", (16, 30),
                    cv2.FONT_HERSHEY_DUPLEX, 0.80, C_ACCENT, 2, cv2.LINE_AA)
        cv2.putText(c, "MIMIC", (16, 50),
                    cv2.FONT_HERSHEY_DUPLEX, 0.55, C_PURPLE, 1, cv2.LINE_AA)
        cv2.line(c, (12, 58), (PW - 6, 58), (30, 48, 78), 1)

        # Estado gate
        if self.mimic_enabled:
            pulse    = int(160 + 80 * abs(np.sin(t * 3.0)))
            gate_col = (0, pulse, min(255, pulse + 40))
            gate_txt = "ACTIVO"
            gate_bg  = (10, 35, 10)
        else:
            gate_col = (70, 80, 100)
            gate_txt = "EN ESPERA"
            gate_bg  = (20, 22, 35)

        rrect(c, 12, 64, PW - 20, 24, 5, gate_bg)
        rrect(c, 12, 64, PW - 20, 24, 5, gate_col, fill=False)
        cv2.putText(c, gate_txt, (20, 81),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, gate_col, 1, cv2.LINE_AA)

        # Deteccion
        det_col = C_GREEN if detected else C_RED
        det_bg  = (10, 32, 10) if detected else (32, 10, 10)
        det_txt = "DETECCION OK" if detected else "SIN DETECCION"
        rrect(c, 12, 94, PW - 20, 22, 5, det_bg)
        rrect(c, 12, 94, PW - 20, 22, 5, det_col, fill=False)
        cv2.putText(c, det_txt, (20, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.37, det_col, 1, cv2.LINE_AA)

        cv2.line(c, (12, 124), (PW - 6, 124), (28, 42, 68), 1)

        # Comando voz
        cv2.putText(c, "COMANDO DE VOZ:", (14, 140),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.34, (80, 100, 130), 1, cv2.LINE_AA)
        cmd_txt = "activar modo" if not self.mimic_enabled else "apagar modo"
        cmd_col = C_ACCENT      if not self.mimic_enabled else C_RED
        rrect(c, 12, 145, PW - 20, 20, 4, (14, 24, 42))
        rrect(c, 12, 145, PW - 20, 20, 4, cmd_col, fill=False)
        cv2.putText(c, cmd_txt, (18, 160),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, cmd_col, 1, cv2.LINE_AA)

        cv2.line(c, (12, 172), (PW - 6, 172), (28, 42, 68), 1)

        # Brazos
        self._side_panel(c, self.latest_pos, is_right=True,  x0=14, y0=178, pw=PW - 18)
        cv2.line(c, (12, 328), (PW - 6, 328), (28, 42, 68), 1)
        self._side_panel(c, self.latest_pos, is_right=False, x0=14, y0=334, pw=PW - 18)

        # Barra inferior
        cv2.rectangle(c, (0, H - 28), (W, H), (10, 16, 28), -1)
        cv2.line(c, (0, H - 28), (W, H - 28), (28, 44, 72), 1)
        cv2.putText(c, "RAMEL - ESPOL  |  yaren_arm_mimic",
                    (12, H - 8), cv2.FONT_HERSHEY_SIMPLEX,
                    0.34, (45, 65, 95), 1, cv2.LINE_AA)

        # Botón Volver
        rrect(c, self.BTN_X, self.BTN_Y, self.BTN_W, self.BTN_H, 4, (30, 40, 60))
        rrect(c, self.BTN_X, self.BTN_Y, self.BTN_W, self.BTN_H, 4, C_RED, fill=False)
        cv2.putText(c, "VOLVER (ESC)", (self.BTN_X + 12, self.BTN_Y + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, C_WHITE, 1, cv2.LINE_AA)

        return c

    def _side_panel(self, c, pos, is_right, x0, y0, pw):
        col  = C_BLUE   if is_right else C_ORANGE
        col2 = C_CYAN   if is_right else C_YELLOW
        ttl  = "BRAZO DER" if is_right else "BRAZO IZQ"
        bw   = 118

        cv2.putText(c, ttl, (x0, y0 + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, col, 1, cv2.LINE_AA)
        y = y0 + 18

        if pos is None or not pos.is_valid:
            cv2.putText(c, "Sin datos", (x0 + 6, y + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.34, (55, 65, 85), 1, cv2.LINE_AA)
            return

        sh_zy = pos.right_shoulder_elbow_zy if is_right else pos.left_shoulder_elbow_zy
        sh_yx = pos.right_shoulder_elbow_yx if is_right else pos.left_shoulder_elbow_yx
        el_zy = pos.right_elbow_wrist_zy    if is_right else pos.left_elbow_wrist_zy
        el_yx = pos.right_elbow_wrist_yx    if is_right else pos.left_elbow_wrist_yx

        cv2.putText(c, "HOMBRO", (x0, y + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.30, (100, 115, 140), 1, cv2.LINE_AA)
        y += 14
        bar(c, x0 + 22, y, sh_zy, -90, 90, "ZY", col,  bw=bw); y += 14
        bar(c, x0 + 22, y, sh_yx, -90, 90, "YX", col,  bw=bw); y += 18

        cv2.putText(c, "CODO", (x0, y + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.30, (100, 115, 140), 1, cv2.LINE_AA)
        y += 14
        bar(c, x0 + 22, y, el_zy, -90, 90, "ZY", col2, bw=bw); y += 14
        bar(c, x0 + 22, y, el_yx, -90, 90, "YX", col2, bw=bw)


def main(args=None):
    rclpy.init(args=args)
    node = BodyPointsDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()