#!/usr/bin/env python3
"""
yaren_rutinanueva.py  —  yaren_movements
Graba rutinas por imitación con countdown automático y calibración por T-pose.

Pipeline:
  csi_cam_pub  →  body_points_detector  →  body_tracker_node  →  /body_tracker
                                                                        ↓
                                                         [T-pose para calibrar neutro]
                                                         [detector de estabilidad]
                                                         [countdown 3s]
                                                         [guarda → publica JointTrajectory]
"""

import math
import time
import threading
import subprocess
import signal
import os
import yaml
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge

from yaren_interfaces.msg import BodyPosition

# ══════════════════════════════════════════════════════════════════════════════
#  Configuración
# ══════════════════════════════════════════════════════════════════════════════

DETECTION_NODES = [
    ["ros2", "run", "camara_usb_csi", "csi_cam_pub.py"],
    ["ros2", "run", "yaren_arm_mimic", "body_points_detector.py"],
    ["ros2", "run", "yaren_arm_mimic", "body_tracker_node"],
]

JOINT_NAMES = [
    "joint_1", "joint_2", "joint_3", "joint_4",
    "joint_5", "joint_6", "joint_7", "joint_8",
    "joint_9", "joint_10", "joint_11", "joint_12",
]

MAX_STEPS        = 10
STEP_DURATION    = 2
COUNTDOWN_SECS   = 3
STABILITY_FRAMES = 20
STABILITY_THRESH = 30.0

CALIB_HOMBRO_TARGET = 170.0
CALIB_THRESHOLD     = 20.0
CALIB_HOLD_FRAMES   = 8
PREP_SECS           = 5

ROUTINES_DIR = Path.home() / ".yaren" / "routines"
SCRIPTS_DIR  = Path("src/YAREN2/yaren_movements/yaren_movements")

# Paleta UI
BG_COLOR  = (13, 5, 30)
POSE_DATABASE = {
    "Reposo / Brazos Abajo": [0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0],
    "Brazos Arriba":         [3.0, 0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0],
    "T-Pose":                [1.5, 1.0, 0.0, 0.0, -1.5, 1.0, 0.0, 0.0],
    "Forma de L":            [1.5, 1.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0],
    "L Invertida":           [3.0, 0.0, 0.0, 0.0, -1.5, 1.0, 0.0, 0.0],
    "Brazos Adelante":       [1.5, 0.0, 0.0, 0.0, -1.5, 0.0, 0.0, 0.0]
}
ACCENT    = (251, 64, 224)
GREEN     = (0, 200, 80)
ORANGE    = (0, 160, 255)
RED_COLOR = (50, 50, 200)
YELLOW    = (0, 220, 220)
WHITE     = (255, 255, 255)
GRAY      = (100, 100, 100)
FONT      = cv2.FONT_HERSHEY_DUPLEX
FONT_S    = cv2.FONT_HERSHEY_PLAIN


# ══════════════════════════════════════════════════════════════════════════════
#  OnScreenKeyboard — Réplica del teclado C++ de face_screen
# ══════════════════════════════════════════════════════════════════════════════

class OnScreenKeyboard:
    """
    Teclado QWERTY táctil en pantalla.
    Misma estética y layout que el OnScreenKeyboard del face_screen C++.
    """

    ROWS_ALPHA = [
        ["q","w","e","r","t","y","u","i","o","p"],
        ["a","s","d","f","g","h","j","k","l"],
        ["z","x","c","v","b","n","m"],
    ]
    ROWS_NUM = [
        ["1","2","3","4","5","6","7","8","9","0"],
        ["-","/",":",";","(",")","$","&","@","\""],
        [".","_","#","!","?","=","+","<",">"],
    ]

    def __init__(self, prompt: str = "", initial: str = "", is_password: bool = False):
        self.text        = initial
        self.prompt      = prompt
        self.is_password = is_password
        self.visible     = True

        self._show_numbers  = False
        self._shift_active  = False
        self._hov_key       = -1
        self._hov_pt        = (0, 0)

        # Rects calculados en render()
        self._key_rects      = []
        self._btn_backspace  = (0, 0, 0, 0)
        self._btn_num_toggle = (0, 0, 0, 0)
        self._btn_ok         = (0, 0, 0, 0)
        self._btn_cancel     = (0, 0, 0, 0)
        self._btn_space      = (0, 0, 0, 0)
        self._btn_shift      = (0, 0, 0, 0)

        self.confirmed  = False   # True cuando el usuario presionó OK/CONECTAR
        self.cancelled  = False   # True cuando presionó CANCELAR

    # ── Helpers ────────────────────────────────────────────────────────────
    @staticmethod
    def _hit(rect, x, y):
        rx, ry, rw, rh = rect
        return rw > 0 and rh > 0 and rx <= x < rx + rw and ry <= y < ry + rh

    def _current_keys(self):
        return self.ROWS_NUM if self._show_numbers else self.ROWS_ALPHA

    def _find_key(self, x, y):
        for i, r in enumerate(self._key_rects):
            if self._hit(r, x, y):
                return i
        return -1

    # ── Eventos de ratón / touch ────────────────────────────────────────────
    def handle_mouse(self, event, x, y):
        """
        Llama desde el callback de OpenCV.
        Retorna True si el usuario confirmó (OK/CONECTAR).
        """
        self._hov_pt = (x, y)

        if event == cv2.EVENT_MOUSEMOVE:
            self._hov_key = self._find_key(x, y)
            return False

        if event == cv2.EVENT_LBUTTONUP:
            return False

        if event != cv2.EVENT_LBUTTONDOWN:
            return False

        # Backspace
        if self._hit(self._btn_backspace, x, y):
            if self.text:
                self.text = self.text[:-1]
            return False

        # Toggle numérico
        if self._hit(self._btn_num_toggle, x, y):
            self._show_numbers = not self._show_numbers
            return False

        # OK / GUARDAR
        if self._hit(self._btn_ok, x, y):
            self.confirmed = True
            self.visible   = False
            return True

        # CANCELAR
        if self._hit(self._btn_cancel, x, y):
            self.cancelled = True
            self.text      = ""
            self.visible   = False
            return False

        # Espacio
        if self._hit(self._btn_space, x, y):
            self.text += "_"  # guion bajo más útil para nombres de archivo
            return False

        # Shift
        if self._hit(self._btn_shift, x, y):
            self._shift_active = not self._shift_active
            return False

        # Teclas normales
        ki = self._find_key(x, y)
        if ki >= 0:
            rows = self._current_keys()
            flat = [k for row in rows for k in row]
            if ki < len(flat):
                ch = flat[ki]
                if self._shift_active and ch:
                    ch = ch.upper()
                    self._shift_active = False
                self.text += ch
        return False

    # ── Render ─────────────────────────────────────────────────────────────
    def render(self, frame: np.ndarray):
        if not self.visible:
            return

        H, W = frame.shape[:2]

        # Fondo semitransparente
        ov = frame.copy()
        cv2.rectangle(ov, (0, 0), (W, H), (2, 6, 16), -1)
        cv2.addWeighted(ov, 0.88, frame, 0.12, 0, frame)

        # Panel del teclado
        KBW, KBH = 760, 310
        KBX = (W - KBW) // 2
        KBY = H - KBH - 10
        cv2.rectangle(frame, (KBX, KBY), (KBX + KBW, KBY + KBH), (8, 14, 28), -1)
        cv2.rectangle(frame, (KBX, KBY), (KBX + KBW, KBY + KBH), (0, 150, 200), 2, cv2.LINE_AA)

        # Label + campo de texto
        field_y = KBY + 18
        cv2.putText(frame, self.prompt, (KBX + 16, field_y + 14),
                    FONT_S, 0.95, (80, 180, 220), 1, cv2.LINE_AA)

        fx, fy, fw, fh = KBX + 16, field_y + 20, KBW - 32, 32
        cv2.rectangle(frame, (fx, fy), (fx + fw, fy + fh), (4, 12, 26), -1)
        cv2.rectangle(frame, (fx, fy), (fx + fw, fy + fh), (0, 180, 230), 1, cv2.LINE_AA)

        # Texto con cursor parpadeante
        display = "*" * len(self.text) if self.is_password else self.text
        if int(time.time() * 2) % 2 == 0:
            display += "|"
        cv2.putText(frame, display, (fx + 8, fy + 22),
                    FONT_S, 1.1, (220, 235, 255), 1, cv2.LINE_AA)

        # ── Teclas ────────────────────────────────────────────────────────
        rows = self._current_keys()
        flat = [k for row in rows for k in row]
        self._key_rects = []

        KEY_H, KEY_GAP = 44, 5
        row_y = KBY + 75

        for row in rows:
            n = len(row)
            key_w = (KBW - KEY_GAP * (n + 1)) // n
            row_x = KBX + (KBW - (key_w * n + KEY_GAP * (n - 1))) // 2

            for ci, ch in enumerate(row):
                kx = row_x + ci * (key_w + KEY_GAP)
                kr = (kx, row_y, key_w, KEY_H)
                self._key_rects.append(kr)

                idx = len(self._key_rects) - 1
                hov = (idx == self._hov_key)
                bg   = (30, 80, 120) if hov else (14, 24, 42)
                bord = (0, 220, 255) if hov else (30, 60, 90)
                cv2.rectangle(frame, (kx, row_y), (kx + key_w, row_y + KEY_H), bg, -1)
                cv2.rectangle(frame, (kx, row_y), (kx + key_w, row_y + KEY_H), bord, 1, cv2.LINE_AA)

                label = ch
                if not self._show_numbers and self._shift_active and label:
                    label = label.upper()
                (tw, th), _ = cv2.getTextSize(label, FONT, 0.55, 1)
                cv2.putText(frame, label,
                            (kx + (key_w - tw) // 2, row_y + KEY_H // 2 + 7),
                            FONT, 0.55,
                            (255, 255, 255) if hov else (180, 200, 220),
                            1, cv2.LINE_AA)

            row_y += KEY_H + KEY_GAP

        # ── Fila de controles ──────────────────────────────────────────────
        ctrl_y = row_y
        ctrl_h = KEY_H
        hx, hy = self._hov_pt

        # 123 / ABC
        self._btn_num_toggle = (KBX + KEY_GAP, ctrl_y, 90, ctrl_h)
        bx, by, bw, bh = self._btn_num_toggle
        h_num = self._hit(self._btn_num_toggle, hx, hy)
        cv2.rectangle(frame, (bx, by), (bx + bw, by + bh),
                      (30, 50, 80) if h_num else (10, 20, 35), -1)
        cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (40, 80, 120), 1, cv2.LINE_AA)
        lbl_num = "ABC" if self._show_numbers else "123"
        cv2.putText(frame, lbl_num, (bx + 18, ctrl_y + ctrl_h // 2 + 7),
                    FONT, 0.55, (160, 200, 230), 1, cv2.LINE_AA)

        # Shift (solo alfa)
        if not self._show_numbers:
            self._btn_shift = (KBX + KEY_GAP + 95, ctrl_y, 70, ctrl_h)
            bx, by, bw, bh = self._btn_shift
            h_sh = self._hit(self._btn_shift, hx, hy)
            sh_bord = (0, 220, 120) if self._shift_active else (40, 80, 120)
            cv2.rectangle(frame, (bx, by), (bx + bw, by + bh),
                          (20, 50, 30) if h_sh else (10, 20, 35), -1)
            cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), sh_bord, 1, cv2.LINE_AA)
            cv2.putText(frame, "SHIFT", (bx + 6, ctrl_y + ctrl_h // 2 + 7),
                        FONT, 0.42,
                        (0, 230, 120) if self._shift_active else (140, 180, 200),
                        1, cv2.LINE_AA)
        else:
            self._btn_shift = (0, 0, 0, 0)

        # Espacio
        sp_x = KBX + (100 + KEY_GAP * 2 if self._show_numbers else 175 + KEY_GAP * 2)
        sp_w = KBW - sp_x + KBX - 100 - KEY_GAP * 3 - 90
        self._btn_space = (sp_x, ctrl_y, sp_w, ctrl_h)
        bx, by, bw, bh = self._btn_space
        h_sp = self._hit(self._btn_space, hx, hy)
        cv2.rectangle(frame, (bx, by), (bx + bw, by + bh),
                      (25, 50, 80) if h_sp else (12, 20, 38), -1)
        cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (30, 70, 110), 1, cv2.LINE_AA)
        cv2.putText(frame, "ESPACIO", (sp_x + sp_w // 2 - 36, ctrl_y + ctrl_h // 2 + 7),
                    FONT, 0.46, (120, 160, 190), 1, cv2.LINE_AA)

        # Backspace
        self._btn_backspace = (KBX + KBW - KEY_GAP - 90, ctrl_y, 90, ctrl_h)
        bx, by, bw, bh = self._btn_backspace
        h_bs = self._hit(self._btn_backspace, hx, hy)
        cv2.rectangle(frame, (bx, by), (bx + bw, by + bh),
                      (60, 20, 20) if h_bs else (25, 10, 10), -1)
        cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (140, 40, 40), 1, cv2.LINE_AA)
        cv2.putText(frame, "<-", (bx + 20, ctrl_y + ctrl_h // 2 + 7),
                    FONT, 0.55, (220, 100, 100), 1, cv2.LINE_AA)

        # Botones OK y CANCELAR (fila inferior)
        btn_row2_y = ctrl_y + ctrl_h + KEY_GAP

        self._btn_ok = (KBX + KBW - KEY_GAP - 200, btn_row2_y, 200, ctrl_h - 6)
        bx, by, bw, bh = self._btn_ok
        h_ok = self._hit(self._btn_ok, hx, hy)
        cv2.rectangle(frame, (bx, by), (bx + bw, by + bh),
                      (0, 60, 20) if h_ok else (0, 30, 10), -1)
        cv2.rectangle(frame, (bx, by), (bx + bw, by + bh),
                      (0, 180, 80), 2 if h_ok else 1, cv2.LINE_AA)
        cv2.putText(frame, "GUARDAR", (bx + 28, btn_row2_y + ctrl_h // 2 + 1),
                    FONT, 0.55, (0, 230, 100), 1, cv2.LINE_AA)

        self._btn_cancel = (KBX + KEY_GAP, btn_row2_y, 130, ctrl_h - 6)
        bx, by, bw, bh = self._btn_cancel
        h_can = self._hit(self._btn_cancel, hx, hy)
        cv2.rectangle(frame, (bx, by), (bx + bw, by + bh),
                      (40, 30, 10) if h_can else (18, 14, 6), -1)
        cv2.rectangle(frame, (bx, by), (bx + bw, by + bh),
                      (140, 100, 30), 2 if h_can else 1, cv2.LINE_AA)
        cv2.putText(frame, "CANCELAR", (bx + 4, btn_row2_y + ctrl_h // 2 + 1),
                    FONT, 0.42, (200, 160, 60), 1, cv2.LINE_AA)


# ══════════════════════════════════════════════════════════════════════════════
#  Estados
# ══════════════════════════════════════════════════════════════════════════════
class State:
    WAITING      = "waiting"
    ARMS_HIDDEN  = "arms_hidden"
    CALIBRATING  = "calibrating"
    T_POSE_HOLD  = "t_pose_hold"
    IMITATING    = "imitating"
    STABLE       = "stable"
    SAVING       = "saving"
    NAMING       = "naming"
    DONE_OPTIONS = "done_options"
    FINISHED     = "finished"


# ══════════════════════════════════════════════════════════════════════════════
#  Nodo ROS2
# ══════════════════════════════════════════════════════════════════════════════
class PoseRecorderNode(Node):

    def __init__(self):
        super().__init__("yaren_pose_recorder")

        self.state          = State.WAITING
        self.steps: list    = []
        self.countdown_start: float = 0.0
        self._lock          = threading.Lock()

        self._angle_window: deque = deque(maxlen=STABILITY_FRAMES)
        self._last_valid_bp = None
        self._detected_arm_angles: list[float] = [0.0] * 8

        self._human_neutral: list[float] = [170.0, 0.0, 0.0, 0.0, 170.0, 0.0, 0.0, 0.0]
        self._calibrated: bool = False
        self._t_pose_frame_count: int = 0
        self._calib_start: float = 0.0
        self.current_detected_pose = "Buscando..."

        self.routine_name = ""
        self.final_script_path = ""

        self.latest_frame = None
        self._bridge = CvBridge()

        self.create_subscription(
            BodyPosition, "/body_tracker",
            self._body_tracker_cb, 10)
        self.create_subscription(
            Image, "csi_camera/image_raw",
            self._camera_cb, 10)

        self._traj_pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory", 10)

        self.create_timer(2.0, self._debug_detection_status)
        self.get_logger().info("PoseRecorderNode listo.")

    def _debug_detection_status(self):
        with self._lock:
            if self._last_valid_bp and self._last_valid_bp.is_valid:
                cal = "CALIBRADO" if self._calibrated else "SIN CALIBRAR"
                self.get_logger().info(
                    f"DETECCION ACTIVA [{cal}] - "
                    f"hombro_der_yx: {self._last_valid_bp.right_shoulder_elbow_yx:.1f}°"
                )
            else:
                self.get_logger().warn(f"SIN DETECCION - estado: {self.state}")

    def _camera_cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.flip(frame, 0)
            with self._lock:
                self.latest_frame = frame
        except Exception:
            pass

    def _body_tracker_cb(self, msg):
        if not msg.is_valid:
            with self._lock:
                if self.state not in (State.SAVING, State.NAMING, State.DONE_OPTIONS, State.FINISHED):
                    self.state = State.WAITING
            return

        angles = [
            msg.right_shoulder_elbow_yx,
            msg.right_shoulder_elbow_zy,
            msg.right_elbow_wrist_yx,
            msg.right_elbow_wrist_zy,
            msg.left_shoulder_elbow_yx,
            msg.left_shoulder_elbow_zy,
            msg.left_elbow_wrist_yx,
            msg.left_elbow_wrist_zy,
        ]

        if any(math.isnan(a) or math.isinf(a) for a in angles):
            return

        arms_visible, _ = self._check_arms_visibility(msg)

        with self._lock:
            if not arms_visible:
                if self.state not in (State.NAMING, State.DONE_OPTIONS):
                    self.state = State.ARMS_HIDDEN
                    self._angle_window.clear()
                    self.countdown_start = 0.0
                    self._t_pose_frame_count = 0
                return

            self._detected_arm_angles = angles
            self._last_valid_bp = msg

            if self.state in (State.SAVING, State.NAMING, State.DONE_OPTIONS, State.FINISHED):
                return

            if not self._calibrated:
                self._handle_calibration(angles)
            else:
                self.current_detected_pose, _ = self._find_closest_pose(angles)
                self._angle_window.append(angles)
                self._update_state()

    def _check_arms_visibility(self, msg) -> tuple:
        rwd = math.sqrt(msg.right_wrist_x**2 + msg.right_wrist_y**2)
        right_valid = abs(msg.right_wrist_x) < 2.0 and abs(msg.right_wrist_y) < 2.0 and rwd > 0.05
        lwd = math.sqrt(msg.left_wrist_x**2 + msg.left_wrist_y**2)
        left_valid = abs(msg.left_wrist_x) < 2.0 and abs(msg.left_wrist_y) < 2.0 and lwd > 0.05
        if not right_valid and not left_valid:
            return False, "Ambos brazos incompletos"
        elif not right_valid:
            return False, "Brazo derecho incompleto"
        elif not left_valid:
            return False, "Brazo izquierdo incompleto"
        return True, ""

    def _is_calib_pose(self, angles: list) -> bool:
        return (abs(angles[0] - CALIB_HOMBRO_TARGET) < CALIB_THRESHOLD and
                abs(angles[4] - CALIB_HOMBRO_TARGET) < CALIB_THRESHOLD)

    def _handle_calibration(self, angles: list):
        if self.state == State.WAITING:
            self.state = State.CALIBRATING
            self._calib_start = time.time()

        if time.time() - self._calib_start < PREP_SECS:
            return

        if self._is_calib_pose(angles):
            self._t_pose_frame_count += 1
            self.state = State.T_POSE_HOLD
            if self._t_pose_frame_count >= CALIB_HOLD_FRAMES:
                self._human_neutral = [
                    angles[4], angles[5], angles[6], angles[7],
                    angles[0], angles[1], angles[2], angles[3],
                ]
                self._calibrated = True
                self._t_pose_frame_count = 0
                self.state = State.IMITATING
        else:
            self._t_pose_frame_count = 0
            if self.state == State.T_POSE_HOLD:
                self.state = State.CALIBRATING

    def _update_state(self):
        if self.state in (State.SAVING, State.NAMING, State.DONE_OPTIONS, State.FINISHED):
            return
        if len(self._angle_window) < STABILITY_FRAMES:
            self.state = State.IMITATING
            return
        arr = np.array(self._angle_window)
        max_variation = float(np.max(arr.max(axis=0) - arr.min(axis=0)))
        if max_variation > STABILITY_THRESH:
            self.state = State.IMITATING
            self.countdown_start = 0.0
            return
        now = time.time()
        if self.state == State.IMITATING:
            self.countdown_start = now
            self.state = State.STABLE
        if self.state == State.STABLE:
            if now - self.countdown_start >= COUNTDOWN_SECS:
                self._save_step()

    def _map_human_to_robot_angles(self, human_angles: list) -> list:
        reordered = [
            human_angles[4], human_angles[6], human_angles[7],
            human_angles[0], human_angles[2], human_angles[3],
        ]
        HUMAN_NEUTRAL = self._human_neutral
        ROBOT_CALIB   = [ 3.0,  0.0, 0.0, -3.0,  3.0, 0.0]
        DIRECTION     = [ 1.0, -1.0, 1.0, -1.0,  1.0, 1.0]
        JOINT_LIMITS  = [(0.0,3.0),(-3.0,0.0),(0.0,1.0),(-3.0,0.0),(0.0,3.0),(0.0,1.0)]
        r = []
        for i in range(6):
            delta_deg = reordered[i] - HUMAN_NEUTRAL[i]
            if abs(delta_deg) < 3.0:
                delta_deg = 0.0
            val = ROBOT_CALIB[i] + math.radians(delta_deg) * DIRECTION[i]
            lo, hi = JOINT_LIMITS[i]
            r.append(max(lo, min(hi, val)))
        j6  = max(0.0, min(1.0, (3.0 - r[0]) / 1.43))
        j10 = max(0.0, min(1.0, (3.0 - abs(r[3])) / 1.43))
        return [r[0], j6, r[1], r[2], r[3], j10, r[4], r[5]]

    def _find_closest_pose(self, human_angles: list) -> tuple:
        r_yx = human_angles[0]
        r_zy = human_angles[1]
        l_yx = human_angles[4]
        l_zy = human_angles[5]
        best_name = "Reposo / Brazos Abajo"
        if r_yx > 130 and l_yx > 130:
            best_name = "Brazos Arriba"
        elif r_yx < 45 and l_yx < 45:
            best_name = "Reposo / Brazos Abajo"
        elif r_yx > 130 and 45 <= l_yx <= 130:
            best_name = "Forma de L"
        elif l_yx > 130 and 45 <= r_yx <= 130:
            best_name = "L Invertida"
        elif 45 <= r_yx <= 130 and 45 <= l_yx <= 130:
            best_name = "Brazos Adelante" if (abs(r_zy) > 35 or abs(l_zy) > 35) else "T-Pose"
        return best_name, POSE_DATABASE[best_name]

    def get_current_pose_name(self) -> str:
        with self._lock:
            return self.current_detected_pose

    def _save_step(self):
        torso_joints = [0.0, 0.0, 0.0, 0.0]
        best_name, matched_arm_joints = self._find_closest_pose(self._detected_arm_angles)
        positions = torso_joints + list(matched_arm_joints)
        self.steps.append(positions)
        self.state = State.SAVING
        self.countdown_start = time.time()
        self.get_logger().info(f"Paso {len(self.steps)} guardado - Pose: {best_name}")
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions  = positions
        pt.velocities = [0.0] * len(JOINT_NAMES)
        pt.time_from_start = Duration(sec=STEP_DURATION, nanosec=0)
        msg.points = [pt]
        self._traj_pub.publish(msg)

    def countdown_progress(self) -> float:
        with self._lock:
            if self.state != State.STABLE or self.countdown_start == 0:
                return 0.0
            return min(1.0, (time.time() - self.countdown_start) / COUNTDOWN_SECS)

    def saving_progress(self) -> float:
        with self._lock:
            if self.state != State.SAVING:
                return 0.0
            return min(1.0, (time.time() - self.countdown_start) / 1.2)

    def t_pose_progress(self) -> float:
        with self._lock:
            return min(1.0, self._t_pose_frame_count / CALIB_HOLD_FRAMES)

    def is_calibrated(self) -> bool:
        with self._lock:
            return self._calibrated

    def get_state(self) -> str:
        with self._lock: return self.state

    def set_state(self, s: str):
        with self._lock: self.state = s

    def get_steps(self) -> list:
        with self._lock: return list(self.steps)

    def get_frame(self):
        with self._lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None


# ══════════════════════════════════════════════════════════════════════════════
#  UI
# ══════════════════════════════════════════════════════════════════════════════
WIN_W, WIN_H = 800, 480
WINDOW = "Yaren - Grabando Rutina"

_BW, _BH, _GAP = 180, 46, 16
_Y_BTN = WIN_H - _BH - 20
BTN_FINISH  = ((WIN_W // 2) - _BW - _GAP // 2, _Y_BTN, _BW, _BH)
BTN_CANCEL  = ((WIN_W // 2) + _GAP // 2, _Y_BTN, _BW, _BH)
BTN_RECALIB = (WIN_W - 210, _Y_BTN, 190, _BH)

BTN_PLAY_ROUTINE = (WIN_W // 2 - 200, WIN_H // 2, 180, 46)
BTN_EXIT_APP     = (WIN_W // 2 + 20,  WIN_H // 2, 180, 46)

_hover = ""

# Instancia global del teclado (se crea cuando hace falta)
_keyboard: OnScreenKeyboard | None = None


def _hit(btn, x, y):
    bx, by, bw, bh = btn
    return bx <= x < bx + bw and by <= y < by + bh


def _on_mouse(event, x, y, flags, node):
    global _hover, _keyboard
    state = node.get_state()

    # ── El teclado captura primero en estado NAMING ────────────────────────
    if state == State.NAMING and _keyboard is not None and _keyboard.visible:
        confirmed = _keyboard.handle_mouse(event, x, y)
        if confirmed:
            # Tomar el texto del teclado como nombre
            node.routine_name = _keyboard.text.strip()
            if node.routine_name:
                _save_routine_files(node)
                node.set_state(State.DONE_OPTIONS)
            else:
                # Nombre vacío → volver a mostrar teclado
                _keyboard.visible = True
                _keyboard.confirmed = False
        elif _keyboard.cancelled:
            # El usuario canceló el naming → volver a grabar
            node.set_state(State.IMITATING)
            _keyboard = None
        return

    # ── Hover normal ──────────────────────────────────────────────────────
    if event == cv2.EVENT_MOUSEMOVE:
        if state == State.DONE_OPTIONS:
            if _hit(BTN_PLAY_ROUTINE, x, y): _hover = "play"
            elif _hit(BTN_EXIT_APP, x, y):   _hover = "exit"
            else: _hover = ""
        else:
            if _hit(BTN_FINISH, x, y):    _hover = "finish"
            elif _hit(BTN_CANCEL, x, y):  _hover = "cancel"
            elif _hit(BTN_RECALIB, x, y): _hover = "recalib"
            else: _hover = ""

    if event == cv2.EVENT_LBUTTONDOWN:
        steps = node.get_steps()

        if state == State.DONE_OPTIONS:
            if _hit(BTN_PLAY_ROUTINE, x, y):
                print(f"[INFO] Reproduciendo rutina: {node.final_script_path}")
                subprocess.Popen(
                    ["python3", str(node.final_script_path)],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    preexec_fn=os.setsid
                )
                node.set_state(State.FINISHED)
            elif _hit(BTN_EXIT_APP, x, y):
                node.set_state(State.FINISHED)
            return

        if _hit(BTN_FINISH, x, y) and len(steps) > 0:
            # Abrir teclado en pantalla para nombrar la rutina
            _keyboard = OnScreenKeyboard(
                prompt="Nombre de la rutina (solo letras, numeros y _):",
                initial="",
                is_password=False
            )
            node.set_state(State.NAMING)
        elif _hit(BTN_CANCEL, x, y):
            node.set_state(State.FINISHED)
        elif _hit(BTN_RECALIB, x, y):
            with node._lock:
                node._calibrated = False
                node._t_pose_frame_count = 0
                node.state = State.CALIBRATING
                node._angle_window.clear()


# ── Helpers de dibujado ────────────────────────────────────────────────────
def _draw_header(canvas):
    title = "YAREN > MOVEMENTS > NUEVA RUTINA"
    cv2.rectangle(canvas, (0, 0), (WIN_W, 48), (0, 0, 0), -1)
    (tw, _), _ = cv2.getTextSize(title, FONT, 0.62, 1)
    cv2.putText(canvas, title, ((WIN_W - tw) // 2, 30),
                FONT, 0.62, ACCENT, 1, cv2.LINE_AA)


def _draw_steps(canvas, steps):
    r, gap = 14, 10
    total_w = MAX_STEPS * (r * 2) + (MAX_STEPS - 1) * gap
    x0 = (WIN_W - total_w) // 2
    y = 72
    cv2.rectangle(canvas, (x0 - 16, y - r - 6),
                  (x0 + total_w + 16, y + r + 22), (0, 0, 0), -1)
    for i in range(MAX_STEPS):
        cx2 = x0 + i * (r * 2 + gap) + r
        done  = i < len(steps)
        color = GREEN if done else (50, 30, 50)
        cv2.circle(canvas, (cx2, y), r, color, -1)
        cv2.circle(canvas, (cx2, y), r, WHITE if done else GRAY, 1, cv2.LINE_AA)
        if done:
            n = str(i + 1)
            (nw, nh), _ = cv2.getTextSize(n, FONT_S, 0.9, 1)
            cv2.putText(canvas, n, (cx2 - nw // 2, y + nh // 2),
                        FONT_S, 0.9, (0, 0, 0), 1, cv2.LINE_AA)
    lbl = f"{len(steps)} / {MAX_STEPS} pasos"
    (lw, _), _ = cv2.getTextSize(lbl, FONT_S, 0.9, 1)
    cv2.putText(canvas, lbl, ((WIN_W - lw) // 2, y + r + 18),
                FONT_S, 0.9, GRAY, 1, cv2.LINE_AA)


def _draw_calibration_panel(canvas, node):
    state      = node.get_state()
    calibrated = node.is_calibrated()
    if calibrated:
        return

    overlay = canvas.copy()
    cv2.rectangle(overlay, (0, 0), (WIN_W, WIN_H), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, canvas, 0.45, 0, canvas)

    cx, cy = WIN_W // 2, WIN_H // 2

    if state in (State.CALIBRATING, State.WAITING):
        elapsed   = time.time() - node._calib_start
        prep_left = max(0.0, PREP_SECS - elapsed)

        if prep_left > 0:
            txt1 = "MANTENE LOS BRAZOS ARRIBA ..."
            num  = str(math.ceil(prep_left))
            (w1, _),  _ = cv2.getTextSize(txt1, FONT, 0.80, 2)
            (nw, nh), _ = cv2.getTextSize(num,  FONT, 3.0,  4)
            cv2.putText(canvas, txt1, (cx - w1 // 2, cy - 80),
                        FONT, 0.80, ORANGE, 2, cv2.LINE_AA)
            cv2.putText(canvas, num,  (cx - nw // 2, cy + nh // 2),
                        FONT, 3.0, ORANGE, 4, cv2.LINE_AA)
        else:
            txt1 = "CALIBRACION NECESARIA"
            txt2 = "Levanta ambos brazos completamente"
            txt3 = "hacia arriba y mantelos quietos"
            (w1, _), _ = cv2.getTextSize(txt1, FONT,   0.80, 2)
            (w2, _), _ = cv2.getTextSize(txt2, FONT_S, 1.1,  1)
            (w3, _), _ = cv2.getTextSize(txt3, FONT_S, 1.0,  1)
            cv2.putText(canvas, txt1, (cx - w1 // 2, cy - 60), FONT,   0.80, YELLOW, 2, cv2.LINE_AA)
            cv2.putText(canvas, txt2, (cx - w2 // 2, cy),      FONT_S, 1.1,  WHITE,  1, cv2.LINE_AA)
            cv2.putText(canvas, txt3, (cx - w3 // 2, cy + 24), FONT_S, 1.0,  GRAY,   1, cv2.LINE_AA)
            _draw_arms_up_icon(canvas, cx, cy + 90)

    elif state == State.T_POSE_HOLD:
        prog  = node.t_pose_progress()
        txt   = "Mantente en T-pose..."
        (tw, _), _ = cv2.getTextSize(txt, FONT, 0.80, 1)
        cv2.putText(canvas, txt, (cx - tw // 2, cy - 110), FONT, 0.80, GREEN, 1, cv2.LINE_AA)
        r_arc = 70
        cv2.circle(canvas, (cx, cy), r_arc, (30, 30, 30), 6, cv2.LINE_AA)
        angle = int(360 * prog)
        if angle > 0:
            cv2.ellipse(canvas, (cx, cy), (r_arc, r_arc), -90, 0, angle, YELLOW, 6, cv2.LINE_AA)
        secs_left = max(1, math.ceil(3 * (1.0 - prog)))
        num = str(secs_left)
        (nw, nh), _ = cv2.getTextSize(num, FONT, 2.0, 3)
        cv2.putText(canvas, num, (cx - nw // 2, cy + nh // 2), FONT, 2.0, WHITE, 3, cv2.LINE_AA)
        _draw_arms_up_icon(canvas, cx, cy + r_arc + 50)


def _draw_arms_up_icon(canvas, cx, cy):
    cv2.circle(canvas, (cx, cy + 20), 12, YELLOW, 2, cv2.LINE_AA)
    cv2.line(canvas, (cx, cy + 32), (cx, cy + 80), YELLOW, 2, cv2.LINE_AA)
    cv2.line(canvas, (cx - 15, cy - 10), (cx, cy + 32), YELLOW, 2, cv2.LINE_AA)
    cv2.line(canvas, (cx + 15, cy - 10), (cx, cy + 32), YELLOW, 2, cv2.LINE_AA)
    cv2.line(canvas, (cx, cy + 80), (cx - 20, cy + 110), YELLOW, 2, cv2.LINE_AA)
    cv2.line(canvas, (cx, cy + 80), (cx + 20, cy + 110), YELLOW, 2, cv2.LINE_AA)


def _draw_state_panel(canvas, state, calibrated):
    if not calibrated:
        return
    msgs = {
        State.IMITATING:   ("Adopta una pose y mantenla", WHITE),
        State.STABLE:      ("Mantente quieto...", GREEN),
        State.WAITING:     ("Ponte frente a la camara", ORANGE),
        State.ARMS_HIDDEN: ("Ubiquese atras hasta ver brazos", ORANGE),
    }
    if state not in msgs:
        return
    txt, color = msgs[state]
    (tw, th), _ = cv2.getTextSize(txt, FONT, 0.70, 1)
    x, y, pad = (WIN_W - tw) // 2, WIN_H - 110, 14
    cv2.rectangle(canvas, (x - pad, y - th - pad), (x + tw + pad, y + pad), (0, 0, 0), -1)
    cv2.rectangle(canvas, (x - pad, y - th - pad), (x + tw + pad, y + pad), color, 1, cv2.LINE_AA)
    cv2.putText(canvas, txt, (x, y), FONT, 0.70, color, 1, cv2.LINE_AA)

    det_color = GREEN if state not in (State.WAITING, State.ARMS_HIDDEN) else RED_COLOR
    cv2.circle(canvas, (WIN_W - 24, 24), 8, det_color, -1, cv2.LINE_AA)
    det_txt = ("Detectado" if state not in (State.WAITING, State.ARMS_HIDDEN)
               else ("Brazos no visibles" if state == State.ARMS_HIDDEN else "Esperando..."))
    (dw, _), _ = cv2.getTextSize(det_txt, FONT_S, 0.9, 1)
    cv2.putText(canvas, det_txt, (WIN_W - dw - 36, 28), FONT_S, 0.9, det_color, 1, cv2.LINE_AA)

    cal_txt = "CAL OK"
    (cw, _), _ = cv2.getTextSize(cal_txt, FONT_S, 0.85, 1)
    cv2.rectangle(canvas, (10, 10), (cw + 20, 34), GREEN, -1)
    cv2.putText(canvas, cal_txt, (15, 28), FONT_S, 0.85, (0, 0, 0), 1, cv2.LINE_AA)


def _draw_countdown_arc(canvas, node):
    prog = node.countdown_progress()
    if prog <= 0:
        return
    cx, cy, r = WIN_W // 2, WIN_H // 2, 70
    cv2.circle(canvas, (cx, cy), r, (30, 30, 30), 6, cv2.LINE_AA)
    angle = int(360 * prog)
    if angle > 0:
        cv2.ellipse(canvas, (cx, cy), (r, r), -90, 0, angle, GREEN, 6, cv2.LINE_AA)
    secs_left = max(1, math.ceil(COUNTDOWN_SECS * (1.0 - prog)))
    num = str(secs_left)
    (nw, nh), _ = cv2.getTextSize(num, FONT, 2.0, 3)
    cv2.putText(canvas, num, (cx - nw // 2, cy + nh // 2), FONT, 2.0, WHITE, 3, cv2.LINE_AA)


def _draw_buttons(canvas, steps, calibrated):
    buttons = [
        (BTN_FINISH,  "FINALIZAR", GREEN,     "finish"),
        (BTN_CANCEL,  "CANCELAR",  RED_COLOR, "cancel"),
    ]
    if calibrated:
        buttons.append((BTN_RECALIB, "RECALIBRAR", YELLOW, "recalib"))

    for btn, label, base_color, key in buttons:
        bx, by, bw, bh = btn
        active = (key in ("cancel", "recalib")) or (key == "finish" and len(steps) > 0)
        hov    = (_hover == key) and active
        color  = base_color if active else (40, 40, 40)
        if hov:
            color = tuple(min(255, int(c * 1.3)) for c in color)
        cv2.rectangle(canvas, (bx, by), (bx + bw, by + bh), color, -1)
        cv2.rectangle(canvas, (bx, by), (bx + bw, by + bh),
                      WHITE if active else GRAY, 2 if hov else 1, cv2.LINE_AA)
        (tw, th), _ = cv2.getTextSize(label, FONT, 0.48, 1)
        cv2.putText(canvas, label,
                    (bx + (bw - tw) // 2, by + (bh + th) // 2 - 2),
                    FONT, 0.48, WHITE if active else GRAY, 1, cv2.LINE_AA)


def _draw_saving_flash(canvas, node, steps):
    prog  = node.saving_progress()
    alpha = max(0.0, 1.0 - prog) * 0.30
    if alpha > 0:
        flash = canvas.copy()
        cv2.rectangle(flash, (0, 0), (WIN_W, WIN_H), GREEN, -1)
        cv2.addWeighted(flash, alpha, canvas, 1 - alpha, 0, canvas)
    txt = f"Paso {len(steps)} guardado!"
    (tw, th), _ = cv2.getTextSize(txt, FONT, 1.0, 2)
    cv2.putText(canvas, txt, ((WIN_W - tw) // 2, WIN_H // 2),
                FONT, 1.0, WHITE, 2, cv2.LINE_AA)


def _draw_current_pose(canvas, node):
    if not node.is_calibrated():
        return
    pose_name = node.get_current_pose_name()
    if pose_name:
        txt = f"Pose: {pose_name}"
        (tw, th), _ = cv2.getTextSize(txt, FONT, 0.8, 2)
        x, y = 20, WIN_H - 90
        cv2.rectangle(canvas, (x - 10, y - th - 10), (x + tw + 10, y + 10), (0, 0, 0), -1)
        cv2.rectangle(canvas, (x - 10, y - th - 10), (x + tw + 10, y + 10), ACCENT, 2, cv2.LINE_AA)
        cv2.putText(canvas, txt, (x, y), FONT, 0.8, WHITE, 2, cv2.LINE_AA)


def _draw_naming_screen(canvas):
    """
    Dibuja solo el fondo oscuro semitransparente detrás del teclado.
    El propio OnScreenKeyboard renderiza el panel del teclado encima.
    """
    ov = canvas.copy()
    cv2.rectangle(ov, (0, 0), (WIN_W, WIN_H), (2, 4, 12), -1)
    cv2.addWeighted(ov, 0.70, canvas, 0.30, 0, canvas)

    # Título arriba
    title = "NOMBRE DE LA NUEVA RUTINA"
    (tw, _), _ = cv2.getTextSize(title, FONT, 0.75, 2)
    cv2.putText(canvas, title, ((WIN_W - tw) // 2, 55),
                FONT, 0.75, ACCENT, 2, cv2.LINE_AA)

    # Subtítulo
    sub = "Usa el teclado de abajo. Solo letras, numeros y _"
    (sw, _), _ = cv2.getTextSize(sub, FONT_S, 1.05, 1)
    cv2.putText(canvas, sub, ((WIN_W - sw) // 2, 90),
                FONT_S, 1.05, GRAY, 1, cv2.LINE_AA)


def _draw_done_options_screen(canvas, node):
    cv2.rectangle(canvas, (0, 0), (WIN_W, WIN_H), BG_COLOR, -1)

    title = "RUTINA GUARDADA CON EXITO"
    (tw, _), _ = cv2.getTextSize(title, FONT, 0.8, 2)
    cv2.putText(canvas, title, ((WIN_W - tw) // 2, 100), FONT, 0.8, GREEN, 2, cv2.LINE_AA)

    sub = f"Guardada como: yaren_{node.routine_name}.py"
    (sw, _), _ = cv2.getTextSize(sub, FONT_S, 1.2, 1)
    cv2.putText(canvas, sub, ((WIN_W - sw) // 2, 150), FONT_S, 1.2, WHITE, 1, cv2.LINE_AA)

    for btn, lbl, key, color in [
        (BTN_PLAY_ROUTINE, "REPRODUCIR AHORA", "play", ORANGE),
        (BTN_EXIT_APP,     "SALIR",             "exit", GRAY),
    ]:
        bx, by, bw, bh = btn
        hov = (_hover == key)
        c   = tuple(min(255, int(ch * 1.3)) for ch in color) if hov else color
        cv2.rectangle(canvas, (bx, by), (bx + bw, by + bh), c, -1)
        cv2.rectangle(canvas, (bx, by), (bx + bw, by + bh), WHITE, 2 if hov else 1, cv2.LINE_AA)
        (lw, lh), _ = cv2.getTextSize(lbl, FONT, 0.48, 1)
        cv2.putText(canvas, lbl, (bx + (bw - lw) // 2, by + (bh + lh) // 2 - 2),
                    FONT, 0.48, WHITE, 1, cv2.LINE_AA)


def _save_routine_files(node):
    steps = node.get_steps()
    name  = "yaren_" + node.routine_name
    yaml_path = ROUTINES_DIR / f"{name}.yaml"
    py_path   = SCRIPTS_DIR  / f"{name}.py"
    node.final_script_path = py_path

    ROUTINES_DIR.mkdir(parents=True, exist_ok=True)
    SCRIPTS_DIR.mkdir(parents=True, exist_ok=True)

    data = {
        "name":    name,
        "created": datetime.now().isoformat(),
        "joints":  JOINT_NAMES,
        "steps":   [{"step": i + 1, "positions": s} for i, s in enumerate(steps)],
    }
    yaml_path.write_text(yaml.dump(data, default_flow_style=False))
    generate_script(steps, name, py_path)
    print(f"[INFO] Rutina guardada en: {py_path}")


# ══════════════════════════════════════════════════════════════════════════════
#  Bucle principal de UI
# ══════════════════════════════════════════════════════════════════════════════
def run_ui(node: PoseRecorderNode) -> bool:
    global _keyboard

    canvas = np.zeros((WIN_H, WIN_W, 3), dtype=np.uint8)
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(WINDOW, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.setMouseCallback(WINDOW, _on_mouse, node)

    while True:
        state      = node.get_state()
        steps      = node.get_steps()
        calibrated = node.is_calibrated()

        # ── Pantalla normal de grabación ──────────────────────────────────
        if state not in (State.NAMING, State.DONE_OPTIONS):
            cam = node.get_frame()
            if cam is not None:
                canvas = cv2.resize(cam, (WIN_W, WIN_H))
                dark   = np.zeros_like(canvas)
                cv2.addWeighted(canvas, 0.45, dark, 0.55, 0, canvas)
            else:
                canvas[:] = BG_COLOR

            _draw_header(canvas)
            _draw_steps(canvas, steps)
            _draw_state_panel(canvas, state, calibrated)
            _draw_calibration_panel(canvas, node)
            _draw_countdown_arc(canvas, node)
            _draw_current_pose(canvas, node)
            _draw_buttons(canvas, steps, calibrated)

            if state == State.SAVING:
                _draw_saving_flash(canvas, node, steps)
                if node.saving_progress() >= 1.0:
                    node.set_state(
                        State.NAMING if len(steps) >= MAX_STEPS else State.IMITATING
                    )

        # ── Pantalla de nombre (teclado en pantalla) ──────────────────────
        elif state == State.NAMING:
            # Fondo: última imagen de cámara oscurecida (o color sólido)
            cam = node.get_frame()
            if cam is not None:
                canvas = cv2.resize(cam, (WIN_W, WIN_H))
                dark   = np.zeros_like(canvas)
                cv2.addWeighted(canvas, 0.25, dark, 0.75, 0, canvas)
            else:
                canvas[:] = BG_COLOR

            _draw_naming_screen(canvas)

            # Crear teclado si no existe
            if _keyboard is None or not _keyboard.visible:
                _keyboard = OnScreenKeyboard(
                    prompt="Nombre de la rutina (letras, numeros y _):",
                    initial="",
                    is_password=False
                )

            # Renderizar teclado encima
            _keyboard.render(canvas)

        # ── Pantalla de opciones finales ──────────────────────────────────
        elif state == State.DONE_OPTIONS:
            _draw_done_options_screen(canvas, node)

        cv2.imshow(WINDOW, canvas)

        key = cv2.waitKey(16)

        if state == State.FINISHED:
            break
        if key == 27:  # ESC
            node.set_state(State.FINISHED)
            break

        # Teclado físico como fallback en estado NAMING
        if state == State.NAMING and key != -1 and _keyboard is not None:
            if key in (8, 127):   # Backspace
                _keyboard.text = _keyboard.text[:-1]
            elif key in (13, 10): # Enter → confirmar
                if _keyboard.text.strip():
                    node.routine_name = _keyboard.text.strip()
                    _save_routine_files(node)
                    node.set_state(State.DONE_OPTIONS)
                    _keyboard = None
            elif 32 <= key <= 126:
                ch = chr(key)
                if ch.isalnum() or ch == "_":
                    if len(_keyboard.text) < 20:
                        _keyboard.text += ch

    cv2.destroyWindow(WINDOW)
    return len(node.get_steps()) > 0


# ══════════════════════════════════════════════════════════════════════════════
#  Generador de script
# ══════════════════════════════════════════════════════════════════════════════
def generate_script(steps: list, name: str, script_path: Path):
    posicion_base = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5]
    steps_con_base = steps + [posicion_base]
    script = f'''#!/usr/bin/env python3
""" {name}.py — Generado por yaren_pose_recorder """
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

JOINT_NAMES   = {JOINT_NAMES!r}
STEP_DURATION = {STEP_DURATION}
STEPS         = {steps_con_base!r}

class RutinaNode(Node):
    def __init__(self):
        super().__init__("{name}")
        self._pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory", 10)

    def run(self):
        self.get_logger().info("Esperando controlador...")
        timeout = 10.0
        start   = time.time()
        while self._pub.get_subscription_count() == 0:
            if time.time() - start > timeout:
                self.get_logger().error("Timeout: controlador no encontrado")
                return
            time.sleep(0.1)

        self.get_logger().info("Controlador conectado. Ejecutando rutina...")
        time.sleep(0.5)

        for i, positions in enumerate(STEPS):
            self.get_logger().info(f"Paso {{i+1}}/{{len(STEPS)}}")
            msg              = JointTrajectory()
            msg.joint_names  = JOINT_NAMES
            pt               = JointTrajectoryPoint()
            pt.positions     = positions
            pt.velocities    = [0.0] * len(JOINT_NAMES)
            pt.time_from_start = Duration(sec=STEP_DURATION, nanosec=0)
            msg.points       = [pt]
            self._pub.publish(msg)
            time.sleep(STEP_DURATION + 0.5)

        self.get_logger().info("Rutina completada.")

def main():
    rclpy.init()
    node = RutinaNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__": main()
'''
    with open(script_path, "w") as f:
        f.write(script)
    script_path.chmod(0o755)


# ══════════════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════════════
def main():
    detection_procs = []
    for cmd in DETECTION_NODES:
        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            detection_procs.append(proc)
            print(f"[INFO] Lanzado: {' '.join(cmd[-2:])}")
        except FileNotFoundError:
            print(f"[WARN] No se pudo lanzar: {' '.join(cmd)}")

    time.sleep(3.0)
    
    print("[INFO] Configurando y activando hardware de cámara...")
    subprocess.run(["ros2", "lifecycle", "set", "/csi_cam_node", "configure"])
    subprocess.run(["ros2", "lifecycle", "set", "/csi_cam_node", "activate"])

    rclpy.init()
    node = PoseRecorderNode()
    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True
    )
    spin_thread.start()

    print("[INFO] Recorder listo. Levanta los brazos para calibrar. ESC para cancelar.")
    try:
        run_ui(node)
    finally:
        print("[INFO] Iniciando limpieza...")

        # 1. PRIMERO: Cerrar la ventana de OpenCV
        print("[INFO] Cerrando ventana UI...")
        cv2.destroyAllWindows()
        cv2.waitKey(1)  # Crucial: Fuerza a OpenCV a vaciar la cola de eventos y cerrar gráficamente la ventana

        # 2. SEGUNDO: Desactivar y limpiar el hardware de la cámara CSI
        print("[INFO] Apagando y liberando cámara CSI...")
        subprocess.run(["ros2", "lifecycle", "set", "/csi_cam_node", "deactivate"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(["ros2", "lifecycle", "set", "/csi_cam_node", "cleanup"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # 3. TERCERO: Matar los subprocesos de detección (tracker, pose, etc.)
        print("[INFO] Deteniendo nodos de detección...")
        for proc in detection_procs:
            try:
                if proc.poll() is None:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    proc.wait(timeout=1.0)
            except Exception:
                pass

        # 4. CUARTO: Apagar ROS 2 de forma segura
        print("[INFO] Apagando nodo ROS 2...")
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        print("[INFO] Limpieza completada.")


if __name__ == "__main__":
    main()
