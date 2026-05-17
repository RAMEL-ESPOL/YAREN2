#!/usr/bin/env python3
"""
yaren_pose_recorder.py  —  yaren_movements
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
    ["ros2", "run", "yaren_arm_mimic", "csi_cam_pub.py"],
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
STABILITY_FRAMES = 10       # frames necesarios para considerar pose estable
STABILITY_THRESH = 20.0     # grados de tolerancia (más alto = más tolerante con niños)

# Constantes — reemplazar las de T-pose
CALIB_HOMBRO_TARGET = 170.0   # brazos arriba en vez de 90° horizontal
CALIB_THRESHOLD     = 20.0    # tolerancia ±20°
CALIB_HOLD_FRAMES   = 8
PREP_SECS           = 5       # ← segundos para ponerse en posición antes de detectar

ROUTINES_DIR = Path.home() / ".yaren" / "routines"
SCRIPTS_DIR  = Path("src/YAREN2/yaren_movements/yaren_movements")

# Paleta UI
BG_COLOR  = (13, 5, 30)
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
#  Estados
# ══════════════════════════════════════════════════════════════════════════════
class State:
    WAITING     = "waiting"       # esperando detectar persona
    ARMS_HIDDEN = "arms_hidden"   # brazos no visibles
    CALIBRATING = "calibrating"   # pidiendo T-pose para calibrar
    T_POSE_HOLD = "t_pose_hold"   # manteniendo T-pose (cuenta frames)
    IMITATING   = "imitating"     # imitando — esperando pose estable
    STABLE      = "stable"        # pose estable — iniciando countdown
    SAVING      = "saving"        # guardando paso
    FINISHED    = "finished"      # rutina terminada


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

        # Ventana de estabilidad
        self._angle_window: deque = deque(maxlen=STABILITY_FRAMES)
        self._last_valid_bp: BodyPosition | None = None

        # Ángulos detectados del humano
        self._detected_arm_angles: list[float] = [0.0] * 8

        # ── Calibración T-pose ──
        # Neutro humano medido en T-pose (se actualiza al calibrar)
        # Orden: L-HYX, L-CYX, L-CZY, R-HYX, R-CYX, R-CZY
        #                                    J5   J6   J7   J8    J9    J10  J11  J12
        self._human_neutral: list[float] = [170.0,0.0, 0.0, 0.0, 170.0, 0.0, 0.0, 0.0]
        self._calibrated: bool = False
        self._t_pose_frame_count: int = 0
        self._calib_start: float = 0.0
        
        # Cámara
        self.latest_frame: np.ndarray | None = None
        self._bridge = CvBridge()

        # Suscripciones
        self.create_subscription(
            BodyPosition, "/body_tracker",
            self._body_tracker_cb, 10)

        self.create_subscription(
            Image, "/csi_camera/image_raw",
            self._camera_cb, 10)

        # Publisher
        self._traj_pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory", 10)

        self.create_timer(2.0, self._debug_detection_status)
        self.get_logger().info("PoseRecorderNode listo.")

    # ── Debug ──────────────────────────────────────────────────────────────
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

    # ── Cámara ─────────────────────────────────────────────────────────────
    def _camera_cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            with self._lock:
                self.latest_frame = frame
        except Exception:
            pass

    # ── Callback principal ─────────────────────────────────────────────────
    def _body_tracker_cb(self, msg: BodyPosition):
        if not msg.is_valid:
            with self._lock:
                if self.state not in (State.SAVING, State.FINISHED):
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
                self.state = State.ARMS_HIDDEN
                self._angle_window.clear()
                self.countdown_start = 0.0
                self._t_pose_frame_count = 0
                return

            self._detected_arm_angles = angles
            self._last_valid_bp = msg

            # ── Flujo de estados ──
            if self.state in (State.SAVING, State.FINISHED):
                return

            if not self._calibrated:
                self._handle_calibration(angles)
            else:
                self._angle_window.append(angles)
                self._update_state()

    # ── Visibilidad de brazos ──────────────────────────────────────────────
    def _check_arms_visibility(self, msg: BodyPosition) -> tuple[bool, str]:
        right_wrist_dist = math.sqrt(msg.right_wrist_x**2 + msg.right_wrist_y**2)
        right_valid = (
            abs(msg.right_wrist_x) < 2.0 and
            abs(msg.right_wrist_y) < 2.0 and
            right_wrist_dist > 0.05
        )
        left_wrist_dist = math.sqrt(msg.left_wrist_x**2 + msg.left_wrist_y**2)
        left_valid = (
            abs(msg.left_wrist_x) < 2.0 and
            abs(msg.left_wrist_y) < 2.0 and
            left_wrist_dist > 0.05
        )
        if not right_valid and not left_valid:
            return False, "Ambos brazos incompletos"
        elif not right_valid:
            return False, "Brazo derecho incompleto"
        elif not left_valid:
            return False, "Brazo izquierdo incompleto"
        return True, ""

    def _is_calib_pose(self, angles: list[float]) -> bool:
        """Detecta brazos arriba: ambos hombros ~170°."""
        r_hombro = angles[0]
        l_hombro = angles[4]
        return (
            abs(r_hombro - CALIB_HOMBRO_TARGET) < CALIB_THRESHOLD and
            abs(l_hombro - CALIB_HOMBRO_TARGET) < CALIB_THRESHOLD)

    def _handle_calibration(self, angles: list[float]):
        """Gestiona la detección y confirmación de T-pose."""
        if self.state == State.WAITING:
            self.state = State.CALIBRATING
            self._calib_start = time.time()

        if time.time() - self._calib_start < PREP_SECS:
            return

        # ERROR CORREGIDO 1: Llamado correcto con un guion bajo
        if self._is_calib_pose(angles):
            self._t_pose_frame_count += 1
            self.state = State.T_POSE_HOLD

            # ERROR CORREGIDO 2: Variable correcta CALIB_HOLD_FRAMES
            if self._t_pose_frame_count >= CALIB_HOLD_FRAMES:
                # Calibrar con valores reales del humano en T-pose
                self._human_neutral = [
                    angles[4],  # L-HombroYX
                    angles[5],  # L-HombroZY
                    angles[6],  # L-CodoYX
                    angles[7],  # L-CodoZY
                    angles[0],  # R-HombroYX
                    angles[1],  # R-HombroZY
                    angles[2],  # R-CodoYX
                    angles[3],  # R-CodoZY
                ]
                self._calibrated = True
                self._t_pose_frame_count = 0
                self.state = State.IMITATING
                self.get_logger().info(
                    f"T-pose calibrada! Neutro: L-H={angles[4]:.1f}° "
                    f"R-H={angles[0]:.1f}°"
                )
        else:
            self._t_pose_frame_count = 0
            if self.state == State.T_POSE_HOLD:
                self.state = State.CALIBRATING

    # ── Actualizar estado (post-calibración) ───────────────────────────────
    def _update_state(self):
        if self.state in (State.SAVING, State.FINISHED):
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
            elapsed = now - self.countdown_start
            if elapsed >= COUNTDOWN_SECS:
                self._save_step()

    # ── Mapeo humano → robot ───────────────────────────────────────────────
    def _map_human_to_robot_angles(self, human_angles: list[float]) -> list[float]:
        reordered = [
            human_angles[4],  # L-HombroYX → J5
            human_angles[6],  # L-CodoYX   → J7
            human_angles[7],  # L-CodoZY   → J8
            human_angles[0],  # R-HombroYX → J9
            human_angles[2],  # R-CodoYX   → J11
            human_angles[3],  # R-CodoZY   → J12
        ]

        HUMAN_NEUTRAL = self._human_neutral

        ROBOT_CALIB = [ 3.0,  0.0, 0.0,   # J5, J7, J8
                       -3.0,  3.0, 0.0]   # J9, J11, J12

        DIRECTION   = [ 1.0, -1.0, 1.0,   # J5, J7, J8
                       -1.0,  1.0, 1.0]   # J9, J11, J12

        JOINT_LIMITS = [
            ( 0.0,  3.0),   # J5
            (-3.0,  0.0),   # J7
            ( 0.0,  1.0),   # J8
            (-3.0,  0.0),   # J9
            ( 0.0,  3.0),   # J11
            ( 0.0,  1.0),   # J12
        ]

        r = []
        for i in range(6):
            delta_deg = reordered[i] - HUMAN_NEUTRAL[i]
            if abs(delta_deg) < 3.0:
                delta_deg = 0.0
            val = ROBOT_CALIB[i] + math.radians(delta_deg) * DIRECTION[i]
            lo, hi = JOINT_LIMITS[i]
            r.append(max(lo, min(hi, val)))

        j6  = max(0.0, min(1.0, (3.0 - r[0])      / 1.43))
        j10 = max(0.0, min(1.0, (3.0 - abs(r[3])) / 1.43))

        return [r[0], j6, r[1], r[2],
                r[3], j10, r[4], r[5]]
                
    # ── Guardar paso ───────────────────────────────────────────────────────
    def _save_step(self):
        torso_joints = [0.0, 0.0, 0.0, 0.0]
        arm_joints   = self._map_human_to_robot_angles(self._detected_arm_angles)
        positions    = torso_joints + arm_joints

        self.steps.append(positions)
        self.state = State.SAVING
        self.countdown_start = time.time()

        deg_in  = [round(x, 1) for x in self._detected_arm_angles]
        rad_out = [round(x, 3) for x in arm_joints]
        self.get_logger().info(f"Paso {len(self.steps)} guardado")
        self.get_logger().info(f"  Humano (deg): {deg_in}")
        self.get_logger().info(f"  Robot  (rad): {rad_out}")

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions  = positions
        pt.velocities = [0.0] * len(JOINT_NAMES)
        pt.time_from_start = Duration(sec=STEP_DURATION, nanosec=0)
        msg.points = [pt]
        self._traj_pub.publish(msg)

    # ── Getters ────────────────────────────────────────────────────────────
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
            # ERROR CORREGIDO 3: Variable correcta CALIB_HOLD_FRAMES
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

    def get_frame(self) -> np.ndarray | None:
        with self._lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None


# ══════════════════════════════════════════════════════════════════════════════
#  UI
# ══════════════════════════════════════════════════════════════════════════════
WIN_W, WIN_H = 800, 480
WINDOW = "Yaren - Grabando Rutina"

_BW, _BH, _GAP = 180, 46, 16
_Y_BTN = WIN_H - _BH - 20
BTN_FINISH = ((WIN_W // 2) - _BW - _GAP // 2, _Y_BTN, _BW, _BH)
BTN_CANCEL = ((WIN_W // 2) + _GAP // 2, _Y_BTN, _BW, _BH)
BTN_RECALIB = (WIN_W - 210, _Y_BTN, 190, _BH)
_hover = ""


def _hit(btn, x, y):
    bx, by, bw, bh = btn
    return bx <= x < bx + bw and by <= y < by + bh


def _on_mouse(event, x, y, flags, node):
    global _hover
    if event == cv2.EVENT_MOUSEMOVE:
        if _hit(BTN_FINISH, x, y):   _hover = "finish"
        elif _hit(BTN_CANCEL, x, y): _hover = "cancel"
        elif _hit(BTN_RECALIB, x, y):_hover = "recalib"
        else: _hover = ""
    if event == cv2.EVENT_LBUTTONDOWN:
        steps = node.get_steps()
        if _hit(BTN_FINISH, x, y) and len(steps) > 0:
            node.set_state(State.FINISHED)
        elif _hit(BTN_CANCEL, x, y):
            node.set_state(State.FINISHED)
        elif _hit(BTN_RECALIB, x, y):
            with node._lock:
                node._calibrated = False
                node._t_pose_frame_count = 0
                node.state = State.CALIBRATING
                node._angle_window.clear()


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
        cx = x0 + i * (r * 2 + gap) + r
        done  = i < len(steps)
        color = GREEN if done else (50, 30, 50)
        cv2.circle(canvas, (cx, y), r, color, -1)
        cv2.circle(canvas, (cx, y), r, WHITE if done else GRAY, 1, cv2.LINE_AA)
        if done:
            n = str(i + 1)
            (nw, nh), _ = cv2.getTextSize(n, FONT_S, 0.9, 1)
            cv2.putText(canvas, n, (cx - nw // 2, y + nh // 2),
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
            txt1 = "PREPARATE PARA T-POSE"
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

            cv2.putText(canvas, txt1, (cx - w1 // 2, cy - 60),
                        FONT,   0.80, YELLOW, 2, cv2.LINE_AA)
            cv2.putText(canvas, txt2, (cx - w2 // 2, cy),
                        FONT_S, 1.1,  WHITE,  1, cv2.LINE_AA)
            cv2.putText(canvas, txt3, (cx - w3 // 2, cy + 24),
                        FONT_S, 1.0,  GRAY,   1, cv2.LINE_AA)

            _draw_arms_up_icon(canvas, cx, cy + 90)

    elif state == State.T_POSE_HOLD:
        prog = node.t_pose_progress()

        txt = "Mantente en T-pose..."
        (tw, _), _ = cv2.getTextSize(txt, FONT, 0.80, 1)
        cv2.putText(canvas, txt, (cx - tw // 2, cy - 110),
                    FONT, 0.80, GREEN, 1, cv2.LINE_AA)

        r_arc = 70
        cv2.circle(canvas, (cx, cy), r_arc, (30, 30, 30), 6, cv2.LINE_AA)
        angle = int(360 * prog)
        if angle > 0:
            cv2.ellipse(canvas, (cx, cy), (r_arc, r_arc),
                        -90, 0, angle, YELLOW, 6, cv2.LINE_AA)

        secs_left = max(1, math.ceil(3 * (1.0 - prog)))
        num = str(secs_left)
        (nw, nh), _ = cv2.getTextSize(num, FONT, 2.0, 3)
        cv2.putText(canvas, num, (cx - nw // 2, cy + nh // 2),
                    FONT, 2.0, WHITE, 3, cv2.LINE_AA)

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
        State.IMITATING: ("Adopta una pose y mantenla", WHITE),
        State.STABLE:    ("Mantente quieto...", GREEN),
        State.WAITING:   ("Ponte frente a la camara", ORANGE),
        State.ARMS_HIDDEN: ("Ubiquese atras hasta ver brazos", ORANGE),
    }
    if state not in msgs:
        return

    txt, color = msgs[state]
    (tw, th), _ = cv2.getTextSize(txt, FONT, 0.70, 1)
    x   = (WIN_W - tw) // 2
    y   = WIN_H - 110
    pad = 14
    cv2.rectangle(canvas, (x - pad, y - th - pad),
                  (x + tw + pad, y + pad), (0, 0, 0), -1)
    cv2.rectangle(canvas, (x - pad, y - th - pad),
                  (x + tw + pad, y + pad), color, 1, cv2.LINE_AA)
    cv2.putText(canvas, txt, (x, y), FONT, 0.70, color, 1, cv2.LINE_AA)

    det_color = GREEN if state not in (State.WAITING, State.ARMS_HIDDEN) else RED_COLOR
    cv2.circle(canvas, (WIN_W - 24, 24), 8, det_color, -1, cv2.LINE_AA)

    det_txt = "Detectado"
    if state == State.ARMS_HIDDEN: det_txt = "Brazos no visibles"
    elif state == State.WAITING:   det_txt = "Esperando..."

    (dw, _), _ = cv2.getTextSize(det_txt, FONT_S, 0.9, 1)
    cv2.putText(canvas, det_txt, (WIN_W - dw - 36, 28),
                FONT_S, 0.9, det_color, 1, cv2.LINE_AA)

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
    cv2.putText(canvas, num, (cx - nw // 2, cy + nh // 2),
                FONT, 2.0, WHITE, 3, cv2.LINE_AA)


def _draw_buttons(canvas, steps, calibrated):
    buttons = [
        (BTN_FINISH, "FINALIZAR", GREEN, "finish"),
        (BTN_CANCEL, "CANCELAR", RED_COLOR, "cancel"),
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


def run_ui(node: PoseRecorderNode) -> bool:
    canvas = np.zeros((WIN_H, WIN_W, 3), dtype=np.uint8)
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(WINDOW, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.setMouseCallback(WINDOW, _on_mouse, node)

    while True:
        state      = node.get_state()
        steps      = node.get_steps()
        calibrated = node.is_calibrated()
        cam        = node.get_frame()

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
        _draw_buttons(canvas, steps, calibrated)

        if state == State.SAVING:
            _draw_saving_flash(canvas, node, steps)
            if node.saving_progress() >= 1.0:
                node.set_state(
                    State.FINISHED if len(steps) >= MAX_STEPS else State.IMITATING
                )

        cv2.imshow(WINDOW, canvas)
        if state == State.FINISHED:
            break
        if cv2.waitKey(16) == 27:
            node.set_state(State.FINISHED)
            break

    cv2.destroyWindow(WINDOW)
    return len(node.get_steps()) > 0


# ══════════════════════════════════════════════════════════════════════════════
#  Generador de script
# ══════════════════════════════════════════════════════════════════════════════
def generate_script(steps: list, name: str, script_path: Path):
    script = f'''#!/usr/bin/env python3
""" {name}.py — Generado por yaren_pose_recorder """
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

JOINT_NAMES   = {JOINT_NAMES!r}
STEP_DURATION = {STEP_DURATION}
STEPS         = {steps!r}

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

    rclpy.init()
    node = PoseRecorderNode()
    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True
    )
    spin_thread.start()

    print("[INFO] Recorder listo. Haz T-pose para calibrar. ESC para cancelar.")
    try:
        rutina_guardada = run_ui(node)
    finally:
        print("[INFO] Iniciando limpieza...")
        node.destroy_node()
        rclpy.shutdown()
        time.sleep(0.5)

        for proc in detection_procs:
            try:
                if proc.poll() is None:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    proc.wait(timeout=1.0)
            except Exception:
                pass

        print("[INFO] Limpieza completada.")

        steps = node.get_steps()
        cv2.destroyAllWindows()

        if steps and rutina_guardada:
            name      = datetime.now().strftime("rutina_%d%m%Y_%H%M%S")
            yaml_path = ROUTINES_DIR / f"{name}.yaml"
            py_path   = SCRIPTS_DIR  / f"{name}.py"
            
            # ERROR CORREGIDO 4: Asegurar la creación de ambos directorios para evitar FileNotFoundError
            ROUTINES_DIR.mkdir(parents=True, exist_ok=True)
            SCRIPTS_DIR.mkdir(parents=True, exist_ok=True)

            data = {
                "name":    name,
                "created": datetime.now().isoformat(),
                "joints":  JOINT_NAMES,
                "steps":   [{"step": i + 1, "positions": s}
                             for i, s in enumerate(steps)],
            }
            yaml_path.write_text(yaml.dump(data, default_flow_style=False))
            generate_script(steps, name, py_path)
            print(f"[INFO] Rutina guardada: {py_path}")
            print(f"[INFO] Pasos: {len(steps)}")
        else:
            print("[INFO] Rutina omitida")


if __name__ == "__main__":
    main()