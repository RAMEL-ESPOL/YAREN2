#!/usr/bin/env python3
"""
dance_game.py -- Yaren Dance (OpenCV) | diseno estilo Just Dance
=================================================================
LifecycleNode. Camara via /csi_camera/image_raw. Mismo formato que memoria_node.py.

NOTA: todo el texto que dibuja OpenCV debe ser ASCII (sin tildes ni enie), porque
cv2.putText las dibuja como "??". Por seguridad, _ascii() limpia cualquier texto.

Diseno nuevo:
  - Fondos de neon por cancion (degradado + haces de luz + vineta + suelo con rejilla + bokeh animado).
  - Coach en silueta rellena con borde oscuro y glow del color de la cancion.
  - Pictogramas que se deslizan por la franja de abajo hacia el aro "AHORA" (como Just Dance).
  - HUD arriba: puntaje, combo, barra de estrellas (5 estrellas) y titulo de la cancion.
  - Calificaciones (PERFECT / GOOD / OK / MISS) con animacion "pop", particulas y aro de impacto.
  - Menu con tarjetas de colores (3x2), pantalla de bienvenida con titulo en degradado y 3 bailarines.
  - Resumen con estrellas que se revelan una a una, puntaje que sube y chips de estadisticas.

Ajustes de rendimiento: FX_GLOW y FX_BOKEH (ponlos en False si el robot va lento).

Correcciones de poses: arms_up (era igual a t_pose), right_arm (brazo izq. mal angulo),
hands_head y el segundo segmento de los brazos (se dibujaba desplazado).
"""

import math
import random
import threading
import time
import os
import glob
import unicodedata
import faulthandler
import traceback

import cv2
import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

_T0 = time.perf_counter()


def _log_t(msg):
    print(f"[TIMING +{time.perf_counter() - _T0:6.2f}s] {msg}", flush=True)


# =============================================================================
#  PATHS UNIVERSALES
# =============================================================================
HOME_DIR = os.path.expanduser("~")
BASE_WS = os.path.join(HOME_DIR, "robotis_ws", "src", "YAREN2")

AUDIO_DIR = os.path.join(BASE_WS, "yaren_radio", "audios")
FONDOS_DIR = os.path.join(BASE_WS, "yaren_juegos", "fondos")
YOLO_MODEL = os.path.join(HOME_DIR, "robotis_ws", "install", "yaren_dice", "share",
                          "yaren_dice", "models", "yolov8s-pose.pt")

SONG_MUSIC = {
    "song_1": os.path.join(AUDIO_DIR, "Balada.mp3"),
    "song_2": os.path.join(AUDIO_DIR, "justthewayuare.mp3"),
    "song_3": os.path.join(AUDIO_DIR, "Luli Pampin - ARAM SAM SAM 2021.mp3"),
    "song_4": os.path.join(AUDIO_DIR, "Luli Pampin - SI TU TIENES MUCHAS GANAS DE APLAUDIR - Official Video.mp3"),
    "song_5": os.path.join(AUDIO_DIR, "The Winner Takes It All.mp3"),
    "song_6": os.path.join(AUDIO_DIR, "Rihanna - Don't Stop The Music.mp3"),
    "song_7": os.path.join(AUDIO_DIR, "Prince Royce - La Carretera.mp3"),
    "song_8": os.path.join(AUDIO_DIR, "La Despedida.mp3"),
    "song_9": os.path.join(AUDIO_DIR, "Justin Bieber - Beauty And A Beat.mp3"),
    "song_10": os.path.join(AUDIO_DIR, "Gotas De Lluvia, Grupo Niche.mp3"),
    "song_11": os.path.join(AUDIO_DIR, "Belanova - Rosa Pastel.mp3"),
    "song_12": os.path.join(AUDIO_DIR, "Aitana - SUPERESTRELLA.mp3"),
    "song_13": os.path.join(AUDIO_DIR, "El Dia De Mi Suerte.mp3"),
    "song_14": os.path.join(AUDIO_DIR, "Goo Goo Dolls - Iris.mp3"),
}
MUSIC_VOLUME = 0.6

# =============================================================================
#  CONFIG  --  800x480
# =============================================================================
WIN_NAME = "Yaren Dance"
WIN_W = 800
WIN_H = 480
FPS = 60
STEP_DUR = 2.4

FX_GLOW = True     # glow alrededor del coach (si va lento, ponlo en False)
FX_BOKEH = True    # circulos de luz flotando en el fondo

# Layout de la pantalla de juego
ACTIVE_X = int(WIN_W * 0.28)
GHOST_X = int(WIN_W * 0.74)
FIG_Y = 250
FLOOR_Y = 346
STRIP_Y0, STRIP_Y1 = 394, 474
STRIP_CY = (STRIP_Y0 + STRIP_Y1) // 2
TARGET_X = 84            # aro "AHORA" de los pictogramas
PX_PER_PROG = 300        # pixeles por unidad de progreso
P_NOW = 0.62             # progreso que cae justo sobre el aro

STAR_T = (0.20, 0.40, 0.60, 0.80, 0.95)   # umbrales de las 5 estrellas

C_WHITE = (255, 255, 255)
C_BLACK = (0, 0, 0)
C_YELLOW = (0, 220, 255)
C_RED = (60, 60, 255)
C_LIME = (50, 255, 80)
C_GOLD = (0, 200, 255)
C_ORANGE = (30, 140, 245)
C_GRAY = (100, 100, 100)
C_ARROW = (255, 210, 0)
C_GREEN = (50, 255, 80)
C_PINK = (170, 60, 250)
C_OKBLUE = (255, 200, 0)

UI_BG = (10, 8, 20)
UI_PANEL = (22, 18, 40)
UI_ACCENT = (230, 160, 60)
UI_TEXT = (240, 235, 255)
UI_DIM = (170, 150, 195)
UI_DIVIDER = (55, 42, 78)

# Keypoints COCO (YOLO)
LEFT_SHOULDER = 5
RIGHT_SHOULDER = 6
LEFT_ELBOW = 7
RIGHT_ELBOW = 8
LEFT_WRIST = 9
RIGHT_WRIST = 10
LEFT_HIP = 11
RIGHT_HIP = 12

POSE_HOLD_NEEDED = 0.4
CAM_INFER_SIZE = 320


class GameState:
    WELCOME = 0
    MENU = 1
    PLAYING = 2
    SUMMARY = 3
    CONFIRM_EXIT = 4
    EXITED = 5
    MODE_SELECT = 6


# =============================================================================
#  DETECCION DE POSE
# =============================================================================
def detect_pose(kpts):
    if kpts is None or len(kpts) < 13:
        return frozenset()

    ls = kpts[LEFT_SHOULDER]
    rs = kpts[RIGHT_SHOULDER]
    lw = kpts[LEFT_WRIST]
    rw = kpts[RIGHT_WRIST]
    le = kpts[LEFT_ELBOW]
    re = kpts[RIGHT_ELBOW]

    def valid(p):
        return p[0] > 5 and p[1] > 5

    sw = abs(ls[0] - rs[0])
    if sw < 10:
        return frozenset()

    v_thresh = sw * 0.40
    side_h = sw * 0.50
    side_v = sw * 0.65
    down_x = sw * 0.85

    mid_shoulder_y = (ls[1] + rs[1]) / 2

    detected = set()

    # UP
    if valid(lw) and valid(rw) and valid(le) and valid(re):
        if ((ls[1] - lw[1]) > v_thresh and (rs[1] - rw[1]) > v_thresh and
                (ls[1] - le[1]) > v_thresh * 0.3 and
                (rs[1] - re[1]) > v_thresh * 0.3):
            detected.add("UP")

    # DOWN
    if valid(lw) and valid(rw) and "UP" not in detected:
        lw_below = lw[1] > mid_shoulder_y + sw * 0.3
        rw_below = rw[1] > mid_shoulder_y + sw * 0.3
        lw_near = abs(lw[0] - ls[0]) < down_x
        rw_near = abs(rw[0] - rs[0]) < down_x
        if lw_below and rw_below and lw_near and rw_near:
            detected.add("DOWN")

    # LEFT
    if valid(rw) and valid(re):
        rw_lateral_x = (rs[0] - rw[0])
        rw_v_diff = abs(rw[1] - rs[1])
        if rw_lateral_x > side_h and rw_v_diff < side_v:
            detected.add("LEFT")

    # RIGHT
    if valid(lw) and valid(le):
        lw_lateral_x = (lw[0] - ls[0])
        lw_v_diff = abs(lw[1] - ls[1])
        if lw_lateral_x > side_h and lw_v_diff < side_v:
            detected.add("RIGHT")

    if "DOWN" in detected and ("LEFT" in detected or "RIGHT" in detected):
        detected.discard("DOWN")

    return frozenset(detected)


# =============================================================================
#  PASOS BASE   (angulos en grados, y hacia abajo: -90 = arriba, 0 = derecha)
# =============================================================================
def _clap_pose(t):
    """Palmas al frente del pecho: o=1 manos abiertas, o=0 manos juntas."""
    o = 1.0 - abs(math.sin(t * 6))
    la, la2 = 120 + 15 * o, -36 - 34 * o
    return dict(la=la, la2=la2, ra=180 - la, ra2=-180 - la2, ll=15, rl=-15,
                by=math.sin(t * 6) * 4)


_ALL_STEPS = {
    "arms_up": {
        "name_es": "Brazos Arriba",
        "name_en": "Arms Up",
        "keys": ["up"],
        "pose": lambda t: dict(la=-125, ra=-55, ll=15, rl=-15, by=math.sin(t * 6) * 5),
    },
    "right_arm": {
        "name_es": "Brazo Der.",
        "name_en": "Right Arm",
        "keys": ["right"],
        "pose": lambda t: dict(la=100, ra=-10, ll=15, rl=-15, by=math.sin(t * 6) * 4),
    },
    "left_arm": {
        "name_es": "Brazo Izq.",
        "name_en": "Left Arm",
        "keys": ["left"],
        "pose": lambda t: dict(la=-170, ra=80, ll=15, rl=-15, by=math.sin(t * 6) * 4),
    },
    "t_pose": {
        "name_es": "Cruz",
        "name_en": "T-Pose",
        "keys": ["left", "right"],
        "pose": lambda t: dict(la=-180, ra=0, ll=15, rl=-15, by=math.sin(t * 5) * 3),
    },
    "jump": {
        "name_es": "Salta",
        "name_en": "Jump",
        "keys": ["up"],
        "pose": lambda t: dict(la=-145, ra=-35, ll=30, rl=-30, by=-abs(math.sin(t * 5)) * 30),
    },
    "clap": {
        "name_es": "Aplaude",
        "name_en": "Clap",
        "keys": ["left", "right"],
        "pose": _clap_pose,
    },
    "spin": {
        "name_es": "Gira",
        "name_en": "Spin",
        "keys": ["right"],
        "pose": lambda t: dict(la=-155, ra=-25, ll=25, rl=-25,
                               by=math.sin(t * 6) * 4, sx=math.sin(t * 4) * 18),
    },
    "hands_head": {
        "name_es": "Manos Cabeza",
        "name_en": "Hands Head",
        "keys": ["up"],
        "pose": lambda t: dict(la=-160, ra=-20, la2=-30, ra2=-150,
                               ll=15, rl=-15, by=math.sin(t * 5) * 4),
    },
    "left_step": {
        "name_es": "Paso Izq.",
        "name_en": "Step Left",
        "keys": ["left"],
        "pose": lambda t: dict(la=-135, ra=60, ll=30, rl=-10, by=math.sin(t * 5) * 6),
    },
    "right_step": {
        "name_es": "Paso Der.",
        "name_en": "Step Right",
        "keys": ["right"],
        "pose": lambda t: dict(la=120, ra=-45, ll=10, rl=-30, by=math.sin(t * 5) * 6),
    },
    "wave": {
        "name_es": "Saluda",
        "name_en": "Wave",
        "keys": ["right"],
        "pose": lambda t: dict(la=100, ra=-45 + math.sin(t * 8) * 20,
                               ll=15, rl=-15, by=math.sin(t * 4) * 3),
    },
    "crouch": {
        "name_es": "Agachate",
        "name_en": "Crouch",
        "keys": ["down"],
        "pose": lambda t: dict(la=-145, ra=-35, ll=45, rl=-45,
                               by=20 + math.sin(t * 6) * 4),
    },
}


def S(name):
    return _ALL_STEPS[name]


# =============================================================================
#  CANCIONES
# =============================================================================
SONGS = [
    {
        "id": "song_1",
        "title": "Balada",
        "figure": (220, 80, 255),
        "routine": [
            S("wave"), S("left_arm"), S("right_arm"), S("hands_head"),
            S("t_pose"), S("wave"), S("arms_up"), S("left_arm"),
            S("right_arm"), S("hands_head"), S("wave"), S("t_pose"),
            S("left_arm"), S("arms_up"), S("wave"), S("right_arm"),
            S("hands_head"), S("t_pose"), S("wave"), S("arms_up"),
            S("left_arm"), S("right_arm"), S("hands_head"), S("wave"),
        ],
    },
    {
        "id": "song_2",
        "title": "Just The Way You Are - Bruno Mars",
        "figure": (0, 200, 255),
        "routine": [
            S("right_step"), S("left_step"), S("wave"), S("right_arm"),
            S("left_arm"), S("clap"), S("right_step"), S("left_step"),
            S("arms_up"), S("wave"), S("right_step"), S("clap"),
            S("left_step"), S("right_arm"), S("left_arm"), S("wave"),
            S("right_step"), S("left_step"), S("clap"), S("arms_up"),
            S("wave"), S("right_arm"), S("clap"), S("left_step"),
        ],
    },
    {
        "id": "song_3",
        "title": "Aram Sam Sam - Luli Pampin",
        "figure": (255, 230, 0),
        "routine": [
            S("clap"), S("arms_up"), S("clap"), S("arms_up"),
            S("clap"), S("right_arm"), S("clap"), S("left_arm"),
            S("clap"), S("arms_up"), S("clap"), S("right_arm"),
            S("clap"), S("left_arm"), S("clap"), S("arms_up"),
            S("clap"), S("arms_up"), S("clap"), S("arms_up"),
        ],
    },
    {
        "id": "song_4",
        "title": "Si Tienes Ganas de Aplaudir - Luli Pampin",
        "figure": (255, 180, 0),
        "routine": [
            S("clap"), S("clap"), S("arms_up"), S("clap"),
            S("clap"), S("right_arm"), S("clap"), S("clap"),
            S("left_arm"), S("clap"), S("clap"), S("arms_up"),
            S("clap"), S("clap"), S("right_step"), S("clap"),
            S("clap"), S("left_step"), S("clap"), S("clap"),
            S("arms_up"), S("clap"), S("clap"), S("arms_up"),
        ],
    },
    {
        "id": "song_5",
        "title": "The Winner Takes It All - ABBA",
        "figure": (80, 180, 255),
        "routine": [
            S("arms_up"), S("right_arm"), S("left_arm"), S("t_pose"),
            S("wave"), S("arms_up"), S("spin"), S("right_step"),
            S("left_step"), S("t_pose"), S("clap"), S("spin"),
            S("arms_up"), S("jump"), S("right_arm"), S("left_arm"),
            S("t_pose"), S("spin"), S("jump"), S("clap"),
            S("arms_up"), S("t_pose"), S("spin"), S("jump"),
        ],
    },
    {
        "id": "song_6",
        "title": "Don't Stop The Music - Rihanna",
        "figure": (255, 80, 150),
        "routine": [
            S("spin"), S("right_step"), S("left_step"), S("jump"),
            S("clap"), S("spin"), S("jump"), S("right_step"),
            S("left_step"), S("spin"), S("clap"), S("jump"),
            S("t_pose"), S("spin"), S("right_step"), S("jump"),
            S("clap"), S("spin"), S("left_step"), S("jump"),
            S("arms_up"), S("spin"), S("clap"), S("jump"),
        ],
    },
    {
        "id": "song_7",
        "title": "La Carretera - Prince Royce",
        "figure": (0, 230, 180),
        "routine": [
            S("right_step"), S("left_step"), S("wave"), S("right_step"),
            S("left_step"), S("wave"), S("spin"), S("right_step"),
            S("left_step"), S("clap"), S("wave"), S("right_step"),
            S("left_step"), S("spin"), S("wave"), S("right_step"),
            S("left_step"), S("clap"), S("wave"), S("arms_up"),
            S("right_step"), S("left_step"), S("wave"), S("spin"),
        ],
    },
    {
        "id": "song_8",
        "title": "La Despedida",
        "figure": (200, 200, 255),
        "routine": [
            S("hands_head"), S("left_arm"), S("right_arm"), S("wave"),
            S("hands_head"), S("t_pose"), S("left_arm"), S("right_arm"),
            S("wave"), S("hands_head"), S("arms_up"), S("wave"),
            S("left_arm"), S("hands_head"), S("right_arm"), S("wave"),
            S("t_pose"), S("hands_head"), S("wave"), S("arms_up"),
            S("left_arm"), S("right_arm"), S("hands_head"), S("wave"),
        ],
    },
    {
        "id": "song_9",
        "title": "Beauty And A Beat - Justin Bieber",
        "figure": (255, 220, 0),
        "routine": [
            S("jump"), S("right_step"), S("left_step"), S("clap"),
            S("jump"), S("spin"), S("right_step"), S("clap"),
            S("left_step"), S("jump"), S("clap"), S("right_step"),
            S("spin"), S("left_step"), S("jump"), S("clap"),
            S("right_step"), S("left_step"), S("jump"), S("spin"),
            S("clap"), S("arms_up"), S("jump"), S("clap"),
        ],
    },
    {
        "id": "song_10",
        "title": "Gotas De Lluvia - Grupo Niche",
        "figure": (255, 200, 80),
        "routine": [
            S("right_step"), S("left_step"), S("right_step"), S("clap"),
            S("spin"), S("right_step"), S("left_step"), S("right_step"),
            S("clap"), S("spin"), S("left_step"), S("right_step"),
            S("left_step"), S("clap"), S("spin"), S("right_step"),
            S("left_step"), S("clap"), S("arms_up"), S("spin"),
            S("right_step"), S("left_step"), S("clap"), S("spin"),
        ],
    },
    {
        "id": "song_11",
        "title": "Rosa Pastel - Belanova",
        "figure": (255, 150, 200),
        "routine": [
            S("wave"), S("right_arm"), S("left_arm"), S("spin"),
            S("wave"), S("hands_head"), S("right_arm"), S("left_arm"),
            S("wave"), S("spin"), S("arms_up"), S("wave"),
            S("right_arm"), S("left_arm"), S("spin"), S("wave"),
            S("hands_head"), S("right_arm"), S("wave"), S("spin"),
            S("arms_up"), S("wave"), S("right_arm"), S("spin"),
        ],
    },
    {
        "id": "song_12",
        "title": "Aitana - SUPERESTRELLA",
        "figure": (255, 80, 180),
        "routine": [
            S("wave"), S("clap"), S("spin"), S("t_pose"),
            S("wave"), S("jump"), S("clap"), S("spin"),
            S("right_step"), S("left_step"), S("wave"), S("clap"),
            S("t_pose"), S("spin"), S("wave"), S("jump"),
            S("clap"), S("right_step"), S("spin"), S("wave"),
            S("t_pose"), S("clap"), S("jump"), S("spin"),
        ],
    },
    {
        "id": "song_13",
        "title": "El Dia De Mi Suerte",
        "figure": (80, 230, 255),
        "routine": [
            S("right_step"), S("left_step"), S("spin"), S("right_step"),
            S("arms_up"), S("left_step"), S("clap"), S("right_step"),
            S("spin"), S("wave"), S("left_step"), S("right_step"),
            S("clap"), S("spin"), S("arms_up"), S("left_step"),
            S("right_step"), S("wave"), S("spin"), S("clap"),
            S("left_step"), S("arms_up"), S("right_step"), S("spin"),
        ],
    },
    {
        "id": "song_14",
        "title": "Iris - Goo Goo Dolls",
        "figure": (200, 230, 255),
        "routine": [
            S("arms_up"), S("t_pose"), S("wave"), S("left_arm"),
            S("right_arm"), S("arms_up"), S("hands_head"), S("t_pose"),
            S("wave"), S("arms_up"), S("left_arm"), S("right_arm"),
            S("t_pose"), S("wave"), S("arms_up"), S("hands_head"),
            S("left_arm"), S("t_pose"), S("right_arm"), S("arms_up"),
            S("wave"), S("t_pose"), S("arms_up"), S("hands_head"),
        ],
    },
]

KEY_MAP = {
    ord('w'): "UP", 82: "UP",
    ord('s'): "DOWN", 84: "DOWN",
    ord('a'): "LEFT", 81: "LEFT",
    ord('d'): "RIGHT", 83: "RIGHT",
}

# =============================================================================
#  CARGA DE FONDOS (opcional: si hay PNG en FONDOS_DIR se mezclan con el neon)
# =============================================================================
_BACKGROUND_IMAGES = []
_BG_LOCK = threading.Lock()


def _load_backgrounds():
    t0 = time.perf_counter()
    loaded = []
    for p in sorted(glob.glob(os.path.join(FONDOS_DIR, "*.png"))):
        img = cv2.imread(p)
        if img is not None:
            loaded.append(cv2.resize(img, (WIN_W, WIN_H), interpolation=cv2.INTER_AREA))
    with _BG_LOCK:
        _BACKGROUND_IMAGES[:] = loaded
    print(f"[Fondos] {len(loaded)} fondos cargados en {time.perf_counter() - t0:.2f}s.")


def _pick_random_bg():
    with _BG_LOCK:
        if _BACKGROUND_IMAGES:
            return random.choice(_BACKGROUND_IMAGES).copy()
    return None


# =============================================================================
#  HELPERS DE DIBUJO
# =============================================================================
def _ascii(s):
    return unicodedata.normalize('NFKD', s).encode('ascii', 'ignore').decode('ascii')


def _shade(c, k):
    return tuple(int(max(0, min(255, v * k))) for v in c)


def _lighten(c, a):
    return tuple(int(min(255, v + a)) for v in c)


def _mix(c1, c2, t):
    return tuple(int(c1[i] * (1 - t) + c2[i] * t) for i in range(3))


def _shift_hue(c, deg):
    px = np.uint8([[list(c)]])
    hsv = cv2.cvtColor(px, cv2.COLOR_BGR2HSV)
    hsv[0, 0, 0] = (int(hsv[0, 0, 0]) + int(deg / 2)) % 180
    out = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]
    return tuple(int(v) for v in out)


def _rr_pts(x1, y1, x2, y2, r):
    r = int(max(0, min(r, (x2 - x1) // 2, (y2 - y1) // 2)))
    if r == 0:
        return np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], np.int32)
    pts = []
    for cx, cy, a0 in ((x2 - r, y1 + r, 270), (x2 - r, y2 - r, 0),
                       (x1 + r, y2 - r, 90), (x1 + r, y1 + r, 180)):
        pts.extend(cv2.ellipse2Poly((cx, cy), (r, r), 0, a0, a0 + 90, 15))
    return np.array(pts, np.int32)


def _rounded_rect(img, pt1, pt2, color, radius=12, thickness=-1, alpha=1.0):
    x1, y1 = pt1
    x2, y2 = pt2
    pts = _rr_pts(x1, y1, x2, y2, radius)
    if alpha >= 1.0:
        if thickness == -1:
            cv2.fillPoly(img, [pts], color, cv2.LINE_AA)
        else:
            cv2.polylines(img, [pts], True, color, thickness, cv2.LINE_AA)
        return
    bx0, by0 = max(0, x1 - 4), max(0, y1 - 4)
    bx1, by1 = min(img.shape[1], x2 + 5), min(img.shape[0], y2 + 5)
    if bx1 <= bx0 or by1 <= by0:
        return
    roi = img[by0:by1, bx0:bx1]
    tmp = roi.copy()
    p = pts - np.array([bx0, by0], np.int32)
    if thickness == -1:
        cv2.fillPoly(tmp, [p], color, cv2.LINE_AA)
    else:
        cv2.polylines(tmp, [p], True, color, thickness, cv2.LINE_AA)
    cv2.addWeighted(tmp, alpha, roi, 1 - alpha, 0, roi)


def _paste(frame, x0, y0, mask, color):
    """Pega `color` (BGR o imagen HxWx3) usando `mask` (uint8, suave) como alpha."""
    h, w = mask.shape[:2]
    fx0, fy0 = max(x0, 0), max(y0, 0)
    fx1, fy1 = min(x0 + w, frame.shape[1]), min(y0 + h, frame.shape[0])
    if fx1 <= fx0 or fy1 <= fy0:
        return
    a = mask[fy0 - y0:fy1 - y0, fx0 - x0:fx1 - x0].astype(np.float32)[..., None] / 255.0
    region = frame[fy0:fy1, fx0:fx1]
    if isinstance(color, np.ndarray):
        col = color[fy0 - y0:fy1 - y0, fx0 - x0:fx1 - x0].astype(np.float32)
    else:
        col = np.array(color, np.float32)
    region[:] = (region.astype(np.float32) * (1.0 - a) + col * a).astype(np.uint8)


def _star(img, cx, cy, r, color, outline=None, thick=2):
    pts = []
    for i in range(10):
        ang = -math.pi / 2 + i * math.pi / 5
        rr = r if i % 2 == 0 else r * 0.45
        pts.append((cx + rr * math.cos(ang), cy + rr * math.sin(ang)))
    pts = np.array(pts, np.int32)
    cv2.fillPoly(img, [pts], color, cv2.LINE_AA)
    if outline is not None:
        cv2.polylines(img, [pts], True, outline, thick, cv2.LINE_AA)


def _step_color(step):
    keys = set(step["keys"])
    if keys == {"up"}:
        return (0, 205, 255)       # dorado
    if keys == {"down"}:
        return (255, 170, 30)      # azul
    if keys == {"left"}:
        return (200, 80, 255)      # rosa
    if keys == {"right"}:
        return (90, 220, 80)       # verde
    return (210, 90, 170)          # violeta (ambos brazos)


def _wrap_text(txt, max_w, scale, thick, font=cv2.FONT_HERSHEY_DUPLEX):
    lines, cur = [], ""
    for word in _ascii(txt).split(' '):
        test = (cur + " " + word).strip()
        (tw, _), _ = cv2.getTextSize(test, font, scale, thick)
        if tw > max_w and cur:
            lines.append(cur)
            cur = word
        else:
            cur = test
    if cur:
        lines.append(cur)
    return lines


def _paint_body(img, cx, cy, pose, color, Sc, extra=0, min_w=2):
    """Dibuja la silueta del coach centrada en (cx, cy). `extra` engorda el trazo (bordes/glow)."""
    la, ra = pose.get("la", 112), pose.get("ra", 68)
    la2, ra2 = pose.get("la2"), pose.get("ra2")
    ll, rl = pose.get("ll", 15), pose.get("rl", -15)

    def pt(dx, dy):
        return (int(round(cx + dx * Sc)), int(round(cy + dy * Sc)))

    w_arm = max(min_w, int(11 * Sc)) + extra
    w_leg = max(min_w, int(14 * Sc)) + extra

    def limb(p1, p2, w):
        cv2.line(img, p1, p2, color, w, cv2.LINE_AA)
        cv2.circle(img, p1, w // 2, color, -1, cv2.LINE_AA)
        cv2.circle(img, p2, w // 2, color, -1, cv2.LINE_AA)

    # piernas separadas (ll>0 abre la izquierda, rl<0 abre la derecha)
    lk, lf = -(9 + ll * 0.35), -(10 + ll * 0.6)
    rk, rf = (9 - rl * 0.35), (10 - rl * 0.6)
    limb(pt(-7, -16), pt(lk, 24), w_leg)
    limb(pt(lk, 24), pt(lf, 64), w_leg)
    limb(pt(lf, 64), pt(lf - 10, 66), max(min_w, int(w_leg * 0.8)))
    limb(pt(7, -16), pt(rk, 24), w_leg)
    limb(pt(rk, 24), pt(rf, 64), w_leg)
    limb(pt(rf, 64), pt(rf + 10, 66), max(min_w, int(w_leg * 0.8)))

    # torso + cuello + cabeza
    torso = np.array([pt(-17, -64), pt(17, -64), pt(11, -14), pt(-11, -14)], np.int32)
    cv2.fillPoly(img, [torso], color, cv2.LINE_AA)
    if extra:
        cv2.polylines(img, [torso], True, color, max(1, extra), cv2.LINE_AA)
    limb(pt(0, -72), pt(0, -62), max(min_w, int(8 * Sc)) + extra)
    cv2.circle(img, pt(0, -89), max(2, int(17 * Sc)) + extra // 2, color, -1, cv2.LINE_AA)

    # brazos (dos segmentos si hay la2/ra2)
    def arm(sx, ang, ang2):
        ax = sx + math.cos(math.radians(ang)) * 44
        ay = -58 + math.sin(math.radians(ang)) * 44
        limb(pt(sx, -58), pt(ax, ay), w_arm)
        ex, ey = ax, ay
        if ang2 is not None:
            ex = ax + math.cos(math.radians(ang2)) * 40
            ey = ay + math.sin(math.radians(ang2)) * 40
            limb(pt(ax, ay), pt(ex, ey), w_arm)
        cv2.circle(img, pt(ex, ey), int(w_arm * 0.62), color, -1, cv2.LINE_AA)

    arm(-13, la, la2)
    arm(13, ra, ra2)


# =============================================================================
#  MUSIC MANAGER (Blindado contra fallos de Pygame)
# =============================================================================
class MusicManager:
    def __init__(self):
        self._available = False
        self._current_song_id = None
        try:
            import pygame
            if not pygame.mixer.get_init():
                pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=512)
            pygame.mixer.music.set_volume(MUSIC_VOLUME)
            self._pygame = pygame
            self._available = True
        except Exception as e:
            print(f"[MusicManager] pygame no disponible: {e}")

    def play(self, song_id):
        if not self._available:
            return
        if self._current_song_id == song_id and self._pygame.mixer.music.get_busy():
            return
        path = SONG_MUSIC.get(song_id, "")
        if not path or not os.path.isfile(path):
            print(f"[MusicManager] No encontrado: {path}")
            self.stop()
            return
        try:
            if self._pygame.mixer.music.get_busy():
                self._pygame.mixer.music.stop()
            self._pygame.mixer.music.load(path)
            self._pygame.mixer.music.set_volume(MUSIC_VOLUME)
            self._pygame.mixer.music.play(loops=-1)
            self._current_song_id = song_id
        except Exception as e:
            print(f"[MusicManager] Error reproduciendo {song_id}: {e}")

    def stop(self):
        if not self._available:
            return
        try:
            if self._pygame.mixer.get_init():
                self._pygame.mixer.music.stop()
        except Exception:
            pass
        self._current_song_id = None

    def quit(self):
        if not self._available:
            return
        try:
            self._pygame.mixer.music.stop()
            self._pygame.mixer.quit()
        except Exception:
            pass


# =============================================================================
#  RENDERER
# =============================================================================
class DanceRenderer:
    SCALE = 1.2

    def __init__(self, w=WIN_W, h=WIN_H):
        self.W = w
        self.H = h
        self.figure_color = C_GREEN
        self.has_theme = False
        self._bg_image = None
        self._u, self._v = np.meshgrid(np.linspace(0, 1, w, dtype=np.float32),
                                       np.linspace(0, 1, h, dtype=np.float32))
        self._title_cache = {}
        self._card_cache = {}
        self._bokeh = self._make_bokeh(11)
        self._bokeh_menu = self._make_bokeh(5, n=22, rmin=3, rmax=11)
        self._bg_menu = self._build_menu_backdrop()
        self._bg_play = self._bg_menu

    # ------------------------------------------------------------ fondos
    def set_theme(self, song_data):
        self.figure_color = song_data["figure"]
        self._bg_image = _pick_random_bg()
        a = _shade(self.figure_color, 0.50)
        b = _shift_hue(_shade(self.figure_color, 0.85), 55)
        self._bg_play = self._build_backdrop(
            a, b, base_img=self._bg_image,
            spots=((ACTIVE_X, FLOOR_Y), (GHOST_X, FLOOR_Y)), floor=True)
        self.has_theme = True

    def _build_backdrop(self, c1, c2, base_img=None, spots=(), floor=False):
        H, W = self.H, self.W
        t = np.clip(self._v * 0.7 + self._u * 0.3, 0, 1)[..., None]
        grad = np.array(c1, np.float32) * (1 - t) + np.array(c2, np.float32) * t
        if base_img is not None:
            grad = grad * 0.40 + base_img.astype(np.float32) * 0.55
        du, dv = (self._u - 0.5) * 2, (self._v - 0.5) * 2
        vign = np.clip(1.0 - 0.55 * (du ** 2 * 0.5 + dv ** 2 * 0.7), 0.35, 1.0)[..., None]
        img = np.clip(grad * vign, 0, 255).astype(np.uint8)

        # haces de luz
        ov = img.copy()
        for i in range(6):
            xt = int(W * (0.08 + 0.17 * i))
            off = int((i - 2.5) * 120)
            poly = np.array([[xt - 16, 0], [xt + 16, 0],
                             [xt + off + 90, H], [xt + off - 90, H]], np.int32)
            cv2.fillPoly(ov, [poly], (255, 255, 255), cv2.LINE_AA)
        cv2.addWeighted(ov, 0.09, img, 0.91, 0, img)

        if floor:
            hy = int(H * 0.70)
            fl = img.copy()
            cv2.rectangle(fl, (0, hy), (W, H), (0, 0, 0), -1)
            cv2.addWeighted(fl, 0.32, img, 0.68, 0, img)
            gl = img.copy()
            col = _lighten(c2, 80)
            for i in range(11):
                x1 = int(W * i / 10)
                cv2.line(gl, (x1, hy), (int((x1 - W / 2) * 2.2 + W / 2), H), col, 1, cv2.LINE_AA)
            for i in range(5):
                fy = hy + int(i * (H - hy) / 4)
                sp = int(i * W * 0.22)
                cv2.line(gl, (-sp, fy), (W + sp, fy), col, 1, cv2.LINE_AA)
            cv2.addWeighted(gl, 0.35, img, 0.65, 0, img)

        for sx, sy in spots:
            cone = img.copy()
            cv2.fillPoly(cone, [np.array([[sx - 20, 0], [sx + 20, 0], [sx + 110, sy], [sx - 110, sy]],
                                         np.int32)], (255, 255, 255), cv2.LINE_AA)
            cv2.addWeighted(cone, 0.10, img, 0.90, 0, img)
            pool = img.copy()
            cv2.ellipse(pool, (sx, sy), (120, 24), 0, 0, 360, (255, 255, 255), -1, cv2.LINE_AA)
            cv2.addWeighted(pool, 0.22, img, 0.78, 0, img)
        return img

    def _build_menu_backdrop(self):
        """Fondo de menu: azul noche -> violeta, estrellas y suelo de rejilla (sin rayos)."""
        H, W = self.H, self.W
        top = np.array((46, 16, 20), np.float32)
        bot = np.array((120, 44, 84), np.float32)
        t = np.clip(self._v, 0, 1)[..., None]
        img = top * (1 - t) + bot * t
        d = np.sqrt(((self._u - 0.5) * 1.5) ** 2 + ((self._v - 0.55) * 1.3) ** 2)
        glow = (np.clip(1.0 - d, 0, 1) ** 2)[..., None]
        img = img + glow * np.array((55, 25, 30), np.float32)
        du, dv = (self._u - 0.5) * 2, (self._v - 0.5) * 2
        vign = np.clip(1.0 - 0.45 * (du ** 2 * 0.5 + dv ** 2 * 0.7), 0.45, 1.0)[..., None]
        img = np.clip(img * vign, 0, 255).astype(np.uint8)

        rng = random.Random(7)
        for _ in range(70):
            x, y = rng.randint(0, W - 1), rng.randint(0, int(H * 0.62))
            b = rng.randint(120, 230)
            cv2.circle(img, (x, y), rng.choice((1, 1, 2)), (b, b, b), -1, cv2.LINE_AA)

        hy = int(H * 0.66)
        grid = img.copy()
        col = (190, 120, 170)
        for i in range(-6, 17):
            x1 = int(W * i / 10)
            cv2.line(grid, (x1, hy), (int((x1 - W / 2) * 3.0 + W / 2), H), col, 1, cv2.LINE_AA)
        for k in range(1, 7):
            fy = hy + int((H - hy) * (k / 6.0) ** 1.8)
            cv2.line(grid, (0, fy), (W, fy), col, 1, cv2.LINE_AA)
        cv2.line(grid, (0, hy), (W, hy), (255, 200, 230), 2, cv2.LINE_AA)
        cv2.addWeighted(grid, 0.30, img, 0.70, 0, img)
        return img

    def _make_bokeh(self, seed, n=16, rmin=8, rmax=26):
        rng = random.Random(seed)
        return [(rng.uniform(0, self.W), rng.uniform(0, self.H), rng.randint(rmin, rmax),
                 rng.uniform(-14, 14), rng.uniform(-22, -6), rng.uniform(0, 6.28))
                for _ in range(n)]

    def draw_background(self, frame, t, play=True):
        themed = play and self.has_theme
        np.copyto(frame, self._bg_play if themed else self._bg_menu)
        if FX_BOKEH:
            if themed:
                base, pts, alpha = _lighten(self.figure_color, 90), self._bokeh, 0.16
            else:
                base, pts, alpha = (255, 215, 170), self._bokeh_menu, 0.10
            ov = frame.copy()
            for i, (x0, y0, r, vx, vy, ph) in enumerate(pts):
                x = int((x0 + vx * t) % (self.W + 60)) - 30
                y = int((y0 + vy * t) % (self.H + 60)) - 30
                rr = int(r * (1 + 0.15 * math.sin(t * 1.5 + ph)))
                cv2.circle(ov, (x, y), rr, base if i % 2 else C_WHITE, -1, cv2.LINE_AA)
            cv2.addWeighted(ov, alpha, frame, 1 - alpha, 0, frame)

    def floor_ring(self, frame, cx, t, alpha_col=(255, 255, 255)):
        rx = int(100 + 6 * math.sin(t * 5))
        cv2.ellipse(frame, (cx, FLOOR_Y), (rx, 20), 0, 0, 360, alpha_col, 2, cv2.LINE_AA)

    # ------------------------------------------------------------ texto
    def text(self, frame, txt, cx, cy, scale=0.9, color=C_WHITE, bold=False,
             align="c", shadow=True):
        txt = _ascii(txt)
        font = cv2.FONT_HERSHEY_DUPLEX if bold else cv2.FONT_HERSHEY_SIMPLEX
        thick = 2 if bold else 1
        (tw, th), _ = cv2.getTextSize(txt, font, scale, thick)
        if align == "c":
            x = int(cx - tw / 2)
        elif align == "r":
            x = int(cx - tw)
        else:
            x = int(cx)
        y = int(cy + th / 2)
        if shadow:
            cv2.putText(frame, txt, (x + 2, y + 2), font, scale, C_BLACK, thick + 2, cv2.LINE_AA)
        cv2.putText(frame, txt, (x, y), font, scale, color, thick, cv2.LINE_AA)

    def text_multiline(self, frame, txt, cx, start_y, scale=0.9, color=C_WHITE, line_spacing=40):
        for i, ln in enumerate(txt.split('\n')):
            self.text(frame, ln, cx, start_y + i * line_spacing, scale, color)

    def title(self, frame, txt, cx, cy, scale=2.0, top=C_GOLD, bot=C_ORANGE, thick=4,
              outline=(60, 20, 70), out_w=5, alpha=1.0):
        """Texto grande con degradado vertical, borde y sombra (estilo logo)."""
        txt = _ascii(txt)
        scale = round(scale * 10) / 10.0
        key = (txt, scale, top, bot, thick, outline, out_w)
        pack = self._title_cache.get(key)
        if pack is None:
            font = cv2.FONT_HERSHEY_DUPLEX
            (tw, th), base = cv2.getTextSize(txt, font, scale, thick)
            pad = out_w + 4
            w, h = tw + pad * 2, th + base + pad * 2
            mask = np.zeros((h, w), np.uint8)
            cv2.putText(mask, txt, (pad, pad + th), font, scale, 255, thick, cv2.LINE_AA)
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (out_w * 2 + 1, out_w * 2 + 1))
            grown = cv2.dilate(mask, k)
            rows = np.clip((np.arange(h) - pad) / max(th, 1), 0, 1).astype(np.float32)[:, None, None]
            grad = np.array(top, np.float32) * (1 - rows) + np.array(bot, np.float32) * rows
            grad = np.repeat(grad, w, axis=1).astype(np.uint8)
            if len(self._title_cache) > 80:
                self._title_cache.clear()
            pack = (mask, grown, grad, w, h)
            self._title_cache[key] = pack
        mask, grown, grad, w, h = pack
        x0, y0 = int(cx - w / 2), int(cy - h / 2)
        target = frame.copy() if alpha < 1.0 else frame
        _paste(target, x0, y0 + 4, grown, (14, 6, 22))
        _paste(target, x0, y0, grown, outline)
        _paste(target, x0, y0, mask, grad)
        if alpha < 1.0:
            cv2.addWeighted(target, alpha, frame, 1 - alpha, 0, frame)

    # ------------------------------------------------------------ coach
    def _draw_figure(self, frame, cx, cy, pose, color, alpha=1.0, scale=None, glow=True):
        Sc = scale or self.SCALE
        cx = int(cx + pose.get("sx", 0))
        cy = int(cy + pose.get("by", 0))
        half = int(140 * Sc) + 24
        x0, y0 = max(0, cx - half), max(0, cy - half)
        x1, y1 = min(self.W, cx + half), min(self.H, cy + half)
        if x1 <= x0 or y1 <= y0:
            return
        roi = frame[y0:y1, x0:x1]
        work = roi.copy() if alpha < 1.0 else roi
        lx, ly = cx - x0, cy - y0
        if glow and FX_GLOW:
            g = np.zeros_like(work)
            _paint_body(g, lx, ly, pose, color, Sc, extra=int(10 * Sc))
            g = cv2.GaussianBlur(g, (0, 0), 9 * Sc)
            cv2.addWeighted(work, 1.0, g, 0.9, 0, work)
        _paint_body(work, lx, ly, pose, (40, 14, 50), Sc, extra=max(3, int(5 * Sc)))
        _paint_body(work, lx, ly, pose, color, Sc)
        if alpha < 1.0:
            cv2.addWeighted(work, alpha, roi, 1 - alpha, 0, roi)

    def draw_pictogram(self, frame, x, y, step, r=30, alpha=1.0, ring=None, miss=False):
        col = _step_color(step)
        pad = r + 10
        x0, y0 = max(0, x - pad), max(0, y - pad)
        x1, y1 = min(self.W, x + pad), min(self.H, y + pad)
        if x1 <= x0 or y1 <= y0:
            return
        roi = frame[y0:y1, x0:x1]
        work = roi.copy() if alpha < 1.0 else roi
        lx, ly = x - x0, y - y0
        cv2.circle(work, (lx, ly + 3), r + 1, (0, 0, 0), -1, cv2.LINE_AA)
        cv2.circle(work, (lx, ly), r, _shade(col, 0.55) if miss else col, -1, cv2.LINE_AA)
        cv2.circle(work, (lx, ly), r, ring or C_WHITE, 3, cv2.LINE_AA)
        sc = r * 0.0092
        pose = step["pose"](0.26)
        _paint_body(work, lx, ly + int(18 * sc), pose, C_WHITE, sc, min_w=3)
        if miss:
            d = int(r * 0.5)
            cv2.line(work, (lx - d, ly - d), (lx + d, ly + d), C_RED, 5, cv2.LINE_AA)
            cv2.line(work, (lx + d, ly - d), (lx - d, ly + d), C_RED, 5, cv2.LINE_AA)
        if alpha < 1.0:
            cv2.addWeighted(work, alpha, roi, 1 - alpha, 0, roi)

    def _draw_arcade_arrow(self, frame, cx, cy, direction, color, alpha=1.0, size=24):
        s = size
        arrow_pts = {
            "up": np.array([[-s // 2, s // 3], [s // 2, s // 3], [s // 2, -s // 5],
                            [s, -s // 5], [0, -s], [-s, -s // 5], [-s // 2, -s // 5]], dtype=np.int32),
            "down": np.array([[-s // 2, -s // 3], [s // 2, -s // 3], [s // 2, s // 5],
                              [s, s // 5], [0, s], [-s, s // 5], [-s // 2, s // 5]], dtype=np.int32),
            "right": np.array([[-s // 3, -s // 2], [-s // 3, s // 2], [s // 5, s // 2],
                               [s // 5, s], [s, 0], [s // 5, -s], [s // 5, -s // 2]], dtype=np.int32),
            "left": np.array([[s // 3, -s // 2], [s // 3, s // 2], [-s // 5, s // 2],
                              [-s // 5, s], [-s, 0], [-s // 5, -s], [-s // 5, -s // 2]], dtype=np.int32),
        }
        pts = arrow_pts.get(direction)
        if pts is None:
            return
        pts = pts + np.array([cx, cy], dtype=np.int32)
        target = frame.copy() if alpha < 1.0 else frame
        cv2.fillPoly(target, [pts], color, cv2.LINE_AA)
        cv2.polylines(target, [pts], True, C_WHITE, 2, cv2.LINE_AA)
        if alpha < 1.0:
            cv2.addWeighted(target, alpha, frame, 1 - alpha, 0, frame)

    def draw_arrows(self, frame, cx, cy, step, alpha=1.0):
        offsets = {"up": (0, -168), "down": (95, 62), "left": (-118, -20), "right": (118, -20)}
        for k in step["keys"]:
            dx, dy = offsets.get(k, (0, -150))
            self._draw_arcade_arrow(frame, cx + dx, cy + dy, k, C_ARROW, alpha)

    # ------------------------------------------------------------ botones
    def draw_button(self, frame, txt, x, y, w, h, bg_color, is_hover=False, scale=0.8):
        r = h // 2
        yy = y - 2 if is_hover else y
        _rounded_rect(frame, (x, yy + 5), (x + w, yy + h + 5), _shade(bg_color, 0.35), radius=r)
        _rounded_rect(frame, (x, yy), (x + w, yy + h), _shade(bg_color, 0.62), radius=r)
        body = _lighten(bg_color, 30) if is_hover else bg_color
        _rounded_rect(frame, (x + 2, yy + 2), (x + w - 2, yy + h - 2), body, radius=r - 2)
        _rounded_rect(frame, (x + r // 2, yy + 5), (x + w - r // 2, yy + h // 2),
                      C_WHITE, radius=max(2, r // 2 - 2), alpha=0.20)
        self.text(frame, txt, x + w // 2, yy + h // 2 - 1, scale, C_WHITE, bold=True)
        return (x, y, w, h)

    # ------------------------------------------------------------ tarjetas del menu
    def card_image(self, song, w, h):
        key = (song["id"], w, h)
        pack = self._card_cache.get(key)
        if pack is None:
            col = song["figure"]
            top, bot = _lighten(col, 5), _shade(col, 0.38)
            tt = np.linspace(0, 1, h, dtype=np.float32)[:, None, None]
            img = np.array(top, np.float32) * (1 - tt) + np.array(bot, np.float32) * tt
            img = np.repeat(img, w, axis=1).astype(np.uint8)
            ov = img.copy()
            cv2.circle(ov, (w - 62, 66), 62, _lighten(col, 70), -1, cv2.LINE_AA)
            cv2.circle(ov, (w - 20, 20), 34, C_WHITE, -1, cv2.LINE_AA)
            cv2.addWeighted(ov, 0.22, img, 0.78, 0, img)
            bar = 64
            img[h - bar:, :] = (img[h - bar:, :].astype(np.float32) * 0.42).astype(np.uint8)
            mask = np.zeros((h, w), np.uint8)
            cv2.fillPoly(mask, [_rr_pts(0, 0, w - 1, h - 1, 18)], 255, cv2.LINE_AA)
            pack = (img, mask)
            self._card_cache[key] = pack
        return pack


# =============================================================================
#  LOGICA DEL JUEGO
# =============================================================================
class DanceGame:
    SONGS_PER_PAGE = 6
    CAM_ANNOUNCE_DUR = 2.0

    def __init__(self, music, pose_camera, is_english=False):
        self.is_english = is_english
        self.music = music
        self.pose_camera = pose_camera
        self.renderer = DanceRenderer(WIN_W, WIN_H)
        self.state = GameState.WELCOME
        self.use_camera = False   # True: solo camara | False: solo teclado

        self._current_song_id = None
        self._routine = []
        self._routine_index = 0
        self._routine_total = 0
        self._steps_done = 0
        self._finishing = False
        self._menu_page = 0

        self.start_delay = 0.0

        self.score = 0
        self.combo = 1
        self.t = 0.0
        self.ui_t = 0.0
        self.state_t = 0.0
        self._last_state = None
        self.queue = []
        self.particles = []
        self._stars_revealed = 0

        self.rating_txt = ""
        self.rating_col = C_GOLD
        self.rating_timer = 0.0
        self.ring_flash = 0.0
        self.ring_col = C_WHITE

        self.stats_attempted = 0
        self.stats_perfect = 0
        self.stats_good = 0
        self.stats_ok = 0

        self.mx = 0
        self.my = 0
        self.btn_lock = threading.Lock()
        self.buttons_rects = {}
        self.temp_rects = {}

        if self.pose_camera:
            self.pose_camera.active_detection = False

    # ------------------------------------------------------------ utilidades
    def _step_name(self, step):
        return step["name_en"] if self.is_english else step["name_es"]

    def _song_title(self):
        for s in SONGS:
            if s["id"] == self._current_song_id:
                return s["title"]
        return ""

    def _perf(self):
        if self._routine_total <= 0:
            return 0.0
        return min(1.0, (self.stats_perfect + 0.7 * self.stats_good + 0.4 * self.stats_ok)
                   / self._routine_total)

    @staticmethod
    def _stars_for(perf):
        return sum(1 for th in STAR_T if perf >= th)

    def _reset_stats(self):
        self.score = 0
        self.combo = 1
        self.t = 0.0
        self.queue = []
        self.particles = []
        self.stats_attempted = 0
        self.stats_perfect = 0
        self.stats_good = 0
        self.stats_ok = 0
        self.start_delay = 0.0
        self._routine_index = 0
        self._steps_done = 0
        self._finishing = False
        self._stars_revealed = 0
        self.rating_timer = 0.0
        self.ring_flash = 0.0
        if self.pose_camera:
            self.pose_camera.active_detection = False

    def play_song(self, song_data):
        self._reset_stats()
        self.renderer.set_theme(song_data)
        self._current_song_id = song_data["id"]
        self._routine = list(song_data["routine"])
        self._routine_total = len(self._routine)
        self._enqueue_next_steps()
        self.state = GameState.PLAYING
        self.start_delay = self.CAM_ANNOUNCE_DUR + 2.0
        self.music.play(self._current_song_id)

    def _enqueue_next_steps(self):
        pending_not_done = sum(1 for q in self.queue if not q["done"])
        slots = max(0, 3 - pending_not_done)
        for _ in range(slots):
            if self._routine_index >= len(self._routine):
                self._finishing = True
                break
            step = self._routine[self._routine_index]
            self._routine_index += 1
            base = min((q["progress"] for q in self.queue), default=0.0)
            self.queue.append({
                "step": step,
                "progress": base - 0.85,
                "done": False,
                "pending": step["keys"][:],
                "hit_flash": 0.0,
                "anim_t": random.random() * 10,
                "done_t": 0.0,
                "result": "",
            })

    def _show_rating(self, txt, col):
        self.rating_txt = txt
        self.rating_col = col
        self.rating_timer = 0.75
        self.ring_flash = 0.45
        self.ring_col = col

    def _spawn_burst(self, x, y, col, n):
        for _ in range(n):
            ang = random.uniform(0, 2 * math.pi)
            sp = random.uniform(70, 260)
            self.particles.append([x, y, math.cos(ang) * sp, math.sin(ang) * sp - 90,
                                   random.uniform(0.5, 0.95),
                                   col if random.random() < 0.6 else C_WHITE,
                                   random.randint(3, 6), random.random() < 0.4])

    def _update_particles(self, dt):
        alive = []
        for p in self.particles:
            p[4] -= dt
            if p[4] <= 0:
                continue
            p[0] += p[2] * dt
            p[1] += p[3] * dt
            p[3] += 420 * dt
            alive.append(p)
        self.particles[:] = alive

    # ------------------------------------------------------------ entrada
    def handle_input(self, direction_or_set):
        if self.state != GameState.PLAYING:
            return
        is_key = isinstance(direction_or_set, str)
        if is_key and self.use_camera:
            return              # modo camara: el teclado se ignora
        if (not is_key) and not self.use_camera:
            return              # modo teclado: la camara se ignora
        active = next((q for q in self.queue
                       if not q["done"] and 0.55 <= q["progress"] <= 1.05), None)
        if not active:
            return

        if isinstance(direction_or_set, str):
            incoming = {direction_or_set.upper()}
        else:
            incoming = {d.upper() for d in direction_or_set}

        if not isinstance(direction_or_set, str):
            target_set = set(k.upper() for k in active["step"]["keys"])
            if incoming != target_set:
                return
            active["pending"] = []
        else:
            pending_set = set(k.upper() for k in active["pending"])
            matched = pending_set & incoming
            if not matched:
                return
            for m in matched:
                for k in list(active["pending"]):
                    if k.upper() == m:
                        active["pending"].remove(k)
                        break

        active["hit_flash"] = 1.0

        if not active["pending"]:
            p = active["progress"]
            self.stats_attempted += 1
            if p < 0.70:
                pts, txt, col = 100 * self.combo, "PERFECT!", C_GOLD
                self.combo += 1
                self.stats_perfect += 1
                active["result"] = "perfect"
                self._spawn_burst(ACTIVE_X, FIG_Y - 40, col, 22)
            elif p < 0.90:
                pts, txt, col = 60 * self.combo, "GOOD", C_LIME
                self.stats_good += 1
                active["result"] = "good"
                self._spawn_burst(ACTIVE_X, FIG_Y - 40, col, 12)
            else:
                pts, txt, col = 20, "OK", C_OKBLUE
                self.combo = max(1, self.combo - 1)
                self.stats_ok += 1
                active["result"] = "ok"
                self._spawn_burst(ACTIVE_X, FIG_Y - 40, col, 6)
            self.score += pts
            self._show_rating(txt, col)
            active["done"] = True
            active["done_t"] = 0.0

    def update(self, dt):
        self.ui_t += dt
        if self.state != self._last_state:
            self._last_state = self.state
            self.state_t = 0.0
            if self.state == GameState.SUMMARY:
                self._stars_revealed = 0
        self.state_t += dt
        self._update_particles(dt)
        self.rating_timer = max(0.0, self.rating_timer - dt)
        self.ring_flash = max(0.0, self.ring_flash - dt)

        if self.state == GameState.SUMMARY:
            final = self._stars_for(self._perf())
            shown = 0
            if self.state_t > 0.6:
                shown = min(final, int((self.state_t - 0.6) / 0.4) + 1)
            if shown > self._stars_revealed:
                self._stars_revealed = shown
                self.renderer_burst_star(shown - 1)

        if self.state != GameState.PLAYING:
            return
        self.t += dt
        if self.start_delay > 0:
            self.start_delay -= dt
            return

        if self.use_camera and self.pose_camera and not self.pose_camera.active_detection:
            self.pose_camera.active_detection = True
            print("[JUEGO] El baile comenzo. Camara activa para puntuar.")

        for q in self.queue:
            q["anim_t"] += dt
            if q["hit_flash"] > 0:
                q["hit_flash"] = max(0.0, q["hit_flash"] - dt * 3.5)
            if q["done"]:
                q["done_t"] += dt
            else:
                q["progress"] += dt / STEP_DUR
        for q in self.queue:
            if not q["done"] and q["progress"] > 1.05:
                self.stats_attempted += 1
                if q["pending"]:
                    self.combo = 1
                    self._show_rating("MISS", C_RED)
                q["done"] = True
                q["done_t"] = 0.0
                q["result"] = "miss"
                q["_was_miss"] = True
                self._steps_done += 1
        for q in self.queue:
            if q["done"] and not q.get("_counted", False):
                q["_counted"] = True
                if not q.get("_was_miss", False):
                    self._steps_done += 1
        self.queue = [q for q in self.queue
                      if not (q["done"] and q["done_t"] > 1.2) and q["progress"] < 2.5]
        if self._finishing:
            if not any(not q["done"] for q in self.queue):
                self.music.stop()
                self.state = GameState.SUMMARY
        else:
            self._enqueue_next_steps()

    def renderer_burst_star(self, i):
        self._spawn_burst(WIN_W // 2 + (i - 2) * 76, 150, C_GOLD, 26)

    # ------------------------------------------------------------ mouse
    def process_mouse(self, x, y, clicked):
        self.mx = x
        self.my = y
        if clicked:
            with self.btn_lock:
                btns = dict(self.buttons_rects)
            for btn_id, rect in btns.items():
                bx, by, bw, bh = rect
                if bx <= x <= bx + bw and by <= y <= by + bh:
                    self._handle_button_click(btn_id)
                    return
            if x < 60 and y < 60:
                if self.state == GameState.PLAYING:
                    self.music.stop()
                    self.state = GameState.SUMMARY
                elif self.state in [GameState.WELCOME, GameState.MENU]:
                    self.state = GameState.CONFIRM_EXIT
                elif self.state == GameState.MODE_SELECT:
                    self.state = GameState.WELCOME

    def _handle_button_click(self, btn_id):
        if btn_id == "btn_start_welcome":
            self.state = GameState.MODE_SELECT
        elif btn_id == "btn_mode_cam":
            self.use_camera = True
            self._apply_input_mode()
            self.state = GameState.MENU
        elif btn_id == "btn_mode_key":
            self.use_camera = False
            self._apply_input_mode()
            self.state = GameState.MENU
        elif btn_id == "btn_menu_mode":
            self.state = GameState.MODE_SELECT
        elif btn_id.startswith("btn_song_"):
            idx = int(btn_id.split("_")[2])
            self.play_song(SONGS[idx])
        elif btn_id == "btn_menu_next":
            max_page = (len(SONGS) - 1) // self.SONGS_PER_PAGE
            self._menu_page = min(self._menu_page + 1, max_page)
        elif btn_id == "btn_menu_prev":
            self._menu_page = max(self._menu_page - 1, 0)
        elif btn_id == "btn_summary_menu":
            self.music.stop()
            self.state = GameState.MENU
        elif btn_id == "btn_summary_restart":
            song_data = next(s for s in SONGS if s["id"] == self._current_song_id)
            self.play_song(song_data)
        elif btn_id == "btn_confirm_yes":
            self.music.stop()
            self.state = GameState.EXITED
        elif btn_id == "btn_confirm_no":
            self.state = GameState.MENU

    def _apply_input_mode(self):
        """Camara: solo pose (teclado ignorado). Sin camara: solo teclado (YOLO no se carga)."""
        pc = self.pose_camera
        if pc is None:
            return
        if self.use_camera:
            pc.enabled = True
            pc.start()          # idempotente; carga YOLO en su hilo la primera vez
        else:
            pc.enabled = False
            pc.active_detection = False

    def _is_hovered(self, rect):
        bx, by, bw, bh = rect
        return bx <= self.mx <= bx + bw and by <= self.my <= by + bh

    # ------------------------------------------------------------ render general
    def render(self):
        self.temp_rects = {}
        frame = np.zeros((WIN_H, WIN_W, 3), dtype=np.uint8)
        if self.state == GameState.WELCOME:
            self._render_welcome(frame)
        elif self.state == GameState.MENU:
            self._render_menu(frame)
        elif self.state == GameState.PLAYING:
            self._render_playing(frame)
        elif self.state == GameState.SUMMARY:
            self._render_summary(frame)
        elif self.state == GameState.CONFIRM_EXIT:
            self._render_confirm_exit(frame)
        elif self.state == GameState.MODE_SELECT:
            self._render_mode_select(frame)

        if self.state in [GameState.WELCOME, GameState.MENU, GameState.PLAYING,
                          GameState.MODE_SELECT]:
            self._draw_close_btn(frame)

        with self.btn_lock:
            self.buttons_rects = self.temp_rects
        return frame

    def _draw_close_btn(self, frame):
        c = (30, 32)
        cv2.circle(frame, (c[0], c[1] + 3), 20, (20, 10, 60), -1, cv2.LINE_AA)
        cv2.circle(frame, c, 20, (70, 70, 225), -1, cv2.LINE_AA)
        cv2.circle(frame, c, 20, C_WHITE, 2, cv2.LINE_AA)
        cv2.line(frame, (23, 25), (37, 39), C_WHITE, 3, cv2.LINE_AA)
        cv2.line(frame, (37, 25), (23, 39), C_WHITE, 3, cv2.LINE_AA)

    def _draw_particles(self, frame):
        for x, y, _, _, life, col, r, is_star in self.particles:
            rr = max(1, int(r * min(1.0, life / 0.4)))
            if is_star:
                _star(frame, int(x), int(y), rr + 3, col)
            else:
                cv2.circle(frame, (int(x), int(y)), rr, col, -1, cv2.LINE_AA)

    # ------------------------------------------------------------ BIENVENIDA
    def _render_welcome(self, frame):
        R = self.renderer
        en = self.is_english
        R.draw_background(frame, self.ui_t, play=False)

        R.title(frame, "YAREN DANCE", WIN_W // 2, 92, 2.6, top=(120, 240, 255), bot=C_ORANGE,
                thick=5, out_w=6)
        desc = ("Follow the dancer's moves to the music.\nEach song has its own routine." if en else
                "Sigue los movimientos del bailarin al ritmo de la musica.\n"
                "Cada cancion tiene su propia coreografia.")
        R.text_multiline(frame, desc, WIN_W / 2, 150, 0.62, UI_TEXT, line_spacing=26)

        trio = (("wave", SONGS[0]["figure"], ACTIVE_X - 60, 0.0),
                ("jump", SONGS[1]["figure"], WIN_W // 2, 1.3),
                ("clap", SONGS[3]["figure"], WIN_W - ACTIVE_X + 60, 2.1))
        for name, col, x, ph in trio:
            pose = S(name)["pose"](self.ui_t + ph)
            cv2.ellipse(frame, (x, 378), (54, 9), 0, 0, 360, (0, 0, 0), -1, cv2.LINE_AA)
            R._draw_figure(frame, x, 320, pose, col, 1.0, scale=0.8)

        bw, bh = 250, 58
        pulse = int(3 * math.sin(self.ui_t * 5))
        bx, by = WIN_W // 2 - bw // 2 - pulse, WIN_H - 86 - pulse // 2
        hov = self._is_hovered((bx, by, bw, bh))
        self.temp_rects["btn_start_welcome"] = R.draw_button(
            frame, "START" if en else "INICIAR", bx, by, bw + 2 * pulse, bh + pulse, C_PINK, hov, 1.0)
        R.text(frame, "W A S D / arrows = test keys" if en else "W A S D / flechas = teclas de prueba",
               WIN_W / 2, WIN_H - 12, 0.42, UI_DIM)

    # ------------------------------------------------------------ MENU
    def _render_menu(self, frame):
        R = self.renderer
        en = self.is_english
        R.draw_background(frame, self.ui_t, play=False)

        total_pages = (len(SONGS) - 1) // self.SONGS_PER_PAGE + 1
        R.title(frame, "SONG MENU" if en else "MENU DE CANCIONES", WIN_W // 2, 38, 1.1,
                top=(140, 245, 255), bot=C_ORANGE, thick=2, out_w=4)
        R.text(frame, f"{self._menu_page + 1}/{total_pages}", WIN_W - 48, 38, 0.65, UI_TEXT, bold=True)
        mode_txt = (("CAMERA" if en else "CAMARA") if self.use_camera
                    else ("KEYBOARD" if en else "TECLADO"))
        self.temp_rects["btn_menu_mode"] = R.draw_button(
            frame, mode_txt, 62, 20, 126, 36, (150, 80, 60),
            self._is_hovered((62, 20, 126, 36)), 0.5)

        cw, ch, gap = 236, 156, 14
        sx0 = (WIN_W - (3 * cw + 2 * gap)) // 2
        sy0 = 78
        start_idx = self._menu_page * self.SONGS_PER_PAGE
        for i, song in enumerate(SONGS[start_idx:start_idx + self.SONGS_PER_PAGE]):
            gi = start_idx + i
            x = sx0 + (i % 3) * (cw + gap)
            y = sy0 + (i // 3) * (ch + gap)
            hov = self._is_hovered((x, y, cw, ch))
            yy = y - 5 if hov else y
            img, mask = R.card_image(song, cw, ch)
            _rounded_rect(frame, (x + 3, yy + 7), (x + cw + 3, yy + ch + 7), C_BLACK, radius=18, alpha=0.40)
            _paste(frame, x, yy, mask, img)
            _rounded_rect(frame, (x, yy), (x + cw - 1, yy + ch - 1),
                          C_WHITE if hov else _lighten(song["figure"], 70), radius=18,
                          thickness=3 if hov else 2)

            # mini coach
            pose = dict(S_pose_for_card(song, self.ui_t if hov else 0.26))
            _paint_body(frame, x + cw - 52, yy + 56, pose, (40, 14, 50), 0.42, extra=3)
            _paint_body(frame, x + cw - 52, yy + 56, pose, C_WHITE, 0.42, min_w=3)

            # numero
            cv2.circle(frame, (x + 30, yy + 30), 20, (30, 14, 40), -1, cv2.LINE_AA)
            cv2.circle(frame, (x + 30, yy + 30), 20, C_WHITE, 2, cv2.LINE_AA)
            R.text(frame, f"{gi + 1}", x + 30, yy + 30, 0.7, C_WHITE, bold=True, shadow=False)

            # titulo / artista
            parts = song["title"].split(" - ", 1)
            name = parts[0]
            artist = parts[1] if len(parts) > 1 else ""
            lines = _wrap_text(name, cw - 22, 0.52, 1)[:2]
            ty = yy + ch - 58 + (8 if len(lines) == 1 and not artist else 0)
            for k, ln in enumerate(lines):
                R.text(frame, ln, x + 12, ty + 10 + k * 18, 0.52, C_WHITE, bold=False, align="l")
            if artist:
                R.text(frame, artist, x + 12, yy + ch - 12, 0.42, _lighten(song["figure"], 90), align="l")
            self.temp_rects[f"btn_song_{gi}"] = (x, y, cw, ch)

        nav_y, nbw, nbh = 428, 150, 40
        max_page = total_pages - 1
        if self._menu_page > 0:
            px = 190
            self.temp_rects["btn_menu_prev"] = R.draw_button(
                frame, "< PREV" if en else "< ANTERIOR", px, nav_y, nbw, nbh, (120, 60, 160),
                self._is_hovered((px, nav_y, nbw, nbh)), 0.7)
        if self._menu_page < max_page:
            nx = WIN_W - 190 - nbw
            self.temp_rects["btn_menu_next"] = R.draw_button(
                frame, "NEXT >" if en else "SIGUIENTE >", nx, nav_y, nbw, nbh, C_PINK,
                self._is_hovered((nx, nav_y, nbw, nbh)), 0.7)
        for p in range(total_pages):
            dx = WIN_W // 2 + int((p - (total_pages - 1) / 2) * 20)
            cv2.circle(frame, (dx, nav_y + 20), 6, C_WHITE if p == self._menu_page else (140, 110, 160),
                       -1, cv2.LINE_AA)

    # ------------------------------------------------------------ MODO DE JUEGO
    def _render_mode_select(self, frame):
        R = self.renderer
        en = self.is_english
        R.draw_background(frame, self.ui_t, play=False)
        R.title(frame, "HOW DO YOU WANT TO PLAY?" if en else "COMO QUIERES JUGAR?",
                WIN_W // 2, 52, 1.1, top=(140, 245, 255), bot=C_ORANGE, thick=2, out_w=4)

        cw, ch, gap = 300, 250, 30
        x0 = (WIN_W - (2 * cw + gap)) // 2
        y0 = 100
        cards = (
            ("btn_mode_cam", (255, 190, 40), "WITH CAMERA" if en else "CON CAMARA",
             "Move your body to dance.\nKeyboard is disabled." if en else
             "Mueve tu cuerpo para bailar.\nEl teclado se desactiva."),
            ("btn_mode_key", (190, 80, 240), "NO CAMERA" if en else "SIN CAMARA",
             "Play with the keyboard.\nCamera is ignored." if en else
             "Juega con el teclado.\nLa camara se ignora."),
        )
        dark = (28, 16, 46)
        for i, (bid, col, label, desc) in enumerate(cards):
            x = x0 + i * (cw + gap)
            hov = self._is_hovered((x, y0, cw, ch))
            y = y0 - 5 if hov else y0
            cx = x + cw // 2
            _rounded_rect(frame, (x + 3, y + 7), (x + cw + 3, y + ch + 7), C_BLACK,
                          radius=22, alpha=0.40)
            _rounded_rect(frame, (x, y), (x + cw, y + ch), dark, radius=22, alpha=0.90)
            _rounded_rect(frame, (x, y), (x + cw, y + ch), C_WHITE if hov else col,
                          radius=22, thickness=3)
            cy = y + 82
            if i == 0:
                _rounded_rect(frame, (cx - 46, cy - 26), (cx + 46, cy + 30), col, radius=10)
                _rounded_rect(frame, (cx - 20, cy - 38), (cx + 14, cy - 24), col, radius=4)
                cv2.circle(frame, (cx, cy + 2), 19, dark, -1, cv2.LINE_AA)
                cv2.circle(frame, (cx, cy + 2), 19, C_WHITE, 3, cv2.LINE_AA)
                cv2.circle(frame, (cx, cy + 2), 8, col, -1, cv2.LINE_AA)
            else:
                for kx, ky, kc in ((cx, cy - 22, "W"), (cx - 40, cy + 18, "A"),
                                   (cx, cy + 18, "S"), (cx + 40, cy + 18, "D")):
                    _rounded_rect(frame, (kx - 17, ky - 17), (kx + 17, ky + 17), col, radius=6)
                    R.text(frame, kc, kx, ky, 0.6, C_WHITE, bold=True, shadow=False)
            R.text(frame, label, cx, y + 168, 0.85, C_WHITE, bold=True)
            R.text_multiline(frame, desc, cx, y + 198, 0.5, UI_DIM, line_spacing=22)
            self.temp_rects[bid] = (x, y0, cw, ch)

        R.text(frame, "You can change this later from the song menu." if en else
               "Puedes cambiarlo despues desde el menu de canciones.",
               WIN_W / 2, 420, 0.5, UI_DIM)

    # ------------------------------------------------------------ HUD / TIMELINE
    def _draw_hud(self, frame):
        R = self.renderer
        en = self.is_english
        _rounded_rect(frame, (-20, -20), (WIN_W + 20, 60), (12, 6, 26), radius=20, alpha=0.62)
        cv2.line(frame, (0, 60), (WIN_W, 60), _lighten(R.figure_color, 20), 2, cv2.LINE_AA)

        # puntaje + combo
        R.text(frame, "SCORE", 66, 14, 0.4, UI_DIM, align="l", shadow=False)
        R.text(frame, f"{self.score}", 66, 38, 0.95, C_WHITE, bold=True, align="l")
        if self.combo > 1:
            _rounded_rect(frame, (172, 24), (222, 50), (30, 150, 20), radius=12)
            R.text(frame, f"x{self.combo}", 197, 37, 0.62, C_WHITE, bold=True, shadow=False)

        # titulo + barra de estrellas
        bx, bw, by = 270, 380, 40
        title = _ascii(self._song_title()).split(" - ")[0]
        R.text(frame, title, bx + bw // 2, 14, 0.5, UI_DIM, shadow=False)
        perf = self._perf()
        _rounded_rect(frame, (bx - 6, by - 8), (bx + bw + 6, by + 8), (40, 28, 60), radius=8)
        fw = int(bw * perf)
        if fw > 3:
            _rounded_rect(frame, (bx - 4, by - 6), (bx - 4 + fw, by + 6), _lighten(R.figure_color, 40), radius=6)
        for th in STAR_T:
            lit = perf >= th
            _star(frame, int(bx + bw * th), by, 12 if lit else 10,
                  C_GOLD if lit else (95, 80, 120), outline=C_WHITE if lit else (140, 120, 160))

        R.text(frame, f"{min(self._steps_done, self._routine_total)}/{self._routine_total}",
               WIN_W - 14, 38, 0.6, UI_TEXT, bold=True, align="r")

    def _draw_timeline(self, frame):
        R = self.renderer
        _rounded_rect(frame, (10, STRIP_Y0), (WIN_W - 10, STRIP_Y1), (12, 6, 26), radius=28, alpha=0.66)
        for x in range(TARGET_X + 54, WIN_W - 30, 20):
            cv2.line(frame, (x, STRIP_CY), (x + 9, STRIP_CY), (140, 120, 170), 2, cv2.LINE_AA)

        pulse = 3 * math.sin(self.ui_t * 6)
        cv2.circle(frame, (TARGET_X, STRIP_CY), int(41 + pulse), C_WHITE, 3, cv2.LINE_AA)
        if self.ring_flash > 0:
            k = 1 - self.ring_flash / 0.45
            cv2.circle(frame, (TARGET_X, STRIP_CY), int(41 + 34 * k), _shade(self.ring_col, 1 - k), 4,
                       cv2.LINE_AA)

        items = []
        for q in self.queue:
            x = TARGET_X + (P_NOW - q["progress"]) * PX_PER_PROG
            if -60 < x < WIN_W + 60:
                items.append((x, q))
        items.sort(key=lambda a: -a[0])
        for x, q in items:
            step = q["step"]
            if q["done"]:
                dt_ = q["done_t"]
                fade = max(0.0, 1 - dt_ / 0.5)
                if fade <= 0:
                    continue
                if q["result"] == "miss":
                    R.draw_pictogram(frame, int(x), STRIP_CY, step, int(30 * (1 - 0.2 * dt_)),
                                     fade * 0.85, miss=True)
                else:
                    col = {"perfect": C_GOLD, "good": C_LIME}.get(q["result"], C_OKBLUE)
                    R.draw_pictogram(frame, int(x), STRIP_CY, step, int(30 * (1 + dt_ * 1.4)), fade, ring=col)
            else:
                near = max(0.0, 1 - abs(x - TARGET_X) / 140)
                R.draw_pictogram(frame, int(x), STRIP_CY, step, int(30 * (1 + 0.22 * near)))

    # ------------------------------------------------------------ JUEGO
    def _render_playing(self, frame):
        R = self.renderer
        en = self.is_english
        R.draw_background(frame, self.ui_t, play=True)
        R.floor_ring(frame, ACTIVE_X, self.ui_t)

        pending = sorted([q for q in self.queue if not q["done"]],
                         key=lambda q: q["progress"], reverse=True)
        top = pending[0] if pending else None
        target_q = top if (top and 0.55 <= top["progress"] <= 1.05) else None
        upcoming_q = next((q for q in pending if q["progress"] < 0.55), None)
        counting = self.start_delay > 0

        # ghost: vista previa del siguiente paso
        if upcoming_q and not counting:
            prog = upcoming_q["progress"]
            alpha = max(0.0, min((prog + 0.5) / 1.05, 1.0)) * 0.5
            if alpha > 0.02:
                R.floor_ring(frame, GHOST_X, self.ui_t, (170, 150, 190))
                pose = upcoming_q["step"]["pose"](upcoming_q["anim_t"])
                R._draw_figure(frame, GHOST_X, FIG_Y, pose, (220, 190, 140), alpha, scale=1.1, glow=False)
                R.draw_arrows(frame, GHOST_X, FIG_Y, upcoming_q["step"], alpha * 0.9)
                tag_a = min(1.0, alpha * 2)
                _rounded_rect(frame, (GHOST_X - 44, 74), (GHOST_X + 44, 98), (30, 20, 50), radius=12,
                              alpha=0.7 * tag_a)
                R.text(frame, "NEXT" if en else "SIGUIENTE", GHOST_X, 86, 0.5,
                       _shade(C_WHITE, tag_a), shadow=False)
                R.text(frame, self._step_name(upcoming_q["step"]), GHOST_X, FIG_Y + 115, 0.7,
                       _shade(UI_TEXT, tag_a), bold=True)

        # coach
        pose_q = None
        if target_q and not counting:
            pose_q = target_q
        elif not counting:
            flashing = [q for q in self.queue if q["done"] and q["hit_flash"] > 0]
            if flashing:
                pose_q = max(flashing, key=lambda q: q["hit_flash"])
        if pose_q:
            pose = pose_q["step"]["pose"](pose_q["anim_t"])
            flash = pose_q["hit_flash"]
        else:
            s = self.ui_t
            pose = dict(la=108 + 8 * math.sin(s * 3), ra=72 - 8 * math.sin(s * 3), ll=15, rl=-15,
                        by=math.sin(s * 5) * 4, sx=math.sin(s * 2.5) * 6)
            flash = 0.0
        fc = R.figure_color
        col = _mix(fc, (255, 255, 200), flash) if flash > 0 else fc
        R._draw_figure(frame, ACTIVE_X, FIG_Y, pose, col, 1.0)
        if target_q and not counting:
            R.draw_arrows(frame, ACTIVE_X, FIG_Y, target_q["step"], 0.95)
            name = self._step_name(target_q["step"])
            _rounded_rect(frame, (ACTIVE_X - 112, FIG_Y + 100), (ACTIVE_X + 112, FIG_Y + 130),
                          (30, 20, 50), radius=15, alpha=0.78)
            R.text(frame, name, ACTIVE_X, FIG_Y + 115, 0.72, C_WHITE, bold=True)

        self._draw_particles(frame)

        # calificacion
        if self.rating_timer > 0:
            age = 0.75 - self.rating_timer
            if age < 0.10:
                k = 0.6 + (age / 0.10) * 0.6
            elif age < 0.20:
                k = 1.2 - (age - 0.10) / 0.10 * 0.2
            else:
                k = 1.0
            fade = min(1.0, self.rating_timer / 0.25)
            R.title(frame, self.rating_txt, WIN_W // 2, 150, 1.7 * k, top=self.rating_col,
                    bot=_shade(self.rating_col, 0.6), thick=4, out_w=5, alpha=fade)

        self._draw_hud(frame)
        self._draw_timeline(frame)

        # fases de arranque
        if self.start_delay > self.CAM_ANNOUNCE_DUR:
            cam_ok = (self.pose_camera.camera_ok and self.pose_camera.model_ready) if self.pose_camera else False
            if not self.use_camera:
                cam_ok = True
            _rounded_rect(frame, (WIN_W // 2 - 260, 120), (WIN_W // 2 + 260, 300), (12, 6, 26),
                          radius=28, alpha=0.84)
            dot = (80, 220, 90) if cam_ok else (60, 190, 255)
            cv2.circle(frame, (WIN_W // 2 - 150, 162), 8, dot, -1, cv2.LINE_AA)
            if not self.use_camera:
                msg = "KEYBOARD MODE" if en else "MODO TECLADO"
            elif cam_ok:
                msg = "CAMERA READY" if en else "CAMARA LISTA"
            else:
                msg = "CONNECTING CAMERA..." if en else "CONECTANDO CAMARA..."
            R.text(frame, msg, WIN_W // 2 + 10, 162, 0.7, C_WHITE, bold=True)
            R.title(frame, "DANCE!" if en else "BAILA!", WIN_W // 2, 238, 2.4,
                    top=(140, 250, 255), bot=C_ORANGE, thick=5, out_w=6)
        elif counting:
            n = max(1, int(math.ceil(self.start_delay)))
            frac = n - self.start_delay
            R.text(frame, "GET READY!" if en else "PREPARATE!", WIN_W // 2, 112, 1.0, C_GOLD, bold=True)
            R.title(frame, str(n), WIN_W // 2, 230, 5.0 * (1.3 - 0.3 * frac), top=C_WHITE,
                    bot=C_GOLD, thick=8, out_w=8)

    # ------------------------------------------------------------ RESUMEN
    def _rating_word(self, stars):
        es = ["INTENTA OTRA VEZ", "SIGUE ASI!", "BIEN!", "GENIAL!", "INCREIBLE!", "SUPERESTRELLA!"]
        en = ["TRY AGAIN", "KEEP GOING!", "NICE!", "GREAT!", "AMAZING!", "SUPERSTAR!"]
        return (en if self.is_english else es)[stars]

    def _render_summary(self, frame):
        R = self.renderer
        en = self.is_english
        R.draw_background(frame, self.ui_t, play=True)

        att = self.stats_attempted
        misses = att - self.stats_perfect - self.stats_good - self.stats_ok
        sr = int((self.stats_perfect + self.stats_good + self.stats_ok) / att * 100) if att else 0
        pr = int(self.stats_perfect / att * 100) if att else 0
        final_stars = self._stars_for(self._perf())

        _rounded_rect(frame, (60, 20), (WIN_W - 60, WIN_H - 28), (14, 8, 30), radius=30, alpha=0.80)
        _rounded_rect(frame, (60, 20), (WIN_W - 60, WIN_H - 28), _lighten(R.figure_color, 30),
                      radius=30, thickness=3)

        R.text(frame, _ascii(self._song_title()), WIN_W / 2, 44, 0.6, UI_DIM, shadow=False)
        word_col = C_GOLD if final_stars >= 3 else (210, 200, 255)
        R.title(frame, self._rating_word(final_stars), WIN_W // 2, 84, 1.4, top=word_col,
                bot=_shade(word_col, 0.6), thick=3, out_w=4)

        for i in range(5):
            x = WIN_W // 2 + (i - 2) * 76
            if i < self._stars_revealed:
                k = min(1.0, (self.state_t - 0.6 - i * 0.4) / 0.25)
                _star(frame, x, 150, int(30 * (1.35 - 0.35 * max(0.0, k))), C_GOLD, outline=C_WHITE, thick=3)
            else:
                _star(frame, x, 150, 28, (70, 56, 96), outline=(120, 100, 150), thick=2)

        shown = int(self.score * min(1.0, max(0.0, (self.state_t - 0.2) / 1.2)))
        R.title(frame, f"{shown}", WIN_W // 2, 214, 1.7, top=C_WHITE, bot=(200, 190, 255),
                thick=3, out_w=4)
        R.text(frame, "SCORE" if en else "PUNTAJE", WIN_W // 2, 186, 0.5, UI_DIM, shadow=False)

        line = (f"Success {sr}%   |   Perfect {pr}%" if en else f"Exito {sr}%   |   Perfectos {pr}%")
        R.text(frame, line, WIN_W / 2, 252, 0.6, UI_TEXT)

        chips = (("PERFECT", self.stats_perfect, C_GOLD),
                 ("GOOD" if en else "BIEN", self.stats_good, C_LIME),
                 ("OK", self.stats_ok, C_OKBLUE),
                 ("MISS", misses, C_RED))
        cw, chh, g = 148, 40, 12
        x0 = (WIN_W - (4 * cw + 3 * g)) // 2
        for i, (lbl, val, col) in enumerate(chips):
            x = x0 + i * (cw + g)
            _rounded_rect(frame, (x, 278), (x + cw, 278 + chh), _shade(col, 0.28), radius=20)
            _rounded_rect(frame, (x, 278), (x + cw, 278 + chh), col, radius=20, thickness=2)
            R.text(frame, f"{lbl} {val}", x + cw // 2, 278 + chh // 2, 0.58, C_WHITE, bold=True)

        self._draw_particles(frame)

        bw, bh, gap = 220, 52, 28
        bx1 = WIN_W // 2 - bw - gap // 2
        bx2 = WIN_W // 2 + gap // 2
        by = 356
        self.temp_rects["btn_summary_menu"] = R.draw_button(
            frame, "MENU", bx1, by, bw, bh, (140, 70, 120), self._is_hovered((bx1, by, bw, bh)), 0.85)
        self.temp_rects["btn_summary_restart"] = R.draw_button(
            frame, "PLAY AGAIN" if en else "REPETIR", bx2, by, bw, bh, C_PINK,
            self._is_hovered((bx2, by, bw, bh)), 0.85)

    # ------------------------------------------------------------ CONFIRMAR SALIDA
    def _render_confirm_exit(self, frame):
        R = self.renderer
        en = self.is_english
        R.draw_background(frame, self.ui_t, play=False)
        frame[:] = cv2.convertScaleAbs(frame, alpha=0.4)
        _rounded_rect(frame, (WIN_W // 2 - 270, 140), (WIN_W // 2 + 270, WIN_H - 140), (20, 12, 40),
                      radius=30, alpha=0.95)
        _rounded_rect(frame, (WIN_W // 2 - 270, 140), (WIN_W // 2 + 270, WIN_H - 140), (200, 90, 230),
                      radius=30, thickness=3)
        R.title(frame, "Exit Yaren Dance?" if en else "Salir de Yaren Dance?", WIN_W // 2, 212, 1.1,
                top=C_WHITE, bot=(210, 190, 255), thick=2, out_w=3)
        bw, bh, gap = 150, 52, 32
        bx1 = WIN_W // 2 - bw - gap // 2
        bx2 = WIN_W // 2 + gap // 2
        by = 262
        self.temp_rects["btn_confirm_yes"] = R.draw_button(
            frame, "YES" if en else "SI", bx1, by, bw, bh, (50, 50, 210), self._is_hovered((bx1, by, bw, bh)), 0.9)
        self.temp_rects["btn_confirm_no"] = R.draw_button(
            frame, "NO", bx2, by, bw, bh, (80, 190, 70), self._is_hovered((bx2, by, bw, bh)), 0.9)


def S_pose_for_card(song, t):
    """Pose del primer paso de la coreografia para la mini silueta de la tarjeta."""
    return song["routine"][0]["pose"](t)


def _splash(msg):
    f = np.full((WIN_H, WIN_W, 3), UI_BG, np.uint8)
    font = cv2.FONT_HERSHEY_DUPLEX
    t1 = "YAREN DANCE"
    (tw, _), _ = cv2.getTextSize(t1, font, 1.8, 4)
    cv2.putText(f, t1, ((WIN_W - tw) // 2, WIN_H // 2 - 10), font, 1.8, C_GOLD, 4, cv2.LINE_AA)
    (tw, _), _ = cv2.getTextSize(msg, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 1)
    cv2.putText(f, msg, ((WIN_W - tw) // 2, WIN_H // 2 + 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, UI_DIM, 1, cv2.LINE_AA)
    return f


# =============================================================================
#  POSECAMERA -- recibe frames desde el nodo via callback, sin cerrarse abruptamente
# =============================================================================
class PoseCamera:
    def __init__(self, model_path=YOLO_MODEL, on_pose_confirmed=None,
                 infer_size=CAM_INFER_SIZE):
        self.model_path = model_path
        self.on_pose_confirmed = on_pose_confirmed
        self.infer_size = infer_size
        self.current_pose = frozenset()
        self.camera_ok = False
        self.model_ready = False      # True cuando YOLO cargo (o fallo y queda modo teclado)
        self.enabled = False          # lo activa DanceGame._apply_input_mode
        self._running = False
        self.active_detection = False
        self._thread = None
        self._lock = threading.Lock()
        self.model = None
        self._latest_frame = None
        self._frame_lock = threading.Lock()

    def _load_model(self):
        t0 = time.perf_counter()
        try:
            if not os.path.exists(self.model_path):
                print("[PoseCamera] Modelo no encontrado. Modo solo teclado.")
                return
            from ultralytics import YOLO
            _log_t("import ultralytics listo")
            self.model = YOLO(self.model_path)
            _log_t("modelo YOLO cargado")
            dummy = np.zeros((self.infer_size, self.infer_size, 3), dtype=np.uint8)
            self.model(dummy, imgsz=self.infer_size, verbose=False)
            print(f"[PoseCamera] YOLO listo en {time.perf_counter() - t0:.2f}s.")
        except Exception as e:
            print(f"[PoseCamera] Error cargando modelo: {e}. Modo solo teclado.")
            self.model = None
        finally:
            self.model_ready = True

    def push_frame(self, frame: np.ndarray):
        """Llamado desde el image_callback del nodo con el frame BGR ya convertido."""
        with self._frame_lock:
            self._latest_frame = frame
        if not self.camera_ok:
            self.camera_ok = True

    def start(self):
        if self._running:
            return
        self._running = True
        self.camera_ok = False
        self.active_detection = False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

    def _loop(self):
        self._load_model()
        if self.model is None:
            print("[PoseCamera] Modelo no disponible. Modo solo teclado.")
            while self._running:
                time.sleep(0.1)
            return

        hold_pose = frozenset()
        hold_t = 0.0
        last_t = time.time()

        while self._running:
            with self._frame_lock:
                frame = self._latest_frame
                self._latest_frame = None

            if frame is None:
                time.sleep(0.03)
                continue

            if not self.enabled:
                continue

            now = time.time()
            dt = now - last_t
            last_t = now

            try:
                results = self.model(frame, imgsz=self.infer_size, verbose=False)
                kpts_raw = results[0].keypoints
                kpts = None
                if kpts_raw is not None and len(kpts_raw.xy) > 0:
                    kpts = kpts_raw.xy[0].cpu().numpy()
            except Exception as e:
                print(f"[PoseCamera] Error en inferencia: {e}")
                kpts = None

            detected = detect_pose(kpts)

            with self._lock:
                self.current_pose = detected

            if self.active_detection:
                if detected and detected == hold_pose:
                    hold_t += dt
                    if hold_t >= POSE_HOLD_NEEDED:
                        hold_t = 0.0
                        if self.on_pose_confirmed:
                            self.on_pose_confirmed(detected)
                else:
                    hold_pose = detected
                    hold_t = 0.0
            else:
                hold_pose = frozenset()
                hold_t = 0.0

        print("[PoseCamera] Hilo terminado.")


# =============================================================================
#  NODO PRINCIPAL -- mismo patron que memoria_node.py
# =============================================================================
class RunState:
    INACTIVE = 0
    INITIALIZING = 1
    ACTIVE = 2
    CLOSING = 3
    ERROR = 4


_RS_NAMES = {0: "INACTIVE", 1: "INITIALIZING", 2: "ACTIVE", 3: "CLOSING", 4: "ERROR"}

CLOSE_WAIT_S = 3.0   # espera maxima (acotada) de los callbacks de ROS al cierre real de la ventana


class DanceGameNode(LifecycleNode):
    """
    REGLA DE HILOS: toda llamada a cv2 con ventana (namedWindow/imshow/waitKey/destroy*)
    ocurre SOLO en el hilo principal (run_display_main_thread). Los callbacks de ROS
    (hilo de spin) unicamente cambian banderas y esperan, con tiempo limite, a que el
    hilo principal termine de cerrar. Asi nunca se toca Qt/OpenCV desde otro hilo.
    """

    def __init__(self):
        super().__init__("dance_game_node")
        self._active = False
        self.is_english = False
        self._mode_pub = None
        self._lang_sub = None
        self._cam_sub = None
        self._game_active_pub = None
        self._bridge = CvBridge()
        self._pose_camera = None
        self.game = None

        self._state_lock = threading.Lock()
        self._run_state = RunState.INACTIVE
        self._want_active = False
        self._shutdown_evt = threading.Event()
        self._closed_evt = threading.Event()
        self._closed_evt.set()          # set = no hay sesion abierta

    # -- helpers de estado ----------------------------------------------------
    def _set_run_state(self, new):
        """Llamar con self._state_lock tomado."""
        old = self._run_state
        self._run_state = new
        self.get_logger().info(f"[ESTADO] {_RS_NAMES[old]} -> {_RS_NAMES[new]}")

    def request_shutdown(self):
        with self._state_lock:
            self._want_active = False
            self._active = False
        self._shutdown_evt.set()
        self._closed_evt.wait(timeout=CLOSE_WAIT_S)

    # -- Lifecycle (hilo de spin: SIN llamadas a la GUI de cv2) -----------------
    def on_configure(self, state):
        self.get_logger().info('DanceGameNode: configurando...')
        try:
            qos_tl = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self._mode_pub = self.create_publisher(String, "/yaren_mode", 10)
            self._game_active_pub = self.create_publisher(Bool, '/yaren/game_active', 10)
            self._lang_sub = self.create_subscription(
                Bool, "/yaren/is_english", self._cb_lang, qos_tl)
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'on_configure error: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        self.get_logger().info('DanceGameNode: ACTIVO')
        with self._state_lock:
            self._want_active = True      # el hilo principal abre UNA sesion cuando pueda
            self._active = True
        self._set_game_active(True)
        if self._cam_sub is None:
            self._cam_sub = self.create_subscription(
                Image, '/csi_camera/image_raw', self._image_callback, 10)
            self.get_logger().info('Suscrito a /csi_camera/image_raw')
        return super().on_activate(state)

    def on_deactivate(self, state):
        self.get_logger().info('DanceGameNode: INACTIVO')
        with self._state_lock:
            self._want_active = False
            self._active = False
        if self._cam_sub is not None:
            try:
                self.destroy_subscription(self._cam_sub)
            except Exception as e:
                self.get_logger().warning(f"Error al destruir suscripcion: {e}")
            self._cam_sub = None
        # El hilo principal cierra ventana, audio y pose. Espera acotada (nunca infinita).
        if not self._closed_evt.wait(timeout=CLOSE_WAIT_S):
            self.get_logger().warning(
                f"El cierre del juego tardo mas de {CLOSE_WAIT_S}s; se continua igualmente.")
        # Solo ahora (ventana y audio ya liberados) se avisa al resto del sistema.
        self._set_game_active(False)
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        with self._state_lock:
            self._want_active = False
            self._active = False
        self._closed_evt.wait(timeout=CLOSE_WAIT_S)
        if self._mode_pub:
            self.destroy_publisher(self._mode_pub)
            self._mode_pub = None
        if self._game_active_pub:
            self.destroy_publisher(self._game_active_pub)
            self._game_active_pub = None
        if self._lang_sub:
            self.destroy_subscription(self._lang_sub)
            self._lang_sub = None
        if self._cam_sub:
            self.destroy_subscription(self._cam_sub)
            self._cam_sub = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.request_shutdown()
        return TransitionCallbackReturn.SUCCESS

    # -- Callbacks --------------------------------------------------------------
    def _cb_lang(self, msg):
        self.is_english = msg.data
        g = self.game
        if g:
            g.is_english = self.is_english

    def _image_callback(self, msg):
        pc = self._pose_camera            # copia local: el hilo principal puede ponerla a None
        if not self._active or pc is None or not getattr(pc, "enabled", True):
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            frame = cv2.flip(frame, 0)
            pc.push_frame(frame)
        except Exception as e:
            self.get_logger().error(f'image_callback error: {e}')

    def _mouse_callback(self, event, x, y, flags, param):
        g = self.game
        if not g:
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            g.process_mouse(x, y, True)
        elif event == cv2.EVENT_MOUSEMOVE:
            g.process_mouse(x, y, False)

    def _set_game_active(self, value: bool):
        if self._game_active_pub:
            try:
                self._game_active_pub.publish(Bool(data=value))
            except Exception:
                pass

    def publish_idle(self):
        if self._mode_pub:
            try:
                msg = String()
                msg.data = "idle"
                self._mode_pub.publish(msg)
            except Exception:
                pass

    @staticmethod
    def _focus_window(win_name):
        time.sleep(0.4)
        # timeout: evita que xdotool --sync quede colgado si la ventana ya se cerro
        os.system(f"timeout 3 xdotool search --sync --name '{win_name}' "
                  "windowactivate --sync windowraise 2>/dev/null")

    # -- Hilo principal: dueno de la ventana --------------------------------------
    def run_display_main_thread(self):
        while rclpy.ok() and not self._shutdown_evt.is_set():
            with self._state_lock:
                go = self._want_active and self._run_state == RunState.INACTIVE
                if go:
                    self._set_run_state(RunState.INITIALIZING)
                    self._closed_evt.clear()
            if not go:
                time.sleep(0.05)
                continue
            self._run_session()

    def _run_session(self):
        win_name = WIN_NAME
        music = None
        pose_camera = None
        window_created = False
        failed = False
        try:
            _log_t("activado -> abriendo ventana")
            self.game = None

            cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
            window_created = True
            cv2.resizeWindow(win_name, WIN_W, WIN_H)
            cv2.setWindowProperty(win_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            cv2.setWindowProperty(win_name, cv2.WND_PROP_TOPMOST, 1)
            cv2.setMouseCallback(win_name, self._mouse_callback)
            cv2.imshow(win_name, _splash("Loading..." if self.is_english else "Cargando..."))
            cv2.waitKey(1)
            _log_t("ventana visible")

            threading.Thread(target=self._focus_window, args=(win_name,), daemon=True).start()
            if not _BACKGROUND_IMAGES:
                threading.Thread(target=_load_backgrounds, daemon=True).start()

            music = MusicManager()
            _log_t("MusicManager listo")

            pose_camera = PoseCamera(model_path=YOLO_MODEL)
            self._pose_camera = pose_camera

            def on_pose_confirmed(pose_set):
                g = self.game
                if g:
                    g.handle_input(pose_set)

            pose_camera.on_pose_confirmed = on_pose_confirmed

            game = DanceGame(music=music, pose_camera=pose_camera,
                             is_english=self.is_english)
            self.game = game
            _log_t("DanceGame listo (renderer + fondo del menu)")

            with self._state_lock:
                self._set_run_state(RunState.ACTIVE)

            last = time.time()
            while rclpy.ok() and self._want_active and not self._shutdown_evt.is_set():
                now = time.time()
                dt = min(now - last, 0.05)
                last = now

                if game.is_english != self.is_english:
                    game.is_english = self.is_english

                if game.state == GameState.EXITED:
                    with self._state_lock:
                        self._want_active = False
                        self._active = False
                    break

                game.update(dt)
                frame = game.render()
                cv2.imshow(win_name, frame)

                key = cv2.waitKey(int(1000 / FPS)) & 0xFF
                if key == 27:
                    if game.state == GameState.PLAYING:
                        music.stop()
                        game.state = GameState.SUMMARY
                    elif game.state in [GameState.WELCOME, GameState.MENU]:
                        game.state = GameState.CONFIRM_EXIT
                    elif game.state == GameState.MODE_SELECT:
                        game.state = GameState.WELCOME
                    else:
                        game.state = GameState.MENU

                direction = KEY_MAP.get(key)
                if direction:
                    game.handle_input(direction)
        except Exception:
            failed = True
            self.get_logger().error("Error en la sesion del juego:\n" + traceback.format_exc())
        finally:
            self._close_session(win_name, window_created, music, pose_camera, failed)

    def _close_session(self, win_name, window_created, music, pose_camera, failed):
        """Cierre idempotente, siempre en el hilo principal (dueno de la ventana)."""
        with self._state_lock:
            if self._run_state == RunState.INACTIVE:
                return                                  # ya cerrado
            self._set_run_state(RunState.ERROR if failed else RunState.CLOSING)
            if failed:
                self._want_active = False               # no reintentar en bucle
                self._active = False

        self.game = None                # el mouse/pose ya no entran al juego
        self._pose_camera = None        # el image_callback deja de empujar frames

        if pose_camera is not None:
            try:
                pose_camera.active_detection = False
                pose_camera.enabled = False
                pose_camera.on_pose_confirmed = None
                pose_camera.stop()
            except Exception as e:
                self.get_logger().warning(f"Error al detener PoseCamera: {e}")
        if music is not None:
            try:
                music.quit()
            except Exception as e:
                self.get_logger().warning(f"Error al cerrar audio: {e}")
        if window_created:
            try:
                cv2.destroyAllWindows()
                for _ in range(3):
                    cv2.waitKey(1)
            except Exception as e:
                self.get_logger().warning(f"Error al cerrar la ventana: {e}")

        if failed:
            self._set_game_active(False)     # que la interfaz no quede en la pantalla de carga
        self.publish_idle()

        with self._state_lock:
            self._set_run_state(RunState.INACTIVE)
            self._closed_evt.set()           # listo para reabrir


# =============================================================================
#  MAIN
# =============================================================================
def main(args=None):
    faulthandler.enable()    # si hay segfault, imprime la traza de Python de cada hilo
    rclpy.init(args=args)
    node = DanceGameNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run_display_main_thread()
    except KeyboardInterrupt:
        print("Interrupcion recibida, cerrando...")
    finally:
        node.request_shutdown()
        try:
            node.trigger_deactivate()
            print("DanceGameNode transicionado a INACTIVE")
        except Exception as e:
            print(f"Error al desactivar: {e}")
        time.sleep(0.5)
        spin_thread.join(timeout=3.0)
        print("DanceGameNode finalizado")


if __name__ == "__main__":
    main()
