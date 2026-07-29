#!/usr/bin/env python3
"""
dance_game.py  --  Yaren Dance (OpenCV)
=========================================
Juego estilo Just Dance renderizado con OpenCV. Adaptado a LifecycleNode.
Rutina fija y unica por cancion. Al terminar la rutina -> SUMMARY automatico.
"""

import math
import random
import threading
import time
import os
import glob
import traceback

import cv2
import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import Bool, String

# =============================================================================
#  MUSICA
# =============================================================================
AUDIO_DIR = "/home/roberto/robotis_ws/src/YAREN2/yaren_radio/audios"
FONDOS_DIR = "/home/roberto/robotis_ws/src/YAREN2/yaren_juegos/fondos"

SONG_MUSIC = {
    "song_1":  os.path.join(AUDIO_DIR, "Balada.mp3"),
    "song_2":  os.path.join(AUDIO_DIR, "justthewayuare.mp3"),
    "song_3":  os.path.join(AUDIO_DIR, "Luli Pampin - ARAM SAM SAM 2021.mp3"),
    "song_4":  os.path.join(AUDIO_DIR, "Luli Pampin - SI TU TIENES MUCHAS GANAS DE APLAUDIR - Official Video.mp3"),
    "song_5":  os.path.join(AUDIO_DIR, "The Winner Takes It All.mp3"),
    "song_6":  os.path.join(AUDIO_DIR, "Rihanna - Don't Stop The Music.mp3"),
    "song_7":  os.path.join(AUDIO_DIR, "Prince Royce - La Carretera.mp3"),
    "song_8":  os.path.join(AUDIO_DIR, "La Despedida.mp3"),
    "song_9":  os.path.join(AUDIO_DIR, "Justin Bieber - Beauty And A Beat.mp3"),
    "song_10": os.path.join(AUDIO_DIR, "Gotas De Lluvia, Grupo Niche.mp3"),
    "song_11": os.path.join(AUDIO_DIR, "Belanova - Rosa Pastel.mp3"),
    "song_12": os.path.join(AUDIO_DIR, "Aitana - SUPERESTRELLA.mp3"),
    "song_13": os.path.join(AUDIO_DIR, "El Dia De Mi Suerte.mp3"),
    "song_14": os.path.join(AUDIO_DIR, "Goo Goo Dolls - Iris.mp3"),
}

MUSIC_VOLUME = 0.6

# =============================================================================
#  CONFIG
# =============================================================================
WIN_NAME = "Yaren Dance"
WIN_W    = 1280
WIN_H    = 720
FPS      = 60
STEP_DUR = 2.4

# Paleta UI
C_WHITE   = (255, 255, 255)
C_BLACK   = (  0,   0,   0)
C_YELLOW  = (  0, 220, 255)
C_RED     = ( 60,  60, 255)
C_LIME    = ( 50, 255,  80)
C_GOLD    = (  0, 200, 255)
C_GRAY    = (100, 100, 100)
C_ARROW   = (255, 210,   0)
C_GREEN   = ( 50, 255,  80)

# Paleta UI mejorada
UI_BG       = ( 10,  8, 20)
UI_PANEL    = ( 22, 18, 40)
UI_ACCENT   = (230, 160, 60)   # dorado YAREN (BGR)
UI_ACCENT2  = (180, 100, 40)
UI_TEXT     = (240, 235, 255)
UI_DIM      = (150, 135, 175)
UI_DIVIDER  = ( 55,  42, 78)
UI_CORRECT  = ( 80, 200,  70)
UI_WRONG    = ( 60,  60, 210)

class GameState:
    WELCOME      = 0
    MENU         = 1
    PLAYING      = 2
    SUMMARY      = 3
    CONFIRM_EXIT = 4
    EXITED       = 5

# =============================================================================
#  PASOS BASE
# =============================================================================
_ALL_STEPS = {
    "arms_up": {
        "name_es": "Brazos Arriba", "name_en": "Arms Up", "keys": ["up"],
        "pose": lambda t: dict(la=-165, ra=-15, ll=15, rl=-15, by=math.sin(t*6)*5),
    },
    "right_arm": {
        "name_es": "Brazo Der.", "name_en": "Right Arm", "keys": ["right"],
        "pose": lambda t: dict(la=-60, ra=-10, ll=15, rl=-15, by=math.sin(t*6)*4),
    },
    "left_arm": {
        "name_es": "Brazo Izq.", "name_en": "Left Arm", "keys": ["left"],
        "pose": lambda t: dict(la=-170, ra=-80, ll=15, rl=-15, by=math.sin(t*6)*4),
    },
    "t_pose": {
        "name_es": "Cruz", "name_en": "T-Pose", "keys": ["left", "right"],
        "pose": lambda t: dict(la=-180, ra=0, ll=15, rl=-15, by=math.sin(t*5)*3),
    },
    "jump": {
        "name_es": "Salta", "name_en": "Jump", "keys": ["up"],
        "pose": lambda t: dict(la=-145, ra=-35, ll=30, rl=-30, by=-abs(math.sin(t*5))*30),
    },
    "clap": {
        "name_es": "Aplaude", "name_en": "Clap", "keys": ["left", "right"],
        "pose": lambda t: (lambda c=abs(math.sin(t*6)): dict(la=-95-c*40, ra=-85+c*40, ll=15, rl=-15, by=math.sin(t*6)*5))(),
    },
    "spin": {
        "name_es": "Gira", "name_en": "Spin", "keys": ["right"],
        "pose": lambda t: dict(la=-155, ra=-25, ll=25, rl=-25, by=math.sin(t*6)*4, sx=math.sin(t*4)*18),
    },
    "hands_head": {
        "name_es": "Manos Cabeza", "name_en": "Hands Head", "keys": ["up"],
        "pose": lambda t: dict(la=-120, ra=-60, la2=-80, ra2=-100, ll=15, rl=-15, by=math.sin(t*5)*4),
    },
    "left_step": {
        "name_es": "Paso Izq.", "name_en": "Step Left", "keys": ["left"],
        "pose": lambda t: dict(la=-120, ra=-60, ll=30, rl=-10, by=math.sin(t*5)*6),
    },
    "right_step": {
        "name_es": "Paso Der.", "name_en": "Step Right", "keys": ["right"],
        "pose": lambda t: dict(la=-60, ra=-120, ll=10, rl=-30, by=math.sin(t*5)*6),
    },
    "wave": {
        "name_es": "Saluda", "name_en": "Wave", "keys": ["right"],
        "pose": lambda t: dict(la=-90, ra=-30+math.sin(t*8)*25, ll=15, rl=-15, by=math.sin(t*4)*3),
    },
    "crouch": {
        "name_es": "Agachate", "name_en": "Crouch", "keys": ["down"],
        "pose": lambda t: dict(la=-145, ra=-35, ll=45, rl=-45, by=20+math.sin(t*6)*4),
    },
}

def S(name):
    return _ALL_STEPS[name]

# =============================================================================
#  CANCIONES
# =============================================================================
SONGS = [
    {
        "id": "song_1", "title": "Balada",
        "figure": (220, 80, 255),
        "routine": [
            S("wave"),       S("left_arm"),   S("right_arm"),  S("hands_head"),
            S("t_pose"),     S("wave"),       S("arms_up"),    S("left_arm"),
            S("right_arm"),  S("hands_head"), S("wave"),       S("t_pose"),
            S("left_arm"),   S("arms_up"),    S("wave"),       S("right_arm"),
            S("hands_head"), S("t_pose"),     S("wave"),       S("arms_up"),
            S("left_arm"),   S("right_arm"),  S("hands_head"), S("wave"),
        ],
    },
    {
        "id": "song_2", "title": "Just The Way You Are - Bruno Mars",
        "figure": (0, 200, 255),
        "routine": [
            S("right_step"), S("left_step"),  S("wave"),       S("right_arm"),
            S("left_arm"),   S("clap"),       S("right_step"), S("left_step"),
            S("arms_up"),    S("wave"),       S("right_step"), S("clap"),
            S("left_step"),  S("right_arm"),  S("left_arm"),   S("wave"),
            S("right_step"), S("left_step"),  S("clap"),       S("arms_up"),
            S("wave"),       S("right_arm"),  S("clap"),       S("left_step"),
        ],
    },
    {
        "id": "song_3", "title": "Aram Sam Sam - Luli Pampin",
        "figure": (255, 230, 0),
        "routine": [
            S("clap"),   S("arms_up"), S("clap"),   S("arms_up"),
            S("clap"),   S("right_arm"),S("clap"),  S("left_arm"),
            S("clap"),   S("arms_up"), S("clap"),   S("right_arm"),
            S("clap"),   S("left_arm"),S("clap"),   S("arms_up"),
            S("clap"),   S("arms_up"), S("clap"),   S("arms_up"),
        ],
    },
    {
        "id": "song_4", "title": "Si Tienes Ganas de Aplaudir - Luli Pampin",
        "figure": (255, 180, 0),
        "routine": [
            S("clap"),   S("clap"),    S("arms_up"),   S("clap"),
            S("clap"),   S("right_arm"),S("clap"),     S("clap"),
            S("left_arm"),S("clap"),  S("clap"),       S("arms_up"),
            S("clap"),   S("clap"),   S("right_step"), S("clap"),
            S("clap"),   S("left_step"),S("clap"),     S("clap"),
            S("arms_up"),S("clap"),   S("clap"),       S("arms_up"),
        ],
    },
    {
        "id": "song_5", "title": "The Winner Takes It All - ABBA",
        "figure": (80, 180, 255),
        "routine": [
            S("arms_up"),  S("right_arm"), S("left_arm"),  S("t_pose"),
            S("wave"),     S("arms_up"),   S("spin"),      S("right_step"),
            S("left_step"),S("t_pose"),    S("clap"),      S("spin"),
            S("arms_up"),  S("jump"),      S("right_arm"), S("left_arm"),
            S("t_pose"),   S("spin"),      S("jump"),      S("clap"),
            S("arms_up"),  S("t_pose"),    S("spin"),      S("jump"),
        ],
    },
    {
        "id": "song_6", "title": "Don't Stop The Music - Rihanna",
        "figure": (255, 80, 150),
        "routine": [
            S("spin"),     S("right_step"),S("left_step"), S("jump"),
            S("clap"),     S("spin"),      S("jump"),      S("right_step"),
            S("left_step"),S("spin"),      S("clap"),      S("jump"),
            S("t_pose"),   S("spin"),      S("right_step"),S("jump"),
            S("clap"),     S("spin"),      S("left_step"), S("jump"),
            S("arms_up"),  S("spin"),      S("clap"),      S("jump"),
        ],
    },
    {
        "id": "song_7", "title": "La Carretera - Prince Royce",
        "figure": (0, 230, 180),
        "routine": [
            S("right_step"),S("left_step"), S("wave"),      S("right_step"),
            S("left_step"), S("wave"),      S("spin"),      S("right_step"),
            S("left_step"), S("clap"),      S("wave"),      S("right_step"),
            S("left_step"), S("spin"),      S("wave"),      S("right_step"),
            S("left_step"), S("clap"),      S("wave"),      S("arms_up"),
            S("right_step"),S("left_step"), S("wave"),      S("spin"),
        ],
    },
    {
        "id": "song_8", "title": "La Despedida",
        "figure": (200, 200, 255),
        "routine": [
            S("hands_head"),S("left_arm"),  S("right_arm"), S("wave"),
            S("hands_head"),S("t_pose"),    S("left_arm"),  S("right_arm"),
            S("wave"),      S("hands_head"),S("arms_up"),   S("wave"),
            S("left_arm"),  S("hands_head"),S("right_arm"), S("wave"),
            S("t_pose"),    S("hands_head"),S("wave"),      S("arms_up"),
            S("left_arm"),  S("right_arm"), S("hands_head"),S("wave"),
        ],
    },
    {
        "id": "song_9", "title": "Beauty And A Beat - Justin Bieber",
        "figure": (255, 220, 0),
        "routine": [
            S("jump"),      S("right_step"),S("left_step"), S("clap"),
            S("jump"),      S("spin"),      S("right_step"),S("clap"),
            S("left_step"), S("jump"),      S("clap"),      S("right_step"),
            S("spin"),      S("left_step"), S("jump"),      S("clap"),
            S("right_step"),S("left_step"), S("jump"),      S("spin"),
            S("clap"),      S("arms_up"),   S("jump"),      S("clap"),
        ],
    },
    {
        "id": "song_10", "title": "Gotas De Lluvia - Grupo Niche",
        "figure": (255, 200, 80),
        "routine": [
            S("right_step"),S("left_step"), S("right_step"),S("clap"),
            S("spin"),      S("right_step"),S("left_step"), S("right_step"),
            S("clap"),      S("spin"),      S("left_step"), S("right_step"),
            S("left_step"), S("clap"),      S("spin"),      S("right_step"),
            S("left_step"), S("clap"),      S("arms_up"),   S("spin"),
            S("right_step"),S("left_step"), S("clap"),      S("spin"),
        ],
    },
    {
        "id": "song_11", "title": "Rosa Pastel - Belanova",
        "figure": (255, 150, 200),
        "routine": [
            S("wave"),      S("right_arm"), S("left_arm"),  S("spin"),
            S("wave"),      S("hands_head"),S("right_arm"), S("left_arm"),
            S("wave"),      S("spin"),      S("arms_up"),   S("wave"),
            S("right_arm"), S("left_arm"),  S("spin"),      S("wave"),
            S("hands_head"),S("right_arm"), S("wave"),      S("spin"),
            S("arms_up"),   S("wave"),      S("right_arm"), S("spin"),
        ],
    },
    {
        "id": "song_12", "title": "Aitana - SUPERESTRELLA",
        "figure": (255, 80, 180),
        "routine": [
            S("wave"),      S("clap"),      S("spin"),      S("t_pose"),
            S("wave"),      S("jump"),      S("clap"),      S("spin"),
            S("right_step"),S("left_step"), S("wave"),      S("clap"),
            S("t_pose"),    S("spin"),      S("wave"),      S("jump"),
            S("clap"),      S("right_step"),S("spin"),      S("wave"),
            S("t_pose"),    S("clap"),      S("jump"),      S("spin"),
        ],
    },
    {
        "id": "song_13", "title": "El Dia De Mi Suerte",
        "figure": (80, 230, 255),
        # Salsa clasica: cadencia latina, pasos y giros con sabor tropical
        "routine": [
            S("right_step"),S("left_step"), S("spin"),      S("right_step"),
            S("arms_up"),   S("left_step"), S("clap"),      S("right_step"),
            S("spin"),      S("wave"),      S("left_step"), S("right_step"),
            S("clap"),      S("spin"),      S("arms_up"),   S("left_step"),
            S("right_step"),S("wave"),      S("spin"),      S("clap"),
            S("left_step"), S("arms_up"),   S("right_step"),S("spin"),
        ],
    },
    {
        "id": "song_14", "title": "Iris - Goo Goo Dolls",
        "figure": (200, 230, 255),
        # Balada rock: movimientos amplios, expresivos, T-pose dramatica
        "routine": [
            S("arms_up"),   S("t_pose"),    S("wave"),      S("left_arm"),
            S("right_arm"), S("arms_up"),   S("hands_head"),S("t_pose"),
            S("wave"),      S("arms_up"),   S("left_arm"),  S("right_arm"),
            S("t_pose"),    S("wave"),      S("arms_up"),   S("hands_head"),
            S("left_arm"),  S("t_pose"),    S("right_arm"), S("arms_up"),
            S("wave"),      S("t_pose"),    S("arms_up"),   S("hands_head"),
        ],
    },
]

KEY_MAP = {
    ord('w'): "up",    82: "up",
    ord('s'): "down",  84: "down",
    ord('a'): "left",  81: "left",
    ord('d'): "right", 83: "right",
}

# =============================================================================
#  CARGA DE FONDOS PNG
# =============================================================================
_BACKGROUND_IMAGES = []   # lista global de np.ndarray (BGR)

def _load_backgrounds():
    """Carga los PNG de FONDOS_DIR al arrancar. Falla en silencio."""
    global _BACKGROUND_IMAGES
    pattern = os.path.join(FONDOS_DIR, "*.png")
    paths   = sorted(glob.glob(pattern))
    for p in paths:
        img = cv2.imread(p)
        if img is not None:
            img_resized = cv2.resize(img, (WIN_W, WIN_H), interpolation=cv2.INTER_LINEAR)
            _BACKGROUND_IMAGES.append(img_resized)
    print(f"[Fondos] Cargados {len(_BACKGROUND_IMAGES)} fondos PNG desde {FONDOS_DIR}")

_load_backgrounds()


def _pick_random_bg():
    """Devuelve un fondo aleatorio (copia), o None si no hay ninguno."""
    if _BACKGROUND_IMAGES:
        return _BACKGROUND_IMAGES[random.randint(0, len(_BACKGROUND_IMAGES)-1)].copy()
    return None


# =============================================================================
#  HELPERS UI
# =============================================================================

def _rounded_rect(img, pt1, pt2, color, radius=12, thickness=-1, alpha=1.0):
    x1, y1 = pt1; x2, y2 = pt2
    if thickness == -1:
        ov = img.copy()
        cv2.rectangle(ov, (x1+radius, y1), (x2-radius, y2), color, -1)
        cv2.rectangle(ov, (x1, y1+radius), (x2, y2-radius), color, -1)
        for cx, cy in [(x1+radius,y1+radius),(x2-radius,y1+radius),
                       (x1+radius,y2-radius),(x2-radius,y2-radius)]:
            cv2.circle(ov, (cx, cy), radius, color, -1)
        if alpha < 1.0:
            cv2.addWeighted(ov, alpha, img, 1-alpha, 0, img)
        else:
            np.copyto(img, ov)
    else:
        cv2.rectangle(img, (x1+radius,y1),(x2-radius,y2), color, thickness)
        cv2.rectangle(img, (x1,y1+radius),(x2,y2-radius), color, thickness)
        for cx, cy in [(x1+radius,y1+radius),(x2-radius,y1+radius),
                       (x1+radius,y2-radius),(x2-radius,y2-radius)]:
            cv2.circle(img, (cx,cy), radius, color, thickness)


def _dark_gradient(h, w, top=(10,8,20), bot=(22,12,38)):
    frame = np.zeros((h,w,3), dtype=np.float32)
    for i in range(h):
        t = i/h
        frame[i] = [top[c]*(1-t)+bot[c]*t for c in range(3)]
    return frame.astype(np.uint8)


# =============================================================================
#  MUSIC MANAGER
# =============================================================================
class MusicManager:
    def __init__(self):
        self._available = False
        self._current_song_id = None
        try:
            import pygame
            pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=512)
            pygame.mixer.music.set_volume(MUSIC_VOLUME)
            self._pygame = pygame
            self._available = True
        except Exception as e:
            print(f"[MusicManager] pygame no disponible: {e}")

    def play(self, song_id):
        if not self._available: return
        if self._current_song_id == song_id and self._pygame.mixer.music.get_busy(): return
        path = SONG_MUSIC.get(song_id, "")
        if not path or not os.path.isfile(path):
            print(f"[MusicManager] No encontrado: {path}")
            self.stop(); return
        try:
            self._pygame.mixer.music.stop()
            self._pygame.mixer.music.load(path)
            self._pygame.mixer.music.set_volume(MUSIC_VOLUME)
            self._pygame.mixer.music.play(loops=-1)
            self._current_song_id = song_id
        except Exception as e:
            print(f"[MusicManager] Error: {e}")

    def stop(self):
        if not self._available: return
        try: self._pygame.mixer.music.stop()
        except Exception: pass
        self._current_song_id = None

    def quit(self):
        if not self._available: return
        try:
            self._pygame.mixer.music.stop()
            self._pygame.mixer.quit()
        except Exception: pass


# =============================================================================
#  RENDERER
# =============================================================================
class DanceRenderer:
    SCALE = 1.6

    def __init__(self, w=WIN_W, h=WIN_H):
        self.W = w
        self.H = h
        self.figure_color = C_GREEN
        self._bg_image = None          # fondo PNG para la cancion actual

    def set_theme(self, song_data):
        self.figure_color = song_data["figure"]
        self._bg_image    = _pick_random_bg()

    def _draw_figure(self, frame, cx, cy, pose, color, alpha=1.0):
        la, ra   = pose.get("la",-90),  pose.get("ra",-90)
        la2, ra2 = pose.get("la2",None),pose.get("ra2",None)
        ll, rl   = pose.get("ll",15),   pose.get("rl",-15)
        by, sx   = pose.get("by",0),    pose.get("sx",0)
        Sc = self.SCALE
        cx, cy = int(cx+sx), int(cy+by)
        lw_body = max(1,int(6*Sc)); lw_limb = max(1,int(5*Sc))
        def pt(dx,dy): return (int(cx+dx*Sc),int(cy+dy*Sc))
        def ang(deg):  return math.radians(deg)
        ov = frame.copy() if alpha<1.0 else frame
        def line(p1,p2,lw=lw_limb): cv2.line(ov,p1,p2,color,lw,cv2.LINE_AA)
        def circle(p,r):             cv2.circle(ov,p,r,color,lw_body,cv2.LINE_AA)
        head_r = max(1,int(18*Sc))
        circle(pt(0,-88),head_r)
        line(pt(0,-70),pt(0,-18),lw_body)
        line(pt(0,-18),pt(-14+ll*0.4,24))
        line(pt(-14+ll*0.4,24),pt(-18+ll*0.7,64))
        line(pt(0,-18),pt(14+rl*0.4,24))
        line(pt(14+rl*0.4,24),pt(18+rl*0.7,64))
        lax,lay = math.cos(ang(la))*46,math.sin(ang(la))*46
        p_ls,p_le = pt(0,-54),pt(lax,lay-54)
        line(p_ls,p_le)
        if la2 is not None:
            lax2=lax+math.cos(ang(la2))*32; lay2=lay-54+math.sin(ang(la2))*32
            line(p_le,pt(lax2,lay2+54))
        rax,ray = math.cos(ang(ra))*46,math.sin(ang(ra))*46
        p_rs,p_re = pt(0,-54),pt(rax,ray-54)
        line(p_rs,p_re)
        if ra2 is not None:
            rax2=rax+math.cos(ang(ra2))*32; ray2=ray-54+math.sin(ang(ra2))*32
            line(p_re,pt(rax2,ray2+54))
        if alpha<1.0:
            cv2.addWeighted(ov,alpha,frame,1-alpha,0,frame)

    def _draw_arrows(self, frame, cx, cy, step, alpha=1.0):
        dir_vec={"up":(0,-1),"down":(0,1),"left":(-1,0),"right":(1,0)}
        keys=step["keys"]; n=len(keys)
        for i,k in enumerate(keys):
            dx,dy=dir_vec[k]
            offset=(i-(n-1)/2)*52
            ax=int(cx+dx*75+offset*abs(dy))
            ay=int(cy-62+dy*75+offset*abs(dx))
            ang_r=math.atan2(dy,dx)
            col_a=tuple(int(c*alpha) for c in C_ARROW)
            cv2.ellipse(frame,(ax,ay),(24,24),0,
                        math.degrees(ang_r)-117,math.degrees(ang_r)+117,col_a,3,cv2.LINE_AA)
            tip_x=int(ax+math.cos(ang_r+math.radians(117))*24)
            tip_y=int(ay+math.sin(ang_r+math.radians(117))*24)
            p1=(tip_x,tip_y)
            p2=(int(tip_x-math.cos(ang_r+math.radians(117)+0.55)*13),
                int(tip_y-math.sin(ang_r+math.radians(117)+0.55)*13))
            p3=(int(tip_x-math.cos(ang_r+math.radians(117)-0.55)*13),
                int(tip_y-math.sin(ang_r+math.radians(117)-0.55)*13))
            cv2.fillPoly(frame,[np.array([p1,p2,p3],np.int32)],col_a,cv2.LINE_AA)

    def text(self, frame, txt, cx, cy, scale=0.7, color=C_WHITE, bold=False):
        font  = cv2.FONT_HERSHEY_DUPLEX if bold else cv2.FONT_HERSHEY_SIMPLEX
        thick = 2 if bold else 1
        (tw,th),_ = cv2.getTextSize(txt,font,scale,thick)
        x,y = int(cx-tw/2),int(cy+th/2)
        # Sombra
        cv2.putText(frame,txt,(x+2,y+2),font,scale,C_BLACK,thick+2,cv2.LINE_AA)
        cv2.putText(frame,txt,(x,y),font,scale,color,thick,cv2.LINE_AA)

    def text_multiline(self, frame, txt, cx, start_y, scale=0.7, color=C_WHITE, line_spacing=40):
        for i,ln in enumerate(txt.split('\n')):
            self.text(frame,ln,cx,start_y+i*line_spacing,scale,color)

    def draw_background(self, frame, t):
        """Fondo PNG con overlay semitransparente para legibilidad."""
        if self._bg_image is not None:
            # Copiar PNG al frame
            np.copyto(frame, self._bg_image)
            # Overlay oscuro para que las figuras y HUD sean legibles
            overlay = np.zeros_like(frame)
            cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)
        else:
            # Fallback gradiente
            frame[:] = _dark_gradient(self.H, self.W)

        # Cuadricula perspectiva sutil en el suelo
        fp = int(self.H*0.70)
        for i in range(11):
            x1=int(self.W*i/10)
            cv2.line(frame,(x1,fp),(int((x1-self.W/2)*2.2+self.W/2),self.H),
                     (60,50,80),1,cv2.LINE_AA)
        for i in range(5):
            fy=fp+int(i*(self.H-fp)/4)
            sp=int(i*self.W*0.22)
            cv2.line(frame,(-sp,fy),(self.W+sp,fy),(60,50,80),1,cv2.LINE_AA)

    def draw_step_bar(self, frame, step_progress):
        bar_w=int(self.W*0.46); bar_x=int(self.W*0.27)
        bar_y=int(self.H*0.88); bar_h=14
        # Fondo
        _rounded_rect(frame,(bar_x-4,bar_y-4),(bar_x+bar_w+4,bar_y+bar_h+4),
                      (18,14,32),radius=8)
        fill_w=int(bar_w*max(0.0,min(step_progress,1.0)))
        if fill_w>2:
            col=(C_LIME if step_progress>0.60 else
                 (C_YELLOW if step_progress>0.25 else C_RED))
            _rounded_rect(frame,(bar_x,bar_y),(bar_x+fill_w,bar_y+bar_h),col,radius=6)
        mx=bar_x+fill_w
        cv2.circle(frame,(mx,bar_y+bar_h//2),9,C_WHITE,-1,cv2.LINE_AA)
        cv2.circle(frame,(mx,bar_y+bar_h//2),9,(140,140,140),1,cv2.LINE_AA)
        self.text(frame,"NOW", bar_x,      bar_y-10,0.42,(180,170,210))
        self.text(frame,"NEXT",bar_x+bar_w,bar_y-10,0.42,(180,170,210))

    def draw_song_progress(self, frame, done_steps, total_steps):
        bar_h=6; progress=done_steps/max(total_steps,1)
        fill_w=int(self.W*progress)
        cv2.rectangle(frame,(0,0),(self.W,bar_h),(20,16,36),-1)
        if fill_w>0:
            cv2.rectangle(frame,(0,0),(fill_w,bar_h),UI_ACCENT,-1)
        self.text(frame,f"{done_steps}/{total_steps}",self.W-50,bar_h+14,0.38,UI_ACCENT)

    def draw_button(self, frame, txt, x, y, w, h, bg_color, is_hover=False):
        radius=10
        color=(min(255,bg_color[0]+50),min(255,bg_color[1]+50),min(255,bg_color[2]+50)) \
              if is_hover else bg_color
        _rounded_rect(frame,(x,y),(x+w,y+h),color,radius=radius)
        border=(UI_ACCENT if is_hover else
                tuple(min(255,int(c*0.55)) for c in UI_ACCENT))
        _rounded_rect(frame,(x,y),(x+w,y+h),border,radius=radius,thickness=2)
        self.text(frame,txt,x+w//2,y+h//2-2,0.65,UI_TEXT,bold=True)
        return (x,y,w,h)


# =============================================================================
#  LOGICA DEL JUEGO
# =============================================================================
class DanceGame:
    SONGS_PER_PAGE = 6

    def __init__(self, music, is_english=False):
        self.is_english = is_english
        self.music      = music
        self.renderer   = DanceRenderer(WIN_W, WIN_H)
        self.state      = GameState.WELCOME

        self._current_song_id = None
        self._routine         = []
        self._routine_index   = 0
        self._routine_total   = 0
        self._steps_done      = 0
        self._finishing       = False
        self._menu_page       = 0

        self.start_delay  = 0.0
        self.score  = 0
        self.combo  = 1
        self.t      = 0.0
        self.queue  = []

        self.rating_txt   = ""
        self.rating_col   = C_GOLD
        self.rating_timer = 0.0

        self.stats_attempted = 0
        self.stats_perfect   = 0
        self.stats_good      = 0
        self.stats_ok        = 0

        self.mx = 0; self.my = 0
        self.btn_lock      = threading.Lock()
        self.buttons_rects = {}
        self.temp_rects    = {}

    # ── helpers ──────────────────────────────────────────────────────────────

    def _step_name(self, step):
        return step["name_en"] if self.is_english else step["name_es"]

    def _reset_stats(self):
        self.score=0; self.combo=1; self.t=0.0; self.queue=[]
        self.stats_attempted=0; self.stats_perfect=0
        self.stats_good=0; self.stats_ok=0
        self.start_delay=0.0; self._routine_index=0
        self._steps_done=0; self._finishing=False

    def play_song(self, song_data):
        self._reset_stats()
        self.renderer.set_theme(song_data)
        self._current_song_id = song_data["id"]
        self._routine         = list(song_data["routine"])
        self._routine_total   = len(self._routine)
        self._enqueue_next_steps()
        self.state       = GameState.PLAYING
        self.start_delay = 2.0
        self.music.play(self._current_song_id)

    def _enqueue_next_steps(self):
        pending_not_done=sum(1 for q in self.queue if not q["done"])
        slots=max(0,3-pending_not_done)
        for _ in range(slots):
            if self._routine_index>=len(self._routine):
                self._finishing=True; break
            step=self._routine[self._routine_index]; self._routine_index+=1
            base=min((q["progress"] for q in self.queue),default=0.0)
            self.queue.append({
                "step":step,"progress":base-0.85,"done":False,
                "pending":step["keys"][:],"hit_flash":0.0,
                "anim_t":random.random()*10,
            })

    def _show_rating(self,txt,col):
        self.rating_txt=txt; self.rating_col=col; self.rating_timer=0.75

    # ── input ────────────────────────────────────────────────────────────────

    def handle_input(self, direction):
        if self.state!=GameState.PLAYING: return
        active=next((q for q in self.queue
                     if not q["done"] and 0.55<=q["progress"]<=1.05),None)
        if not active or direction not in active["pending"]: return
        active["pending"].remove(direction); active["hit_flash"]=1.0
        if not active["pending"]:
            p=active["progress"]; self.stats_attempted+=1
            if p<0.70:
                pts,txt,col=100*self.combo,"PERFECT!",C_GOLD
                self.combo+=1; self.stats_perfect+=1
            elif p<0.90:
                pts,txt,col=60*self.combo,"GOOD",C_LIME
                self.stats_good+=1
            else:
                pts,txt,col=20,"OK",(255,200,0)
                self.combo=max(1,self.combo-1); self.stats_ok+=1
            self.score+=pts; self._show_rating(txt,col); active["done"]=True

    # ── update ───────────────────────────────────────────────────────────────

    def update(self, dt):
        if self.state!=GameState.PLAYING: return
        self.t+=dt; self.rating_timer=max(0.0,self.rating_timer-dt)
        if self.start_delay>0:
            self.start_delay-=dt; return
        for q in self.queue:
            q["anim_t"]+=dt
            if not q["done"]: q["progress"]+=dt/STEP_DUR
        for q in self.queue:
            if not q["done"] and q["progress"]>1.05:
                self.stats_attempted+=1
                if q["pending"]: self.combo=1; self._show_rating("MISS",C_RED)
                q["done"]=True; q["_was_miss"]=True; self._steps_done+=1
        for q in self.queue:
            if q["done"] and not q.get("_counted",False):
                q["_counted"]=True
                if not q.get("_was_miss",False): self._steps_done+=1
        self.queue=[q for q in self.queue if q["progress"]<2.5]
        if self._finishing:
            if not any(not q["done"] for q in self.queue):
                self.music.stop(); self.state=GameState.SUMMARY
        else:
            self._enqueue_next_steps()

    # ── mouse ────────────────────────────────────────────────────────────────

    def process_mouse(self, x, y, clicked):
        self.mx=x; self.my=y
        if clicked:
            with self.btn_lock:
                btns=dict(self.buttons_rects)
            for btn_id,rect in btns.items():
                bx,by,bw,bh=rect
                if bx<=x<=bx+bw and by<=y<=by+bh:
                    self._handle_button_click(btn_id)
            if x<60 and y<60:
                if self.state==GameState.PLAYING:
                    self.music.stop(); self.state=GameState.SUMMARY
                elif self.state in [GameState.WELCOME,GameState.MENU]:
                    self.state=GameState.CONFIRM_EXIT

    def _handle_button_click(self, btn_id):
        if btn_id=="btn_start_welcome":
            self.state=GameState.MENU
        elif btn_id.startswith("btn_song_"):
            idx=int(btn_id.split("_")[2]); self.play_song(SONGS[idx])
        elif btn_id=="btn_menu_next":
            max_page=(len(SONGS)-1)//self.SONGS_PER_PAGE
            self._menu_page=min(self._menu_page+1,max_page)
        elif btn_id=="btn_menu_prev":
            self._menu_page=max(self._menu_page-1,0)
        elif btn_id=="btn_summary_menu":
            self.music.stop(); self.state=GameState.MENU
        elif btn_id=="btn_summary_restart":
            song_data=next(s for s in SONGS if s["id"]==self._current_song_id)
            self.play_song(song_data)
        elif btn_id=="btn_confirm_yes":
            self.music.stop(); self.state=GameState.EXITED
        elif btn_id=="btn_confirm_no":
            self.state=GameState.MENU

    def _is_hovered(self, rect):
        bx,by,bw,bh=rect
        return bx<=self.mx<=bx+bw and by<=self.my<=by+bh

    # ── render ───────────────────────────────────────────────────────────────

    def render(self):
        self.temp_rects={}
        frame=np.zeros((WIN_H,WIN_W,3),dtype=np.uint8)
        if   self.state==GameState.WELCOME:     self._render_welcome(frame)
        elif self.state==GameState.MENU:         self._render_menu(frame)
        elif self.state==GameState.PLAYING:      self._render_playing(frame)
        elif self.state==GameState.SUMMARY:      self._render_summary(frame)
        elif self.state==GameState.CONFIRM_EXIT: self._render_confirm_exit(frame)

        # Boton X universal
        if self.state in [GameState.WELCOME,GameState.MENU,GameState.PLAYING]:
            _rounded_rect(frame,(4,4),(54,54),(38,28,58),radius=10)
            _rounded_rect(frame,(4,4),(54,54),UI_DIVIDER,radius=10,thickness=1)
            cv2.putText(frame,"X",(16,39),cv2.FONT_HERSHEY_DUPLEX,
                        1.1,(130,100,200),2,cv2.LINE_AA)

        with self.btn_lock:
            self.buttons_rects=self.temp_rects
        return frame

    # ─ Welcome ───────────────────────────────────────────────────────────────

    def _render_welcome(self, frame):
        frame[:]=_dark_gradient(WIN_H,WIN_W,(8,6,18),(22,14,38))

        # Titulo grande
        title = ("Welcome to Yaren Dance" if self.is_english
                 else "Bienvenido a Yaren Dance")
        desc  = ("Follow the dancer's moves to the music.\n"
                 "Each song has its own unique routine.\n"
                 "Soon: camera will evaluate your moves!" if self.is_english else
                 "Sigue los movimientos del bailarin al ritmo.\n"
                 "Cada cancion tiene su coreografia unica.\n"
                 "Proximamente: la camara evaluara tus pasos!")

        # Panel central
        _rounded_rect(frame,(WIN_W//2-380,100),(WIN_W//2+380,WIN_H-140),
                      UI_PANEL,radius=20,alpha=0.85)
        _rounded_rect(frame,(WIN_W//2-380,100),(WIN_W//2+380,WIN_H-140),
                      UI_DIVIDER,radius=20,thickness=1)

        # Linea decorativa dorada
        cv2.line(frame,(WIN_W//2-200,148),(WIN_W//2+200,148),UI_ACCENT,2,cv2.LINE_AA)

        self.renderer.text(frame,title,WIN_W/2,170,1.25,UI_ACCENT,bold=True)
        self.renderer.text_multiline(frame,desc,WIN_W/2,270,0.78,UI_TEXT,line_spacing=48)

        bw,bh=240,60; bx=WIN_W//2-bw//2; by=WIN_H-170
        hov=self._is_hovered((bx,by,bw,bh))
        self.temp_rects["btn_start_welcome"]=self.renderer.draw_button(
            frame,"INICIAR" if not self.is_english else "START",
            bx,by,bw,bh,(50,90,30),hov)

    # ─ Menu ──────────────────────────────────────────────────────────────────

    def _render_menu(self, frame):
        frame[:]=_dark_gradient(WIN_H,WIN_W,(10,8,22),(20,14,40))

        title_txt="MENU DE CANCIONES" if not self.is_english else "SONG MENU"
        total_pages=(len(SONGS)-1)//self.SONGS_PER_PAGE+1
        page_lbl=f"{self._menu_page+1} / {total_pages}"

        # Cabecera
        _rounded_rect(frame,(0,0),(WIN_W,72),UI_PANEL,radius=0)
        cv2.line(frame,(0,72),(WIN_W,72),UI_DIVIDER,1,cv2.LINE_AA)
        self.renderer.text(frame,title_txt,WIN_W/2,40,1.05,UI_ACCENT,bold=True)
        self.renderer.text(frame,page_lbl,WIN_W-80,40,0.55,UI_DIM)

        # Tarjetas de canciones
        start_idx=self._menu_page*self.SONGS_PER_PAGE
        page_songs=SONGS[start_idx:start_idx+self.SONGS_PER_PAGE]

        bw,bh=860,72; gap=8
        total_h=len(page_songs)*bh+(len(page_songs)-1)*gap
        start_y=(WIN_H-total_h)//2+20

        for i,song in enumerate(page_songs):
            global_idx=start_idx+i
            bx=WIN_W//2-bw//2; by=start_y+i*(bh+gap)
            hov=self._is_hovered((bx,by,bw,bh))

            # Fondo de tarjeta
            bg_col=(35,28,55) if not hov else (55,40,85)
            _rounded_rect(frame,(bx,by),(bx+bw,by+bh),bg_col,radius=12)
            border_col=UI_ACCENT if hov else UI_DIVIDER
            _rounded_rect(frame,(bx,by),(bx+bw,by+bh),border_col,
                          radius=12,thickness=2 if hov else 1)

            # Numero de cancion
            num_txt=f"{global_idx+1:02d}"
            cv2.putText(frame,num_txt,(bx+18,by+bh//2+8),
                        cv2.FONT_HERSHEY_DUPLEX,0.75,
                        UI_ACCENT if hov else UI_DIM,2,cv2.LINE_AA)

            # Titulo (sin numero de pasos)
            txt=song["title"]
            col=UI_TEXT if hov else UI_DIM
            cv2.putText(frame,txt,(bx+70,by+bh//2+8),
                        cv2.FONT_HERSHEY_DUPLEX,0.70,col,1,cv2.LINE_AA)

            self.temp_rects[f"btn_song_{global_idx}"]=(bx,by,bw,bh)

        # Navegacion paginas
        nav_y=WIN_H-52; nav_bw=180; nav_bh=40
        max_page=(len(SONGS)-1)//self.SONGS_PER_PAGE
        if self._menu_page>0:
            px=WIN_W//2-nav_bw-16
            hov=self._is_hovered((px,nav_y,nav_bw,nav_bh))
            self.temp_rects["btn_menu_prev"]=self.renderer.draw_button(
                frame,"< ANTERIOR" if not self.is_english else "< PREV",
                px,nav_y,nav_bw,nav_bh,(40,50,75),hov)
        if self._menu_page<max_page:
            nx=WIN_W//2+16
            hov=self._is_hovered((nx,nav_y,nav_bw,nav_bh))
            self.temp_rects["btn_menu_next"]=self.renderer.draw_button(
                frame,"SIGUIENTE >" if not self.is_english else "NEXT >",
                nx,nav_y,nav_bw,nav_bh,(40,50,75),hov)

    # ─ Summary ───────────────────────────────────────────────────────────────

    def _render_summary(self, frame):
        frame[:]=_dark_gradient(WIN_H,WIN_W,(8,6,18),(22,14,38))

        if self.stats_attempted>0:
            sr=int(((self.stats_perfect+self.stats_good+self.stats_ok)/self.stats_attempted)*100)
            pr=int((self.stats_perfect/self.stats_attempted)*100)
        else:
            sr,pr=0,0
        misses=self.stats_attempted-self.stats_perfect-self.stats_good-self.stats_ok

        # Panel central
        _rounded_rect(frame,(WIN_W//2-380,80),(WIN_W//2+380,WIN_H-100),
                      UI_PANEL,radius=20,alpha=0.90)
        _rounded_rect(frame,(WIN_W//2-380,80),(WIN_W//2+380,WIN_H-100),
                      UI_DIVIDER,radius=20,thickness=1)
        cv2.line(frame,(WIN_W//2-280,140),(WIN_W//2+280,140),UI_ACCENT,2,cv2.LINE_AA)

        title="RESUMEN DEL BAILE" if not self.is_english else "DANCE SUMMARY"
        self.renderer.text(frame,title,WIN_W/2,118,1.3,UI_ACCENT,bold=True)

        l1=(f"Exito: {sr}%" if not self.is_english else f"Success: {sr}%")
        l2=(f"Puntaje: {self.score} pts" if not self.is_english else f"Score: {self.score} pts")
        l3=(f"Perfectos: {self.stats_perfect}  Buenos: {self.stats_good}  OK: {self.stats_ok}  Miss: {misses}"
            if not self.is_english else
            f"Perfect: {self.stats_perfect}  Good: {self.stats_good}  OK: {self.stats_ok}  Miss: {misses}")
        l4=(f"{pr}% pasos perfectos - sigue asi!" if not self.is_english
            else f"{pr}% perfect steps - keep it up!")

        self.renderer.text(frame,l1,WIN_W/2,220,1.1,UI_TEXT,bold=True)
        self.renderer.text(frame,l2,WIN_W/2,285,0.95,C_GOLD,bold=True)
        self.renderer.text(frame,l3,WIN_W/2,355,0.72,(180,170,210))
        self.renderer.text(frame,l4,WIN_W/2,415,0.78,UI_TEXT)

        bw,bh=280,58; gap=36
        bx1=WIN_W//2-bw-gap//2; bx2=WIN_W//2+gap//2; by=WIN_H-145
        hov1=self._is_hovered((bx1,by,bw,bh)); hov2=self._is_hovered((bx2,by,bw,bh))
        self.temp_rects["btn_summary_menu"]=self.renderer.draw_button(
            frame,"MENU",bx1,by,bw,bh,(50,45,80),hov1)
        self.temp_rects["btn_summary_restart"]=self.renderer.draw_button(
            frame,"REPETIR" if not self.is_english else "PLAY AGAIN",
            bx2,by,bw,bh,(30,80,150),hov2)

    # ─ Confirm exit ──────────────────────────────────────────────────────────

    def _render_confirm_exit(self, frame):
        frame[:]=_dark_gradient(WIN_H,WIN_W,(20,8,8),(40,14,14))
        _rounded_rect(frame,(WIN_W//2-320,200),(WIN_W//2+320,WIN_H-200),
                      UI_PANEL,radius=20,alpha=0.90)
        _rounded_rect(frame,(WIN_W//2-320,200),(WIN_W//2+320,WIN_H-200),
                      UI_DIVIDER,radius=20,thickness=1)
        msg=("Salir de Yaren Dance?" if not self.is_english
             else "Exit Yaren Dance?")
        self.renderer.text(frame,msg,WIN_W/2,300,1.1,UI_TEXT,bold=True)
        bw,bh=160,56; gap=40
        bx1=WIN_W//2-bw-gap//2; bx2=WIN_W//2+gap//2; by=390
        hov1=self._is_hovered((bx1,by,bw,bh)); hov2=self._is_hovered((bx2,by,bw,bh))
        self.temp_rects["btn_confirm_yes"]=self.renderer.draw_button(
            frame,"SI" if not self.is_english else "YES",
            bx1,by,bw,bh,(30,30,160),hov1)
        self.temp_rects["btn_confirm_no"]=self.renderer.draw_button(
            frame,"NO",bx2,by,bw,bh,(50,45,80),hov2)

    # ─ Playing ───────────────────────────────────────────────────────────────

    def _render_playing(self, frame):
        R=self.renderer
        R.draw_background(frame,self.t)
        R.draw_song_progress(frame,min(self._steps_done,self._routine_total),self._routine_total)

        ACTIVE_X=int(WIN_W*0.30); GHOST_X=int(WIN_W*0.72); FIG_Y=int(WIN_H*0.50)

        if self.start_delay>0:
            txt="GET READY!" if self.is_english else "PREPARATE!"
            R.text(frame,txt,WIN_W/2,WIN_H/2,1.8,C_GOLD,bold=True)
            R.draw_step_bar(frame,1.0)
        else:
            pending_steps=sorted(
                [q for q in self.queue if not q["done"]],
                key=lambda q:q["progress"],reverse=True)
            target_q=pending_steps[0] if pending_steps else None

            if target_q:
                bar_prog=1.0-max(0.0,min((target_q["progress"]-0.55)/0.50,1.0))
                R.draw_step_bar(frame,bar_prog)
            else:
                R.draw_step_bar(frame,1.0)

            upcoming_q=next(
                (q for q in pending_steps if q["progress"]<0.55),None)

            active_drawn=False; active_name="..."
            for q in self.queue:
                prog=q["progress"]
                if prog<-1.5 or prog>1.5: continue
                pose=q["step"]["pose"](q["anim_t"])
                if q==upcoming_q:
                    alpha=max(0.0,min((prog+0.5)/1.05,1.0))*0.32
                    R._draw_figure(frame,GHOST_X,FIG_Y,pose,(200,140,80),alpha)
                    R._draw_arrows(frame,GHOST_X,FIG_Y,q["step"],alpha*0.9)
                    na=int(alpha*1.3*200)
                    R.text(frame,self._step_name(q["step"]),GHOST_X,FIG_Y+100,
                           0.6,(na,na//2+80,na//2+100))
                is_target  =(q==target_q and 0.55<=prog<=1.05)
                is_flashing=(q["done"] and q.get("hit_flash",0)>0)
                if is_target or is_flashing:
                    if is_flashing:
                        q["hit_flash"]=max(0.0,q["hit_flash"]-0.06)
                    flash=q.get("hit_flash",0)
                    fc=R.figure_color
                    col=(int(fc[0]*(1-flash)+255*flash),
                         int(fc[1]*(1-flash)+255*flash),
                         int(fc[2]*(1-flash)+200*flash)) if flash>0 else fc
                    R._draw_figure(frame,ACTIVE_X,FIG_Y,pose,col,1.0)
                    if is_target:
                        R._draw_arrows(frame,ACTIVE_X,FIG_Y,q["step"],0.95)
                        active_drawn=True; active_name=self._step_name(q["step"])
            if active_drawn:
                R.text(frame,active_name,ACTIVE_X,FIG_Y+105,0.65,C_WHITE,bold=True)

            # Nombre del paso actual centrado arriba
            closest_name=self._step_name(target_q["step"]) if target_q else "..."
            cw=260; cx_box=WIN_W//2-cw//2
            _rounded_rect(frame,(cx_box,10),(cx_box+cw,48),(20,16,38),radius=10)
            _rounded_rect(frame,(cx_box,10),(cx_box+cw,48),UI_DIVIDER,radius=10,thickness=1)
            R.text(frame,closest_name,WIN_W/2,32,0.72,UI_TEXT,bold=True)

        # Score y combo
        _rounded_rect(frame,(68,10),(180,70),(22,18,42),radius=10)
        R.text(frame,str(self.score),124,35,1.0,UI_TEXT,bold=True)
        if self.combo>1:
            R.text(frame,f"x{self.combo}",124,62,0.68,C_LIME,bold=True)

        # Controles
        hint="W A S D / flechas  |  X = salir"
        R.text(frame,hint,WIN_W/2,WIN_H-14,0.38,(110,100,140))

        # Rating flash
        if self.rating_timer>0:
            alpha=min(self.rating_timer/0.3,1.0)
            if alpha>0:
                R.text(frame,self.rating_txt,WIN_W/2,int(WIN_H*0.38),2.2,
                       tuple(int(c*alpha) for c in self.rating_col),bold=True)


# =============================================================================
#  LIFECYCLE NODE
# =============================================================================
class DanceGameNode(LifecycleNode):
    def __init__(self):
        super().__init__("dance_game_node")
        self._active=False; self.is_english=False
        self._mode_pub=None; self._lang_sub=None; self.game=None

    def on_configure(self, state):
        self.get_logger().info('DanceGameNode: configurando...')
        try:
            qos_tl=QoSProfile(depth=1,durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self._mode_pub=self.create_publisher(String,"/yaren_mode",10)
            self._lang_sub=self.create_subscription(Bool,"/yaren/is_english",self._cb_lang,qos_tl)
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'on_configure error: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        self.get_logger().info('DanceGameNode: ACTIVO')
        self._active=True
        return super().on_activate(state)

    def on_deactivate(self, state):
        self.get_logger().info('DanceGameNode: INACTIVO')
        self._active=False
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        if self._mode_pub: self.destroy_publisher(self._mode_pub)
        if self._lang_sub: self.destroy_subscription(self._lang_sub)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self._active=False
        return TransitionCallbackReturn.SUCCESS

    def _cb_lang(self, msg):
        self.is_english=msg.data
        if self.game: self.game.is_english=self.is_english

    def publish_idle(self):
        if self._mode_pub:
            msg=String(); msg.data="idle"; self._mode_pub.publish(msg)


# =============================================================================
#  MAIN
# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    node=DanceGameNode()
    ros_thread=threading.Thread(target=rclpy.spin,args=(node,),daemon=True)
    ros_thread.start()

    music=MusicManager()
    window_created=False; game=None; last=time.time()

    def mouse_callback(event,x,y,flags,param):
        if game and node._active:
            if event==cv2.EVENT_MOUSEMOVE:
                game.process_mouse(x,y,False)
            elif event==cv2.EVENT_LBUTTONDOWN:
                game.process_mouse(x,y,True)

    try:
        while rclpy.ok():
            if node._active:
                if not window_created:
                    game=DanceGame(music=music,is_english=node.is_english)
                    node.game=game
                    cv2.namedWindow(WIN_NAME,cv2.WINDOW_NORMAL)
                    cv2.resizeWindow(WIN_NAME,WIN_W,WIN_H)
                    cv2.setWindowProperty(WIN_NAME,cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
                    cv2.setWindowProperty(WIN_NAME,cv2.WND_PROP_TOPMOST,1)
                    cv2.setMouseCallback(WIN_NAME,mouse_callback)
                    def _focus():
                        time.sleep(0.4)
                        os.system(f"xdotool search --sync --name '{WIN_NAME}' "
                                  "windowactivate --sync windowraise 2>/dev/null")
                    threading.Thread(target=_focus,daemon=True).start()
                    window_created=True; last=time.time()

                now=time.time(); dt=min(now-last,0.05); last=now
                if game.is_english!=node.is_english:
                    game.is_english=node.is_english
                if game.state==GameState.EXITED:
                    node._active=False; node.publish_idle(); continue

                game.update(dt)
                frame=game.render()
                cv2.imshow(WIN_NAME,frame)

                key=cv2.waitKey(int(1000/FPS))&0xFF
                if key==27:
                    if game.state==GameState.PLAYING:
                        music.stop(); game.state=GameState.SUMMARY
                    elif game.state in [GameState.WELCOME,GameState.MENU]:
                        game.state=GameState.CONFIRM_EXIT
                    else:
                        game.state=GameState.MENU

                direction=KEY_MAP.get(key) or KEY_MAP.get(cv2.waitKeyEx(1)&0xFFFF)
                if direction: game.handle_input(direction)

            else:
                if window_created:
                    music.stop(); cv2.destroyWindow(WIN_NAME)
                    window_created=False; game=None; node.game=None
                time.sleep(0.1)

    except Exception as e:
        node.get_logger().error(f"Fallo en loop: {e}\n{traceback.format_exc()}")
    finally:
        music.quit()
        if window_created: cv2.destroyAllWindows()
        node.publish_idle(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=="__main__":
    main()