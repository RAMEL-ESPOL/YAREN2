#!/usr/bin/env python3
"""
palabras_game.py - YAREN2 Word Game (El Juego de las Palabras)
LifecycleNode | OpenCV touch input | Bilingue | 3 niveles de dificultad

NOTA: todo el texto que se dibuja con OpenCV debe ser ASCII (sin tildes ni enie),
porque cv2.putText las dibuja como "??". Como red de seguridad, la funcion
ascii_safe() limpia automaticamente cualquier texto antes de dibujarlo.

Version con MONO + PALMERA (todo en un solo archivo):
  - El mono sube la palmera con cada acierto y baja con cada error.
  - Si ganas, el mono llega a la cima y agarra una banana.
  - Si pierdes, el mono se cae y queda mareado (espirales + estrellitas girando).
  - Pista, pantallas de resultado, confeti, teclado fisico y thread-safety
    como en la version anterior.
"""

import math
import os
import threading
import random
import time
import subprocess
import unicodedata

import cv2
import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool

# ---------------------------------------------
#  WORD BANK (Palabra : Definicion)  -- SIN TILDES
# ---------------------------------------------
WORD_BANK = {
    "es": {
        "facil": {
            "GATO": "Felino domestico que suele cazar ratones.",
            "PERRO": "Animal canino considerado el mejor amigo del hombre.",
            "CASA": "Edificio donde viven las personas.",
            "LUNA": "Satelite natural que orbita la Tierra.",
            "MESA": "Mueble con tablero y patas usado para comer o trabajar.",
            "SILLA": "Mueble con respaldo disenado para sentarse.",
            "AGUA": "Liquido transparente esencial para la vida.",
            "FUEGO": "Emision de luz y calor producida por la combustion.",
            "TIERRA": "Planeta donde vivimos o el suelo que pisamos.",
            "NUBE": "Masa de vapor de agua suspendida en el cielo."
        },
        "medio": {
            "MARIPOSA": "Insecto volador con alas grandes y coloridas.",
            "ELEFANTE": "Mamifero terrestre de gran tamano con trompa larga.",
            "DINOSAURIO": "Reptil prehistorico extinto hace millones de anos.",
            "COMPUTADORA": "Maquina electronica para procesar informacion.",
            "TELESCOPIO": "Instrumento optico para observar los astros.",
            "BIBLIOTECA": "Lugar donde se guardan y prestan libros.",
            "SUBMARINO": "Vehiculo capaz de navegar bajo el agua.",
            "MONTANA": "Gran elevacion natural del terreno."
        },
        "dificil": {
            "FOTOSINTESIS": "Proceso de las plantas para convertir luz en energia.",
            "ELECTROMAGNETICO": "Relativo a la interaccion de campos electricos y magneticos.",
            "BIODIVERSIDAD": "Variedad de especies animales y vegetales en su medio ambiente.",
            "EXTRATERRESTRE": "Que pertenece al espacio exterior de la Tierra.",
            "PALEONTOLOGIA": "Ciencia que estudia los seres organicos extintos a traves de fosiles."
        }
    },
    "en": {
        "facil": {
            "CAT": "Domestic feline that often hunts mice.",
            "DOG": "Canine animal considered man's best friend.",
            "HOUSE": "Building where people live.",
            "MOON": "Natural satellite that orbits the Earth.",
            "TABLE": "Furniture with a flat top and legs.",
            "CHAIR": "Furniture with a back designed for sitting.",
            "WATER": "Clear liquid essential for life.",
            "FIRE": "Emission of light and heat from combustion.",
            "EARTH": "The planet we live on, or the soil.",
            "CLOUD": "Mass of water vapor suspended in the sky."
        },
        "medio": {
            "BUTTERFLY": "Flying insect with large, colorful wings.",
            "ELEPHANT": "Large terrestrial mammal with a long trunk.",
            "DINOSAUR": "Prehistoric reptile extinct for millions of years.",
            "COMPUTER": "Electronic machine for processing data.",
            "TELESCOPE": "Optical instrument to observe distant stars.",
            "LIBRARY": "Place where books are kept and lent out.",
            "SUBMARINE": "Vehicle capable of navigating underwater.",
            "MOUNTAIN": "Large natural elevation of the earth's surface."
        },
        "dificil": {
            "PHOTOSYNTHESIS": "Process by which plants convert light into energy.",
            "ELECTROMAGNETIC": "Relating to the interrelation of electric currents and magnetic fields.",
            "BIODIVERSITY": "Variety of plant and animal life in a particular habitat.",
            "EXTRATERRESTRIAL": "Originating outside the limits of the Earth.",
            "PALEONTOLOGY": "Branch of science concerned with fossil animals and plants."
        }
    }
}

LEVEL_CONFIG = {
    "facil":   {"lives": 6, "words": 10, "label_es": "FACIL",   "label_en": "EASY",   "color": (85, 185, 60)},
    "medio":   {"lives": 5, "words": 5,  "label_es": "MEDIO",   "label_en": "MEDIUM", "color": (40, 150, 245)},
    "dificil": {"lives": 3, "words": 1,  "label_es": "DIFICIL", "label_en": "HARD",   "color": (70, 70, 225)},
}
LEVEL_ORDER = ("facil", "medio", "dificil")

W, H = 800, 480
FPS = 30

# Colores (BGR)
KRAFT = (82, 150, 205);  PAPER = (234, 246, 252); PAPER_DK = (190, 212, 226)
RULE = (238, 218, 178);  MARGIN = (160, 160, 235)
INK = (92, 48, 30);      INK_DIM = (125, 105, 95); PURPLE = (150, 52, 112)
TEAL = (95, 105, 30);    WHITE = (255, 255, 255);  GRAY_OFF = (190, 200, 205)
BLUE = (225, 80, 25);    PINK = (150, 60, 235);    YELLOW = (40, 205, 250)
CYAN = (205, 195, 30);   ORANGE = (50, 100, 240);  BLACK = (45, 42, 42)
GREEN = (85, 185, 60);   RED = (70, 70, 225)
BROWN = (30, 70, 125);   BROWN_DK = (18, 42, 80);  ROPE = (95, 145, 195); SKIN = (205, 230, 242)
RED_DK = (30, 30, 140);  GRAY_DK = (150, 160, 165)

ALPHABET = list("ABCDEFGHIJKLMNOPQRSTUVWXYZ")

KEY_STYLES = [(BLUE, WHITE), (PINK, WHITE), (YELLOW, INK),
              (CYAN, WHITE), (BLACK, WHITE), (ORANGE, WHITE)]

FONT = cv2.FONT_HERSHEY_DUPLEX
SCRIPT = cv2.FONT_HERSHEY_SCRIPT_COMPLEX


# ---------------------------------------------
#  Helpers de dibujo
# ---------------------------------------------
def ascii_safe(s):
    """Quita tildes/enie/signos que OpenCV no puede dibujar (evita los '??')."""
    return unicodedata.normalize('NFKD', s).encode('ascii', 'ignore').decode('ascii')


def _mul(c, k):
    return tuple(int(max(0, min(255, v * k))) for v in c)


def _lighten(c, a):
    return tuple(min(255, v + a) for v in c)


def _rr_fill(img, x1, y1, x2, y2, color, r):
    r = max(0, min(r, (x2 - x1) // 2, (y2 - y1) // 2))
    if r == 0:
        cv2.rectangle(img, (x1, y1), (x2, y2), color, -1)
        return
    cv2.rectangle(img, (x1 + r, y1), (x2 - r, y2), color, -1)
    cv2.rectangle(img, (x1, y1 + r), (x2, y2 - r), color, -1)
    for cx, cy in ((x1 + r, y1 + r), (x2 - r, y1 + r), (x1 + r, y2 - r), (x2 - r, y2 - r)):
        cv2.circle(img, (cx, cy), r, color, -1, cv2.LINE_AA)


def rrect(img, p1, p2, fill, r=12, border=None, bw=2, alpha=1.0):
    x1, y1 = p1
    x2, y2 = p2
    if alpha < 1.0:
        xa, ya = max(0, x1), max(0, y1)
        xb, yb = min(img.shape[1] - 1, x2), min(img.shape[0] - 1, y2)
        if xb <= xa or yb <= ya:
            return
        roi = img[ya:yb + 1, xa:xb + 1]
        tmp = roi.copy()
        rrect(tmp, (x1 - xa, y1 - ya), (x2 - xa, y2 - ya), fill, r, border, bw, 1.0)
        cv2.addWeighted(tmp, alpha, roi, 1 - alpha, 0, roi)
        return
    if border is not None:
        _rr_fill(img, x1, y1, x2, y2, border, r)
        _rr_fill(img, x1 + bw, y1 + bw, x2 - bw, y2 - bw, fill, max(0, r - bw))
    else:
        _rr_fill(img, x1, y1, x2, y2, fill, r)


def text_width(s, scale, thick, font=FONT):
    (tw, _), _ = cv2.getTextSize(ascii_safe(s), font, scale, thick)
    return tw


def put(img, s, x, y, scale=0.8, color=INK, thick=1, center=False, shadow=None, font=FONT):
    s = ascii_safe(s)
    if center:
        x = x - text_width(s, scale, thick, font) // 2
    if shadow is not None:
        cv2.putText(img, s, (x + 2, y + 2), font, scale, shadow, thick + 1, cv2.LINE_AA)
    cv2.putText(img, s, (x, y), font, scale, color, thick, cv2.LINE_AA)


def wrap_lines(text, max_w, font, scale, thick):
    """Divide el texto en lineas que caben en max_w."""
    lines, current = [], ""
    for word in ascii_safe(text).split(' '):
        test = (current + " " + word).strip()
        if text_width(test, scale, thick, font) > max_w and current:
            lines.append(current)
            current = word
        else:
            current = test
    if current:
        lines.append(current)
    return lines


def draw_fitted_text(img, text, cx, top, max_w, max_h, color, thick=2):
    """Dibuja texto centrado, reduciendo la fuente hasta que quepa en max_w x max_h."""
    lines, scale, line_h = [], 1.0, 40
    for scale in (1.0, 0.9, 0.8, 0.7, 0.6, 0.5):
        lines = wrap_lines(text, max_w, FONT, scale, thick)
        line_h = int(44 * scale)
        if len(lines) * line_h <= max_h:
            break
    block_h = len(lines) * line_h
    y = top + (max_h - block_h) // 2 + int(line_h * 0.75)
    for i, line in enumerate(lines):
        put(img, line, cx, y + i * line_h, scale, color, thick, center=True)


_T = np.linspace(0, 2 * np.pi, 48)
_HEART_PTS = np.stack([16 * np.sin(_T) ** 3,
                       -(13 * np.cos(_T) - 5 * np.cos(2 * _T) - 2 * np.cos(3 * _T) - np.cos(4 * _T))],
                      axis=1) / 17.0


def heart(img, cx, cy, size, color, outline=None):
    pts = (_HEART_PTS * size + np.array([cx, cy])).astype(np.int32)
    cv2.fillPoly(img, [pts], color, cv2.LINE_AA)
    if outline is not None:
        cv2.polylines(img, [pts], True, outline, 2, cv2.LINE_AA)


def star(img, cx, cy, r, color, outline=None):
    pts = []
    for i in range(10):
        ang = -math.pi / 2 + i * math.pi / 5
        rr = r if i % 2 == 0 else r * 0.45
        pts.append((cx + rr * math.cos(ang), cy + rr * math.sin(ang)))
    pts = np.array(pts, np.int32)
    cv2.fillPoly(img, [pts], color, cv2.LINE_AA)
    if outline is not None:
        cv2.polylines(img, [pts], True, outline, 2, cv2.LINE_AA)


def pencil(img, ox, oy, ang_deg=-45, s=1.0):
    a = math.radians(ang_deg)
    ca, sa = math.cos(a), math.sin(a)

    def tr(pts):
        return np.array([(ox + (x * ca - y * sa) * s, oy + (x * sa + y * ca) * s) for x, y in pts], np.int32)

    parts = [([(-20, 0), (0, -9), (0, 9)], (150, 190, 225)),
             ([(-20, 0), (-11, -4), (-11, 4)], (60, 50, 50)),
             ([(0, -9), (62, -9), (62, 9), (0, 9)], YELLOW),
             ([(62, -9), (74, -9), (74, 9), (62, 9)], (190, 190, 195)),
             ([(74, -9), (90, -9), (90, 9), (74, 9)], PINK)]
    for pts, col in parts:
        p = tr(pts)
        cv2.fillPoly(img, [p], col, cv2.LINE_AA)
        cv2.polylines(img, [p], True, INK, 2, cv2.LINE_AA)


def speech_bubble(img, cx, cy, text, tail_to, rx=50, ry=26, scale=0.7):
    A, B = (cx - 20, cy + ry - 8), (cx - 2, cy + ry - 4)
    cv2.ellipse(img, (cx, cy), (rx, ry), 0, 0, 360, INK, -1, cv2.LINE_AA)
    cv2.fillPoly(img, [np.array([A, B, tail_to], np.int32)], INK, cv2.LINE_AA)
    cv2.ellipse(img, (cx, cy), (rx - 4, ry - 4), 0, 0, 360, WHITE, -1, cv2.LINE_AA)
    tip = (int(tail_to[0] + (cx - tail_to[0]) * 0.09), int(tail_to[1] + (cy - tail_to[1]) * 0.09))
    cv2.fillPoly(img, [np.array([(cx - 17, cy + ry - 11), (cx - 5, cy + ry - 7), tip], np.int32)],
                 WHITE, cv2.LINE_AA)
    put(img, text, cx, cy + int(10 * scale), scale, INK, 2, center=True)


def cloud(img, x, y, s=1.0):
    parts = [(0, 0, 10), (14, -6, 13), (30, 0, 10), (14, 6, 10)]
    for dy, col in ((2, (238, 222, 200)), (0, WHITE)):
        for px, py, pr in parts:
            cv2.circle(img, (int(x + px * s), int(y + py * s + dy)), max(2, int(pr * s)), col, -1, cv2.LINE_AA)


def draw_close_button(frame, c=(30, 30), r=20):
    cv2.circle(frame, (c[0], c[1] + 3), r, RED_DK, -1, cv2.LINE_AA)
    cv2.circle(frame, c, r, RED, -1, cv2.LINE_AA)
    cv2.circle(frame, c, r, WHITE, 2, cv2.LINE_AA)
    d = int(r * 0.35)
    cv2.line(frame, (c[0] - d, c[1] - d), (c[0] + d, c[1] + d), WHITE, 3, cv2.LINE_AA)
    cv2.line(frame, (c[0] + d, c[1] - d), (c[0] - d, c[1] + d), WHITE, 3, cv2.LINE_AA)


# ---------------------------------------------
#  MONO + PALMERA
#  Estados: 'climb' (sube/baja), 'win' (agarra banana), 'dizzy' (caido y mareado)
# ---------------------------------------------
# Colores (BGR)
MK_BODY = (22, 92, 184);  MK_DK = (12, 48, 105)
MK_FACE = (170, 205, 255); MK_EAR = (150, 160, 250)
TRUNK = (45, 95, 150);    TRUNK_DK = (18, 42, 80)
FROND = (70, 165, 60);    FROND_DK = (35, 95, 30);  FROND_LT = (120, 205, 100)
BANANA = (60, 215, 250);  BANANA_DK = (20, 120, 170)
GRASS = (110, 200, 110);  GRASS_DK = (70, 160, 70)
AURA = (40, 150, 245);    HALO = (130, 130, 130)
STAR_COLS = ((150, 60, 235), (50, 100, 240), (225, 80, 25), (85, 185, 60))
AA = cv2.LINE_AA


# ---------------------------------------------
#  Helpers
# ---------------------------------------------
def _pt(p):
    return (int(round(p[0])), int(round(p[1])))


def _arr(pts):
    return np.array([_pt(p) for p in pts], np.int32)


def _smooth(pts, n=8):
    """Catmull-Rom: suaviza una lista de puntos."""
    p = [pts[0]] + list(pts) + [pts[-1]]
    out = []
    for i in range(1, len(p) - 2):
        p0, p1, p2, p3 = p[i - 1], p[i], p[i + 1], p[i + 2]
        for j in range(n):
            u = j / n
            u2, u3 = u * u, u * u * u
            out.append(tuple(0.5 * ((2 * p1[k]) + (-p0[k] + p2[k]) * u +
                                    (2 * p0[k] - 5 * p1[k] + 4 * p2[k] - p3[k]) * u2 +
                                    (-p0[k] + 3 * p1[k] - 3 * p2[k] + p3[k]) * u3) for k in (0, 1)))
    out.append(pts[-1])
    return out


def _limb(img, pts, w, col=MK_BODY, smooth=False):
    if smooth:
        pts = _smooth(pts)
    a = _arr(pts)
    r = max(1, w // 2)
    cv2.polylines(img, [a], False, MK_DK, w + 4, AA)
    for q in (a[0], a[-1]):
        cv2.circle(img, (int(q[0]), int(q[1])), r + 2, MK_DK, -1, AA)
    cv2.polylines(img, [a], False, col, w, AA)
    for q in (a[0], a[-1]):
        cv2.circle(img, (int(q[0]), int(q[1])), r, col, -1, AA)


def _blob(img, c, axes, ang=0, col=MK_BODY, th=2):
    ax = (max(1, int(axes[0])), max(1, int(axes[1])))
    cv2.ellipse(img, _pt(c), ax, ang, 0, 360, col, -1, AA)
    if th:
        cv2.ellipse(img, _pt(c), ax, ang, 0, 360, MK_DK, th, AA)


def _spiral(img, c, r0, r1, turns, start, col, th=2):
    pts = []
    n = 40
    for i in range(n + 1):
        u = i / n
        ang = start + u * turns * 2 * math.pi
        r = r0 + (r1 - r0) * u
        pts.append((c[0] + r * math.cos(ang), c[1] + r * math.sin(ang)))
    cv2.polylines(img, [_arr(pts)], False, col, th, AA)


def _star5(img, cx, cy, r, col, outline=None):
    pts = []
    for i in range(10):
        ang = -math.pi / 2 + i * math.pi / 5
        rr = r if i % 2 == 0 else r * 0.45
        pts.append((cx + rr * math.cos(ang), cy + rr * math.sin(ang)))
    a = _arr(pts)
    cv2.fillPoly(img, [a], col, AA)
    cv2.polylines(img, [a], True, outline if outline else _dark(col), 1, AA)


def _dark(c):
    return tuple(int(v * 0.6) for v in c)


def _banana(img, cx, cy, size, ang_deg):
    """Banana en forma de media luna centrada en (cx, cy)."""
    a = math.radians(ang_deg)
    ca, sa = math.cos(a), math.sin(a)
    outer, inner = [], []
    n = 14
    for i in range(n + 1):
        u = -1 + 2 * i / n
        yc = 0.45 * size * (1 - u * u)
        w = 0.30 * size * (1 - u * u) ** 0.6 + 0.06 * size
        outer.append((size * u, yc + w))
        inner.append((size * u, yc - w))
    loc = outer + inner[::-1]
    pts = [(cx + x * ca - y * sa, cy + x * sa + y * ca) for x, y in loc]
    poly = _arr(pts)
    cv2.fillPoly(img, [poly], BANANA, AA)
    cv2.polylines(img, [poly], True, BANANA_DK, 2, AA)
    for sgn in (-1, 1):  # puntas oscuras
        x, y = sgn * size, 0.0
        cv2.circle(img, _pt((cx + x * ca - y * sa, cy + x * sa + y * ca)), max(1, int(size * 0.1)), TRUNK_DK, -1, AA)


# ---------------------------------------------
#  Cabeza del mono
# ---------------------------------------------
def _head(img, hx, hy, s, look=1.0, mood='happy', t=0):
    for sx in (-1, 1):
        _blob(img, (hx + sx * 15 * s, hy), (7 * s, 7 * s))
        cv2.circle(img, _pt((hx + sx * 15 * s, hy)), max(1, int(4 * s)), MK_EAR, -1, AA)
    # mechon
    for dx, dy in ((-2, -19), (3, -20)):
        cv2.line(img, _pt((hx + dx * s * 0.5, hy - 12 * s)), _pt((hx + dx * s, hy + dy * s)), MK_DK,
                 max(2, int(3 * s)), AA)
    _blob(img, (hx, hy), (15 * s, 14 * s), 0)
    # cara color durazno
    for sx in (-1, 1):
        cv2.circle(img, _pt((hx + sx * 6.5 * s, hy + 1 * s)), max(2, int(8 * s)), MK_FACE, -1, AA)
    cv2.ellipse(img, _pt((hx, hy + 5 * s)), (int(11 * s), int(8 * s)), 0, 0, 360, MK_FACE, -1, AA)

    ey = hy - 2 * s
    for sx in (-1, 1):
        ex = hx + sx * 5.5 * s + look * 1.5 * s
        if mood == 'dizzy':
            _spiral(img, (ex, ey), 0.6 * s, 4.4 * s, 2, t * 0.3, MK_DK, 1)
        else:
            cv2.circle(img, _pt((ex, ey)), max(2, int(2.7 * s)), (30, 25, 25), -1, AA)
            cv2.circle(img, _pt((ex - 0.8 * s, ey - 0.9 * s)), max(1, int(0.9 * s)), (255, 255, 255), -1, AA)
    cv2.ellipse(img, _pt((hx + look * 1.0 * s, hy + 3 * s)), (max(1, int(2 * s)), max(1, int(1.2 * s))), 0, 0, 360,
                MK_DK, -1, AA)

    if mood == 'open':
        c = _pt((hx + look * s, hy + 8 * s))
        ax = (int(5.5 * s), int(4.5 * s))
        cv2.ellipse(img, c, ax, 0, 0, 180, (80, 70, 215), -1, AA)
        cv2.ellipse(img, c, ax, 0, 0, 180, MK_DK, 1, AA)
    elif mood == 'dizzy':
        cv2.ellipse(img, _pt((hx, hy + 9 * s)), (max(1, int(2.6 * s)), max(1, int(3 * s))), 0, 0, 360,
                    (80, 70, 215), -1, AA)
        cv2.ellipse(img, _pt((hx, hy + 9 * s)), (max(1, int(2.6 * s)), max(1, int(3 * s))), 0, 0, 360, MK_DK, 1, AA)
    else:
        cv2.ellipse(img, _pt((hx + look * s, hy + 6.5 * s)), (int(4.5 * s), int(2.8 * s)), 0, 10, 170, MK_DK, 2, AA)


# ---------------------------------------------
#  Mono abrazado al tronco (subiendo / ganando)
# ---------------------------------------------
def _monkey_hug(img, tx, cy, s, ph=0.0, win=False, t=0):
    x0 = tx - 18 * s
    lw = max(3, int(5 * s))
    # cola enroscada
    tail = [(x0 - 6 * s, cy + 16 * s), (x0 - 20 * s, cy + 28 * s), (x0 - 36 * s, cy + 20 * s),
            (x0 - 40 * s, cy + 2 * s), (x0 - 30 * s, cy - 8 * s), (x0 - 22 * s, cy - 2 * s)]
    _limb(img, tail, max(3, int(4.5 * s)), smooth=True)
    # piernas agarradas al tronco
    _limb(img, [(x0 + 2 * s, cy + 14 * s), (x0 + 15 * s, cy + 26 * s), (tx - 3 * s, cy + 19 * s + ph)], lw)
    _limb(img, [(x0 - 2 * s, cy + 14 * s), (x0 + 9 * s, cy + 35 * s), (tx - 3 * s, cy + 35 * s - ph)], lw)
    for fy in (cy + 19 * s + ph, cy + 35 * s - ph):
        _blob(img, (tx - 2 * s, fy), (4.5 * s, 4 * s), 0)
    # cuerpo + panza
    _blob(img, (x0, cy), (12 * s, 19 * s), 12)
    cv2.ellipse(img, _pt((x0 + 3 * s, cy + 2 * s)), (int(7 * s), int(13 * s)), 12, 0, 360, MK_FACE, -1, AA)
    # brazo que se agarra
    _limb(img, [(x0 + 5 * s, cy - 11 * s), (x0 + 14 * s, cy - 20 * s), (tx, cy - 26 * s - ph)], lw)
    _blob(img, (tx, cy - 26 * s - ph), (4.8 * s, 4.8 * s))
    if win:
        hand = (x0 - 26 * s, cy - 38 * s + 2 * math.sin(t * 0.3) * s)
        _limb(img, [(x0 - 3 * s, cy - 10 * s), (x0 - 18 * s, cy - 22 * s), hand], lw)
        _banana(img, hand[0] - 3 * s, hand[1] - 13 * s, 17 * s, -70)
        _blob(img, hand, (4.8 * s, 4.8 * s))
    else:
        _limb(img, [(x0 + 4 * s, cy - 6 * s), (x0 + 13 * s, cy - 5 * s), (tx, cy - 10 * s + ph)], lw)
        _blob(img, (tx, cy - 10 * s + ph), (4.8 * s, 4.8 * s))
    _head(img, x0 + 4 * s, cy - 31 * s, s, look=1.0, mood='open' if win else 'happy', t=t)


# ---------------------------------------------
#  Mono mareado en el suelo
# ---------------------------------------------
def _monkey_dizzy(img, mx, gy, s, t=0):
    lw = max(3, int(5 * s))
    cv2.ellipse(img, _pt((mx - 8 * s, gy)), (int(52 * s), int(6 * s)), 0, 0, 360, (190, 205, 200), -1, AA)
    # cola
    tail = [(mx + 22 * s, gy - 10 * s), (mx + 40 * s, gy - 3 * s), (mx + 54 * s, gy - 14 * s),
            (mx + 58 * s, gy - 30 * s), (mx + 48 * s, gy - 34 * s)]
    _limb(img, tail, max(3, int(4.5 * s)), smooth=True)
    # piernas al aire
    _limb(img, [(mx + 14 * s, gy - 14 * s), (mx + 26 * s, gy - 30 * s), (mx + 30 * s, gy - 46 * s)], lw)
    _blob(img, (mx + 31 * s, gy - 49 * s), (5 * s, 5 * s), 0)
    _limb(img, [(mx + 6 * s, gy - 14 * s), (mx + 12 * s, gy - 34 * s), (mx + 7 * s, gy - 50 * s)], lw)
    _blob(img, (mx + 6 * s, gy - 53 * s), (5 * s, 5 * s), 0)
    # cuerpo
    _blob(img, (mx, gy - 16 * s), (25 * s, 13 * s), -4)
    cv2.ellipse(img, _pt((mx + 2 * s, gy - 14 * s)), (int(15 * s), int(7 * s)), -4, 0, 360, MK_FACE, -1, AA)
    # brazos
    _limb(img, [(mx - 12 * s, gy - 12 * s), (mx - 6 * s, gy - 4 * s), (mx + 4 * s, gy - 3 * s)], lw)
    _limb(img, [(mx - 14 * s, gy - 20 * s), (mx - 6 * s, gy - 32 * s), (mx - 12 * s, gy - 42 * s)], lw)
    # cabeza
    hx, hy = mx - 36 * s, gy - 28 * s
    _head(img, hx, hy, s, look=0, mood='dizzy', t=t)

    # auras naranjas (espirales)
    _spiral(img, (hx - 34 * s, hy - 36 * s), 1 * s, 13 * s, 2.2, t * 0.12, AURA, 2)
    _spiral(img, (hx + 36 * s, hy - 30 * s), 1 * s, 11 * s, 2.0, -t * 0.12 + 1, AURA, 2)

    # halos grises + estrellitas girando
    hc = (hx, hy - 24 * s)
    cv2.ellipse(img, _pt(hc), (int(34 * s), int(9 * s)), -8, 0, 360, HALO, 2, AA)
    cv2.ellipse(img, _pt((hc[0] + 4 * s, hc[1] - 4 * s)), (int(28 * s), int(7 * s)), 6, 0, 360, HALO, 2, AA)
    for i in range(4):
        ang = t * 0.14 + i * math.pi / 2
        sx = hc[0] + 38 * s * math.cos(ang)
        sy = hc[1] - 6 * s + 13 * s * math.sin(ang)
        _star5(img, sx, sy, (6 + 3 * (math.sin(ang) > 0)) * s, STAR_COLS[i])


# ---------------------------------------------
#  Palmera
# ---------------------------------------------
def _frond(img, top, ang_deg, L, wmax, droop):
    a = math.radians(ang_deg)
    ca, sa = math.cos(a), math.sin(a)
    n = 14
    left, right, spine = [], [], []
    for i in range(n + 1):
        u = i / n
        x = top[0] + L * u * ca
        y = top[1] + L * u * sa + droop * L * u * u
        dx, dy = L * ca, L * sa + 2 * droop * L * u
        m = math.hypot(dx, dy) or 1.0
        nx, ny = -dy / m, dx / m
        w = wmax * (math.sin(math.pi * min(1.0, u * 1.05)) ** 0.7) + 0.5
        if i > 2:
            w *= 0.72 + 0.28 * (i % 2)   # bordes dentados tipo hoja de palma
        left.append((x + nx * w, y + ny * w))
        right.append((x - nx * w, y - ny * w))
        spine.append((x, y))
    poly = _arr(left + right[::-1])
    cv2.fillPoly(img, [poly], FROND, AA)
    cv2.polylines(img, [poly], True, FROND_DK, 2, AA)
    cv2.polylines(img, [_arr(spine)], False, FROND_LT, 1, AA)


def draw_palm(img, bx, by, h, t=0, bananas=3):
    k = h / 255.0
    wb, wt = 9 * k, 5 * k
    trunk = _arr([(bx - wb, by), (bx + wb, by), (bx + wt, by - h), (bx - wt, by - h)])
    cv2.fillPoly(img, [trunk], TRUNK, AA)
    n = max(4, int(h / 14))
    for i in range(1, n):
        y = by - i * h / n
        hw = wb + (wt - wb) * (i / n)
        cv2.line(img, _pt((bx - hw, y)), _pt((bx + hw, y - 3 * k)), TRUNK_DK, max(1, int(2 * k)), AA)
    cv2.polylines(img, [trunk], True, TRUNK_DK, max(2, int(2.5 * k)), AA)

    top = (bx, by - h)
    fronds = [(165, 42), (15, 42), (-172, 52), (-8, 52), (-145, 62), (-35, 62), (-115, 54), (-65, 54)]
    for i, (ang, L) in enumerate(fronds):
        _frond(img, top, ang + 3 * math.sin(t * 0.07 + i), L * k * 1.35, 9 * k, 0.6)

    # racimo de bananas colgando bajo las hojas
    sx, sy = bx + 3 * k, by - h + 4 * k
    offs = (-9, 0, 9) if bananas >= 3 else (-9, 9)
    for off in offs:
        cx, cy = sx + off * 0.9 * k, sy + 19 * k
        _banana(img, cx, cy, 14 * k, 90 + off * 1.6)
    cv2.circle(img, _pt((sx, sy + 2 * k)), max(2, int(3 * k)), TRUNK_DK, -1, AA)


def draw_ground(img, bx, by, w):
    cv2.ellipse(img, (int(bx), int(by + 2)), (int(w), 9), 0, 0, 360, GRASS, -1, AA)
    cv2.ellipse(img, (int(bx), int(by + 2)), (int(w), 9), 0, 0, 360, GRASS_DK, 2, AA)


# ---------------------------------------------
#  API publica
# ---------------------------------------------
def draw_monkey_scene(frame, bx, by, h, state='climb', progress=0.5, t=0, ms=None, ground=True):
    """bx, by = base del tronco (centro, en el suelo). h = altura del tronco en px.
    progress 0..1 = que tan arriba va el mono (solo 'climb'). t = contador de frames."""
    s = ms if ms is not None else max(0.7, h / 175.0)
    if ground:
        draw_ground(frame, bx, by, 62 * h / 255.0 + 20)
    draw_palm(frame, bx, by, h, t, bananas=2 if state == 'win' else 3)
    top_y = by - h
    if state == 'climb':
        progress = max(0.0, min(0.92, progress))
        cy = by - 48 * s - progress * (h - 95 * s)
        _monkey_hug(frame, bx, cy, s, ph=3 * s * math.sin(t * 0.2), t=t)
    elif state == 'win':
        _monkey_hug(frame, bx, top_y + 68 * s, s, ph=0.0, win=True, t=t)
    else:
        _monkey_dizzy(frame, bx - 62 * s, by - 2, s, t)



KEY_STYLE = (BLUE, WHITE)   # color unico de las teclas

KID_DEFS = {
    "es": {
        "GATO": "Es un animalito peludo que dice miau y le gusta tomar leche.",
        "PERRO": "Es un animal que ladra, mueve la cola y juega contigo.",
        "CASA": "Es el lugar donde vives y duermes con tu familia.",
        "LUNA": "Brilla en el cielo de noche y a veces parece una sonrisa.",
        "MESA": "Ahi pones tu plato para comer o tu hoja para dibujar.",
        "SILLA": "Es donde te sientas para comer o hacer la tarea.",
        "AGUA": "La tomas cuando tienes sed y tambien sirve para lavarte las manos.",
        "FUEGO": "Es caliente y brilla con colores naranja y rojo. Cuidado, quema!",
        "TIERRA": "Es el planeta donde vivimos y tambien el suelo donde crecen las plantas.",
        "NUBE": "Es blanca y suave, flota en el cielo y a veces trae lluvia.",
    },
    "en": {
        "CAT": "A furry pet that says meow and loves to drink milk.",
        "DOG": "A pet that barks, wags its tail and plays with you.",
        "HOUSE": "The place where you live and sleep with your family.",
        "MOON": "It shines in the sky at night and sometimes looks like a smile.",
        "TABLE": "You put your plate on it to eat or your paper to draw.",
        "CHAIR": "You sit on it to eat or to do your homework.",
        "WATER": "You drink it when you are thirsty and wash your hands with it.",
        "FIRE": "It is hot and shiny, with orange and red colors. Careful, it burns!",
        "EARTH": "The planet where we live and the soil where plants grow.",
        "CLOUD": "It is white and soft, floats in the sky and sometimes brings rain.",
    },
}


def _terrain_y(x):
    """Altura del terreno: alto a la izquierda (palmera) y bajando hacia la derecha."""
    if x <= 240:
        return 360
    if x >= 380:
        return 452
    u = (x - 240) / 140.0
    u = u * u * (3 - 2 * u)
    return int(360 + 92 * u)


def _build_jungle_bg():
    rng = np.random.default_rng(11)
    img = np.zeros((H, W, 3), np.uint8)
    top = np.array((95, 200, 165), np.float32)
    bot = np.array((150, 232, 222), np.float32)
    for y in range(H):
        k = y / (H - 1)
        img[y] = (top * (1 - k) + bot * k).astype(np.uint8)

    # arboles lejanos (neblina)
    far = (135, 175, 120)
    ov = img.copy()
    for x in range(-30, W + 60, 70):
        cy = 190 + int(rng.integers(-25, 25))
        cv2.ellipse(ov, (x, cy), (70 + int(rng.integers(0, 30)), 34 + int(rng.integers(0, 14))),
                    0, 0, 360, far, -1, cv2.LINE_AA)
        cv2.rectangle(ov, (x - 5, cy), (x + 5, 460), far, -1)
    cv2.addWeighted(ov, 0.55, img, 0.45, 0, img)

    # troncos grandes
    base, light, dark = (80, 100, 85), (110, 130, 105), (50, 62, 52)

    def trunk(x0, w0, x1, w1):
        ys = np.linspace(H, 0, 22)
        left, right = [], []
        for i, y in enumerate(ys):
            k = i / (len(ys) - 1)
            cx = x0 + (x1 - x0) * k + 14 * math.sin(k * 5.0 + x0 * 0.05)
            w = w0 + (w1 - w0) * k
            left.append((int(cx - w), int(y)))
            right.append((int(cx + w), int(y)))
        cv2.fillPoly(img, [np.array(left + right[::-1], np.int32)], base, cv2.LINE_AA)
        cv2.polylines(img, [np.array(right, np.int32)], False, dark, 6, cv2.LINE_AA)
        hl = [(int(l[0] + (r[0] - l[0]) * 0.22), l[1]) for l, r in zip(left, right)]
        cv2.polylines(img, [np.array(hl, np.int32)], False, light, 5, cv2.LINE_AA)
        for _ in range(10):
            idx = int(rng.integers(1, len(ys) - 2))
            l, r = left[idx], right[idx]
            px = int(l[0] + (r[0] - l[0]) * rng.uniform(0.3, 0.8))
            cv2.line(img, (px, l[1]), (px + int(rng.integers(-4, 4)), l[1] - int(rng.integers(20, 50))),
                     dark, 2, cv2.LINE_AA)

    for x0, w0, x1, w1 in ((38, 42, 52, 28), (268, 26, 292, 20), (560, 34, 540, 24), (735, 40, 750, 28)):
        trunk(x0, w0, x1, w1)

    # copa de arboles arriba
    for col, base_y, ry in (((52, 108, 44), 48, 30), ((36, 82, 26), 28, 28)):
        for x in range(-40, W + 80, 58):
            cx = x + int(rng.integers(-12, 12))
            cy = base_y - int(rng.integers(0, 14))
            cv2.ellipse(img, (cx, cy), (62, ry), 0, 0, 360, col, -1, cv2.LINE_AA)
    for x in range(-10, W + 10, 26):
        h = int(rng.integers(8, 22))
        cv2.fillPoly(img, [np.array([(x - 9, 44), (x + 9, 44), (x, 44 + h)], np.int32)],
                     (36, 82, 26), cv2.LINE_AA)
    for _ in range(50):
        cv2.circle(img, (int(rng.integers(0, W)), int(rng.integers(2, 40))), 3, (60, 125, 50), -1, cv2.LINE_AA)

    # terreno: tierra + pasto
    xs = list(range(0, W + 1, 4))
    top_pts = [(x, _terrain_y(x)) for x in xs]
    cv2.fillPoly(img, [np.array(top_pts + [(W, H), (0, H)], np.int32)], (55, 78, 112), cv2.LINE_AA)
    for _ in range(160):
        x = int(rng.integers(0, W))
        y = int(rng.integers(_terrain_y(x) + 18, H))
        cv2.ellipse(img, (x, y), (int(rng.integers(3, 9)), int(rng.integers(2, 5))), 0, 0, 360,
                    (40, 60, 92), -1, cv2.LINE_AA)
    band = top_pts + [(x, _terrain_y(x) + 14 + (4 if (x // 8) % 2 else 0)) for x in reversed(xs)]
    cv2.fillPoly(img, [np.array(band, np.int32)], (45, 185, 110), cv2.LINE_AA)
    cv2.polylines(img, [np.array(top_pts, np.int32)], False, (95, 225, 160), 3, cv2.LINE_AA)

    # arbustos
    def bush(cx, r):
        cy = _terrain_y(cx) - int(r * 0.35)
        parts = ((-0.9, 0.2, 0.75), (0.9, 0.25, 0.7), (0.0, -0.2, 1.0), (-0.3, 0.3, 0.9))
        for dx, dy, rr in parts:
            cv2.circle(img, (cx + int(dx * r), cy + int(dy * r)), int(rr * r), (70, 105, 30), -1, cv2.LINE_AA)
        for dx, dy, rr in parts:
            cv2.circle(img, (cx + int(dx * r) - 3, cy + int(dy * r) - 4), int(rr * r * 0.55),
                       (90, 150, 60), -1, cv2.LINE_AA)

    for bx, br in ((48, 20), (100, 15), (240, 18), (395, 20), (760, 22)):
        bush(bx, br)

    # hongos rojos
    for mx in (128, 214):
        my = _terrain_y(mx)
        cv2.rectangle(img, (mx - 3, my - 12), (mx + 3, my), (225, 235, 245), -1)
        cv2.ellipse(img, (mx, my - 12), (12, 9), 0, 180, 360, (50, 50, 215), -1, cv2.LINE_AA)
        cv2.circle(img, (mx - 4, my - 16), 2, (255, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(img, (mx + 4, my - 15), 2, (255, 255, 255), -1, cv2.LINE_AA)

    # panel translucido para la palabra y el teclado
    rrect(img, (300, 104), (786, 446), PAPER, r=22, border=(150, 180, 205), bw=3, alpha=0.86)
    return img


def _build_base_bg():
    rng = np.random.default_rng(7)
    bg = np.empty((H, W, 3), np.int16)
    bg[:] = KRAFT
    bg += rng.integers(-9, 9, (H, W, 1), dtype=np.int16)
    bg = np.clip(bg, 0, 255).astype(np.uint8)
    rrect(bg, (16, 18), (W - 10, H - 8), (0, 0, 0), r=10, alpha=0.25)
    rrect(bg, (12, 12), (W - 16, H - 14), PAPER, r=10)
    for y in range(52, H - 16, 28):
        cv2.line(bg, (14, y), (W - 18, y), RULE, 1, cv2.LINE_AA)
    cv2.line(bg, (62, 14), (62, H - 16), MARGIN, 1, cv2.LINE_AA)
    for x, y, r, c in ((772, 40, 6, PINK), (32, 452, 5, ORANGE), (776, 446, 7, CYAN), (780, 76, 4, YELLOW)):
        cv2.circle(bg, (x, y), r, c, -1, cv2.LINE_AA)
        cv2.circle(bg, (x - r // 3, y - r // 3), max(1, r // 4), _lighten(c, 80), -1, cv2.LINE_AA)
    star(bg, 746, 444, 9, BLUE)
    return bg


# ---------------------------------------------
#  Boton
# ---------------------------------------------
class Button:
    def __init__(self, x, y, w, h, text, color=ORANGE, text_color=WHITE, font_scale=0.8, kind='pill'):
        self.rect = (x, y, w, h)
        self.text = ascii_safe(text)
        self.color = color
        self.text_color = text_color
        self.font_scale = font_scale
        self.kind = kind
        self.hovered = False

    def draw(self, frame):
        x, y, w, h = self.rect
        if self.kind == 'tile':
            self._draw_tile(frame, x, y, w, h)
        else:
            self._draw_pill(frame, x, y, w, h)

    def _draw_pill(self, f, x, y, w, h):
        if self.hovered:
            y -= 3
        rrect(f, (x, y + 5), (x + w, y + h + 5), _mul(self.color, 0.55), r=h // 2)
        body = _lighten(self.color, 25) if self.hovered else self.color
        rrect(f, (x, y), (x + w, y + h), body, r=h // 2, border=_mul(self.color, 0.75), bw=2)
        rrect(f, (x + h // 3, y + 5), (x + w - h // 3, y + h // 2 - 1), WHITE, r=max(2, h // 6), alpha=0.22)
        (tw, th), _ = cv2.getTextSize(self.text, FONT, self.font_scale, 2)
        put(f, self.text, x + w // 2, y + (h + th) // 2, self.font_scale, self.text_color, 2,
            center=True, shadow=_mul(self.color, 0.45))

    def _draw_tile(self, f, x, y, w, h):
        if self.hovered:
            y -= 2
        rrect(f, (x + 1, y + 5), (x + w + 1, y + h + 5), (0, 0, 0), r=12, alpha=0.18)
        border = (60, 150, 235) if self.hovered else (200, 214, 222)
        rrect(f, (x, y), (x + w, y + h), (248, 252, 254), r=12, border=border, bw=2)
        cx, cy = x + w // 2, y + h // 2
        rad = 22 + (2 if self.hovered else 0)
        cv2.circle(f, (cx, cy + 2), rad, _mul(self.color, 0.55), -1, cv2.LINE_AA)
        cv2.circle(f, (cx, cy), rad, self.color, -1, cv2.LINE_AA)
        cv2.circle(f, (cx - int(rad * 0.55), cy - int(rad * 0.55)), 3, _lighten(self.color, 90), -1, cv2.LINE_AA)
        (tw, th), _ = cv2.getTextSize(self.text, FONT, self.font_scale, 2)
        cv2.putText(f, self.text, (cx - tw // 2, cy + th // 2), FONT, self.font_scale,
                    self.text_color, 2, cv2.LINE_AA)

    def is_clicked(self, mx, my):
        x, y, w, h = self.rect
        return x <= mx <= x + w and y <= my <= y + h


# ---------------------------------------------
#  Musica de fondo + controles de audio
# ---------------------------------------------
MUSIC_FILE = os.path.expanduser('~/robotis_ws/src/YAREN2/yaren_juegos/sounds/BANANA.mp3')
MUSIC_DEFAULT_VOLUME = 0.5


class MusicPlayer:
    """Musica de fondo en bucle (pygame.mixer). Si falta pygame o el archivo, el juego sigue sin musica."""

    def __init__(self, path, logger=None, volume=MUSIC_DEFAULT_VOLUME):
        self.path = path
        self.logger = logger
        self.volume = volume
        self.muted = False
        self._playing = False
        self._lock = threading.Lock()

    def _warn(self, msg):
        if self.logger is not None:
            self.logger.warning(msg)

    def _effective(self):
        return 0.0 if self.muted else self.volume

    def play(self):
        with self._lock:
            if self._playing:
                return
            try:
                if not os.path.isfile(self.path):
                    self._warn(f'Musica no encontrada: {self.path}')
                    return
                os.environ.setdefault('PYGAME_HIDE_SUPPORT_PROMPT', '1')
                import pygame
                if not pygame.mixer.get_init():
                    pygame.mixer.init()
                pygame.mixer.music.load(self.path)
                pygame.mixer.music.set_volume(self._effective())
                pygame.mixer.music.play(loops=-1, fade_ms=1000)
                self._playing = True
            except Exception as e:
                self._warn(f'No se pudo reproducir la musica (pip3 install pygame?): {e}')

    def stop(self):
        with self._lock:
            if not self._playing:
                return
            self._playing = False
            try:
                import pygame
                pygame.mixer.music.stop()
                pygame.mixer.quit()      # libera el dispositivo de audio para el resto del robot
            except Exception:
                pass

    def _apply(self):
        with self._lock:
            if not self._playing:
                return
            try:
                import pygame
                pygame.mixer.music.set_volume(self._effective())
            except Exception:
                pass

    def toggle_mute(self):
        self.muted = not self.muted
        self._apply()

    def change_volume(self, delta):
        self.volume = round(max(0.0, min(1.0, self.volume + delta)), 2)
        if delta > 0:
            self.muted = False       # subir el volumen quita el mute
        self._apply()


def draw_speaker(img, cx, cy, color, muted, level=1.0, s=1.0):
    """Icono de bocina (escalable): con ondas segun el volumen, o con una X si esta muteado."""
    def P(dx, dy):
        return (int(round(cx + dx * s)), int(round(cy + dy * s)))
    cone = np.array([P(-9, -4), P(-4, -4), P(3, -10), P(3, 10), P(-4, 4), P(-9, 4)], np.int32)
    cv2.fillPoly(img, [cone], color, cv2.LINE_AA)
    th = max(1, int(round(2.5 * s)))
    if muted:
        cv2.line(img, P(8, -5), P(16, 5), color, th, cv2.LINE_AA)
        cv2.line(img, P(16, -5), P(8, 5), color, th, cv2.LINE_AA)
    else:
        waves = 1 if level < 0.34 else (2 if level < 0.67 else 3)
        for i in range(waves):
            r = max(2, int(round((6 + 4 * i) * s)))
            cv2.ellipse(img, P(3, 0), (r, r), 0, -50, 50, color, th, cv2.LINE_AA)


# ---------------------------------------------
#  Nodo principal
# ---------------------------------------------
class PalabrasNode(LifecycleNode):

    # Geometria de la tarjeta de resultados y de las zonas de contenido
    CARD = (110, 70, 690, 420)
    CONTENT_CX = 490          # centro horizontal del texto/botones a la derecha del mono
    BTN_W = 250
    # Control de audio: boton de bocina en la barra superior + una tira que se abre hacia la izquierda.
    # Todo queda dentro de y=13..57, asi que nunca tapa titulos, tarjetas ni la palabra.
    AUD_C = (522, 35)
    AUD_R = 18
    AUD_STRIP = (334, 13, 544, 57)
    AUD_MINUS = (358, 35, 13)
    AUD_PLUS = (446, 35, 13)
    AUD_MUTE = (480, 35, 14)
    AUD_BARS_X = 378

    def __init__(self):
        super().__init__('palabras_node')
        self.get_logger().info('PalabrasNode created')
        self._active = False
        self.is_english = False
        self.language = 'es'
        self._lang_sub = None
        self._mode_pub = None
        self._game_active_pub = None
        self.show_exit_confirm = False
        self._lock = threading.RLock()
        self._frame_count = 0

        self._bg_base = _build_base_bg()
        self._bg_jungle = _build_jungle_bg()
        self._music = MusicPlayer(MUSIC_FILE, self.get_logger())

        rnd = random.Random(5)
        palette = [BLUE, PINK, YELLOW, CYAN, ORANGE, GREEN]
        self._confetti = []
        while len(self._confetti) < 30:
            x, y = rnd.randint(20, 780), rnd.randint(20, 460)
            r = rnd.randint(4, 8)
            self._confetti.append((x, y, r, rnd.choice(palette), rnd.random() < 0.3))

        self._reset_game_state()

    # ------------- Lifecycle -------------
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring palabras_node...')
        self._game_active_pub = self.create_publisher(Bool, '/yaren/game_active', 10)
        self._mode_pub = self.create_publisher(String, '/yaren_mode', 1)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._lang_sub = self.create_subscription(Bool, '/yaren/is_english', self._lang_cb, qos)
        with self._lock:
            self._reset_game_state()
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating palabras_node...')
        with self._lock:
            self._reset_game_state()
        self._active = True
        self._game_active_pub.publish(Bool(data=True))
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating palabras_node...')
        self._active = False
        self._music.stop()
        self._game_active_pub.publish(Bool(data=False))
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        if self._lang_sub:
            self.destroy_subscription(self._lang_sub)
            self._lang_sub = None
        if self._mode_pub:
            self.destroy_publisher(self._mode_pub)
            self._mode_pub = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._active = False
        return TransitionCallbackReturn.SUCCESS

    def _exit_to_menu(self):
        self._active = False
        if self._mode_pub is not None:
            try:
                self._mode_pub.publish(String(data='idle'))
            except Exception as e:
                self.get_logger().warning(f'No se pudo publicar idle: {e}')

    def _lang_cb(self, msg):
        with self._lock:
            self.is_english = msg.data
            self.language = 'en' if self.is_english else 'es'
            self._rebuild_buttons_for_screen()
            self._build_confirm_buttons()

    # ------------- Estado del juego -------------
    def _reset_game_state(self):
        self.screen = 'intro'
        self.language = 'en' if self.is_english else 'es'
        self.level = 'facil'
        self.word_list = []
        self.word_index = 0
        self.current_word = ''
        self.current_def = ''
        self.guessed = set()
        self.errors = 0
        self.total_errors = 0
        self.show_hint_text = False
        self.max_lives = 6
        self.letters_shown = []
        self.remaining_pool = []
        self.hint_used = False
        self.hint_letters = set()
        self.buttons = {}
        self.show_exit_confirm = False
        self.audio_open = False
        self._audio_t = 0
        self._build_intro_buttons()
        self._build_confirm_buttons()

    def _t(self, es_text, en_text):
        return en_text if self.language == 'en' else es_text

    def _go_to_levels(self):
        self.screen = 'level'
        self.show_hint_text = False
        self._build_level_buttons()

    def _start_level(self):
        cfg = LEVEL_CONFIG[self.level]
        bank_keys = list(WORD_BANK[self.language][self.level].keys())
        random.shuffle(bank_keys)
        self.word_list = bank_keys[:min(len(bank_keys), cfg['words'])]
        self.word_index = 0
        self.total_errors = 0
        self._load_word()

    def _load_word(self):
        self.current_word = self.word_list[self.word_index]
        self.current_def = WORD_BANK[self.language][self.level][self.current_word]
        kid = KID_DEFS[self.language].get(self.current_word)
        if self.level == 'facil' and kid:
            self.current_def = kid
        self.guessed = set()
        self.errors = 0
        self.show_hint_text = False
        self.max_lives = LEVEL_CONFIG[self.level]['lives']
        self.hint_used = False
        self.hint_letters = set()
        self._generate_letters()
        self.screen = 'playing'
        self._build_letter_buttons()

    def _generate_letters(self):
        word_letters = list(set(self.current_word))
        fillers = [l for l in ALPHABET if l not in word_letters]
        random.shuffle(word_letters)
        random.shuffle(fillers)

        num_correct_initial = min(3, len(word_letters))
        initial = word_letters[:num_correct_initial] + fillers[:10 - num_correct_initial]
        random.shuffle(initial)

        self.letters_shown = initial
        self.remaining_pool = word_letters[num_correct_initial:] + fillers[10 - num_correct_initial:]
        random.shuffle(self.remaining_pool)

    def _check_win(self):
        return all(ch in self.guessed or ch == ' ' for ch in self.current_word)

    def _process_guess(self, letter):
        if self.screen != 'playing' or letter in self.guessed:
            return
        self.guessed.add(letter)
        word = self.current_word
        if letter not in word:
            self.errors += 1
            self.total_errors += 1

        # Reemplazar la tecla usada por una nueva (garantizando que siempre haya una correcta visible)
        try:
            idx = self.letters_shown.index(letter)
        except ValueError:
            idx = None

        if idx is not None:
            self.letters_shown[idx] = ''
            correct_on_screen = sum(1 for l in self.letters_shown if l != '' and l in word)
            unguessed_correct = [l for l in set(word)
                                 if l not in self.guessed and l not in self.letters_shown and l != ' ']
            new_letter = ''
            if correct_on_screen == 0 and unguessed_correct:
                new_letter = unguessed_correct[0]
                if new_letter in self.remaining_pool:
                    self.remaining_pool.remove(new_letter)
            elif self.remaining_pool:
                new_letter = self.remaining_pool.pop(0)
            self.letters_shown[idx] = new_letter

        self._build_letter_buttons()

        if self._check_win():
            if self.word_index >= len(self.word_list) - 1:
                self.screen = 'level_complete'
                self._build_level_complete_buttons()
            else:
                self.screen = 'word_result'
                self._build_word_result_buttons()
        elif self.errors >= self.max_lives:
            self.screen = 'game_over'
            self._build_game_over_buttons()

    def _use_hint(self):
        if self.screen != 'playing':
            return
        if self.level == 'facil':
            self.show_hint_text = True
            return
        if self.hint_used:
            return
        self.hint_used = True
        self._reveal_letters(2 if self.level == 'medio' else 1)
        self._build_letter_buttons()

    def _reveal_letters(self, n):
        word = self.current_word
        unique = [c for c in dict.fromkeys(word) if c != ' ' and c not in self.guessed]
        n = min(n, len(unique) - 1)          # nunca revela la ultima letra
        if n <= 0:
            return
        once = [c for c in unique if word.count(c) == 1]
        pool = once if len(once) >= n else unique
        for l in random.sample(pool, n):
            self.guessed.add(l)
            self.hint_letters.add(l)
            if l in self.remaining_pool:
                self.remaining_pool.remove(l)
            if l in self.letters_shown:
                i = self.letters_shown.index(l)
                self.letters_shown[i] = self.remaining_pool.pop(0) if self.remaining_pool else ''
        self._ensure_correct_visible()

    def _ensure_correct_visible(self):
        word = self.current_word
        missing = [l for l in set(word) if l != ' ' and l not in self.guessed]
        if not missing:
            return
        if any(l and l in word for l in self.letters_shown):
            return
        new = missing[0]
        if new in self.remaining_pool:
            self.remaining_pool.remove(new)
        for i, l in enumerate(self.letters_shown):
            if l == '':
                self.letters_shown[i] = new
                return
        for i, l in enumerate(self.letters_shown):
            if l not in word:
                self.remaining_pool.append(l)
                self.letters_shown[i] = new
                return

    def _draw_leaves(self, frame):
        t = self._frame_count
        cols = ((60, 170, 70), (50, 140, 100), (80, 190, 120))
        for i in range(7):
            x = 25 + (i * 41) % 260 + int(14 * math.sin(t / 25.0 + i))
            y = int((i * 83 + t * (0.8 + 0.15 * (i % 3))) % 340) + 70
            ang = int(40 * math.sin(t / 15.0 + i) + i * 30)
            cv2.ellipse(frame, (x, y), (9, 4), ang, 0, 360, cols[i % 3], -1, cv2.LINE_AA)

    # ------------- Construccion de botones -------------
    def _rebuild_buttons_for_screen(self):
        builders = {
            'intro': self._build_intro_buttons,
            'level': self._build_level_buttons,
            'playing': self._build_letter_buttons,
            'word_result': self._build_word_result_buttons,
            'level_complete': self._build_level_complete_buttons,
            'game_over': self._build_game_over_buttons,
        }
        fn = builders.get(self.screen)
        if fn:
            fn()

    def _stack_buttons(self, specs, y0, h=48, gap=12, font_scale=0.72):
        """Apila botones verticalmente a la derecha del mono. specs = [(key, texto, color), ...]"""
        self.buttons = {}
        for i, (key, text, color) in enumerate(specs):
            x = self.CONTENT_CX - self.BTN_W // 2
            y = y0 + i * (h + gap)
            self.buttons[key] = Button(x, y, self.BTN_W, h, text, color, WHITE, font_scale)

    def _build_intro_buttons(self):
        self.buttons = {
            'start': Button(362, 372, 170, 56, self._t('EMPEZAR', 'START'), ORANGE, WHITE, 0.95),
            'back': Button(548, 372, 170, 56, self._t('VOLVER', 'BACK'), BLUE, WHITE, 0.95),
        }

    def _build_level_buttons(self):
        self.buttons = {}
        for lvl, cx in (('facil', 160), ('medio', 400), ('dificil', 640)):
            self.buttons[lvl] = Button(cx - 80, 326, 160, 44, self._t('JUGAR', 'PLAY'),
                                       LEVEL_CONFIG[lvl]['color'], WHITE, 0.8)
        self.buttons['back'] = Button(W // 2 - 90, 402, 180, 48, self._t('VOLVER', 'BACK'), BLUE, WHITE, 0.85)

    def _build_letter_buttons(self):
        self.buttons = {}
        btn_y = 15
        hint_off = (self.level != 'facil' and self.hint_used)
        self.buttons['hint'] = Button(W - 130, btn_y, 110, 42, self._t('PISTA', 'HINT'), GRAY_DK if hint_off else PURPLE, WHITE, 0.7)
        self.buttons['levels_ingame'] = Button(W - 250, btn_y, 110, 42, self._t('NIVELES', 'LEVELS'), BLUE, WHITE, 0.7)

        cols = 5
        btn_w, btn_h = 78, 62
        gap_x, gap_y = 12, 12
        total_w = cols * btn_w + (cols - 1) * gap_x
        start_x = 310 + (460 - total_w) // 2
        start_y = 300

        for i, letter in enumerate(self.letters_shown):
            if letter == '':
                continue
            row, col = i // cols, i % cols
            x = start_x + col * (btn_w + gap_x)
            y = start_y + row * (btn_h + gap_y)
            circle_c, text_c = KEY_STYLE
            self.buttons[f'letter_{letter}'] = Button(x, y, btn_w, btn_h, letter, circle_c, text_c, 0.95, kind='tile')

    def _build_word_result_buttons(self):
        self._stack_buttons([('next', self._t('Siguiente ->', 'Next ->'), GREEN)], y0=318, h=52, font_scale=0.85)

    def _build_level_complete_buttons(self):
        is_last = self.level == LEVEL_ORDER[-1]
        first = ('restart', self._t('Jugar otra vez', 'Play again'), ORANGE) if is_last \
            else ('next_level', self._t('Siguiente nivel', 'Next level'), GREEN)
        self._stack_buttons([first, ('levels', self._t('Elegir nivel', 'Choose level'), BLUE)],
                            y0=302, h=44, gap=10)

    def _build_game_over_buttons(self):
        self._stack_buttons([('restart', self._t('Reiniciar nivel', 'Restart level'), ORANGE),
                             ('levels', self._t('Elegir nivel', 'Choose level'), BLUE)],
                            y0=270, h=48, gap=12)

    def _build_confirm_buttons(self):
        btn_w, btn_h = 130, 48
        self.confirm_buttons = {
            'no': Button(W // 2 - btn_w - 10, H // 2 + 14, btn_w, btn_h, 'NO', GREEN, WHITE, 0.85),
            'yes': Button(W // 2 + 10, H // 2 + 14, btn_w, btn_h, self._t('SI', 'YES'), RED, WHITE, 0.85),
        }

    # ------------- Entrada (mouse / tactil / teclado) -------------
    @staticmethod
    def _hint_geometry():
        pw, ph = 600, 260
        return (W - pw) // 2, (H - ph) // 2, pw, ph

    def _on_mouse(self, event, x, y, flags, param):
        with self._lock:
            self._handle_mouse(event, x, y)

    def _handle_mouse(self, event, x, y):
        if event == cv2.EVENT_MOUSEMOVE:
            if self.show_exit_confirm:
                for btn in self.confirm_buttons.values():
                    btn.hovered = btn.is_clicked(x, y)
                return
            if self.screen == 'playing' and self.show_hint_text:
                return
            for btn in self.buttons.values():
                btn.hovered = btn.is_clicked(x, y)
            return

        if event != cv2.EVENT_LBUTTONDOWN:
            return

        # Confirmacion de salida
        if self.show_exit_confirm:
            if self.confirm_buttons['yes'].is_clicked(x, y):
                self.show_exit_confirm = False
                self._exit_to_menu()
            elif self.confirm_buttons['no'].is_clicked(x, y):
                self.show_exit_confirm = False
            return

        # Panel de pista: cierra con la X o tocando fuera del panel
        if self.screen == 'playing' and self.show_hint_text:
            px, py, pw, ph = self._hint_geometry()
            cx, cy = px + pw - 35, py + 35
            on_x = (x - cx) ** 2 + (y - cy) ** 2 <= 30 ** 2
            outside = not (px <= x <= px + pw and py <= y <= py + ph)
            if on_x or outside:
                self.show_hint_text = False
            return

        # Controles de audio (boton de bocina + panel de volumen)
        if self._handle_audio_click(x, y):
            return

        # Esquina superior izquierda = cerrar
        if x < 60 and y < 60:
            if self.screen == 'intro':
                self._exit_to_menu()
            else:
                self.show_exit_confirm = True
            return

        clicked = self._clicked_button(x, y)
        if clicked is None:
            return

        if self.screen == 'intro':
            if clicked == 'start':
                self._go_to_levels()
            elif clicked == 'back':
                self._exit_to_menu()

        elif self.screen == 'level':
            if clicked in LEVEL_ORDER:
                self.level = clicked
                self._start_level()
            elif clicked == 'back':
                self.screen = 'intro'
                self._build_intro_buttons()

        elif self.screen == 'playing':
            if clicked == 'levels_ingame':
                self._go_to_levels()
            elif clicked == 'hint':
                self._use_hint()
            elif clicked.startswith('letter_'):
                self._process_guess(clicked[len('letter_'):])

        elif self.screen == 'word_result':
            if clicked == 'next':
                self.word_index += 1
                self._load_word()

        elif self.screen == 'level_complete':
            if clicked == 'next_level':
                idx = LEVEL_ORDER.index(self.level)
                if idx < len(LEVEL_ORDER) - 1:
                    self.level = LEVEL_ORDER[idx + 1]
                    self._start_level()
                else:
                    self._go_to_levels()
            elif clicked == 'restart':
                self._start_level()
            elif clicked == 'levels':
                self._go_to_levels()

        elif self.screen == 'game_over':
            if clicked == 'restart':
                self._start_level()
            elif clicked == 'levels':
                self._go_to_levels()

    def _clicked_button(self, x, y):
        for key, btn in list(self.buttons.items()):
            if btn.is_clicked(x, y):
                return key
        return None

    def _handle_key(self, key):
        """Teclado fisico: letras A-Z juegan la letra (util para pruebas sin pantalla tactil)."""
        if key == 255 or key == -1:
            return
        ch = chr(key).upper() if 32 <= key < 127 else ''
        with self._lock:
            if (ch.isalpha() and self.screen == 'playing' and not self.show_hint_text
                    and not self.show_exit_confirm and f'letter_{ch}' in self.buttons):
                self._process_guess(ch)

    # ------------- Audio -------------
    @staticmethod
    def _in_circle(x, y, c, extra=6):
        return (x - c[0]) ** 2 + (y - c[1]) ** 2 <= (c[2] + extra) ** 2

    def _handle_audio_click(self, x, y):
        if self._in_circle(x, y, (self.AUD_C[0], self.AUD_C[1], self.AUD_R), 6):
            self.audio_open = not self.audio_open
            self._audio_t = self._frame_count
            return True
        if not self.audio_open:
            return False
        x1, y1, x2, y2 = self.AUD_STRIP
        if x1 <= x <= x2 and y1 <= y <= y2:
            self._audio_t = self._frame_count
            if self._in_circle(x, y, self.AUD_MUTE, 3):
                self._music.toggle_mute()
            elif self._in_circle(x, y, self.AUD_MINUS, 3):
                self._music.change_volume(-0.1)
            elif self._in_circle(x, y, self.AUD_PLUS, 3):
                self._music.change_volume(0.1)
            return True
        self.audio_open = False      # toque fuera de la tira: se cierra y el toque sigue su curso
        return False

    def _draw_audio(self, frame):
        m = self._music
        if self.audio_open and self._frame_count - self._audio_t > FPS * 4:
            self.audio_open = False  # se esconde sola tras 4 s sin tocarla

        # animacion de apertura/cierre de la tira
        target = 1.0 if self.audio_open else 0.0
        a = getattr(self, '_aud_p', 0.0)
        if a < target:
            a = min(target, a + 0.2)
        elif a > target:
            a = max(target, a - 0.2)
        self._aud_p = a

        cx, cy = self.AUD_C
        col = GRAY_DK if m.muted else BLUE

        if a > 0.0:
            x1, y1, x2, y2 = self.AUD_STRIP
            left = int(x2 - (x2 - x1) * (a * (2.0 - a)))
            self._paper_card(frame, left, y1, x2, y2, border=col, r=22)
            if a >= 0.99:
                mx, my, mr = self.AUD_MUTE
                mcol = GRAY_DK if m.muted else ORANGE
                cv2.circle(frame, (mx, my + 2), mr, _mul(mcol, 0.55), -1, cv2.LINE_AA)
                cv2.circle(frame, (mx, my), mr, mcol, -1, cv2.LINE_AA)
                draw_speaker(frame, mx - 3, my, WHITE, m.muted, m.volume, 0.75)

                for (bx, by, br), plus, ok in ((self.AUD_MINUS, False, m.volume > 0.0),
                                               (self.AUD_PLUS, True, m.volume < 1.0)):
                    bcol = PURPLE if ok else GRAY_OFF
                    cv2.circle(frame, (bx, by + 2), br, _mul(bcol, 0.55), -1, cv2.LINE_AA)
                    cv2.circle(frame, (bx, by), br, bcol, -1, cv2.LINE_AA)
                    cv2.line(frame, (bx - 5, by), (bx + 5, by), WHITE, 3, cv2.LINE_AA)
                    if plus:
                        cv2.line(frame, (bx, by - 5), (bx, by + 5), WHITE, 3, cv2.LINE_AA)

                filled = 0 if m.muted else int(round(m.volume * 10))
                for i in range(10):
                    x = self.AUD_BARS_X + i * 5
                    hh = 5 + i
                    cv2.rectangle(frame, (x, 42 - hh), (x + 3, 42), GREEN if i < filled else GRAY_OFF, -1)

        # boton principal (siempre visible, encima de la tira)
        r = self.AUD_R
        cv2.circle(frame, (cx, cy + 3), r, _mul(col, 0.55), -1, cv2.LINE_AA)
        cv2.circle(frame, (cx, cy), r, col, -1, cv2.LINE_AA)
        cv2.circle(frame, (cx, cy), r, WHITE, 2, cv2.LINE_AA)
        draw_speaker(frame, cx - 4, cy, WHITE, m.muted, m.volume, 0.9)

    # ------------- Render -------------
    def _title(self, frame, text, cx, y, color, scale=1.4, thick=3):
        put(frame, text, cx, y, scale, color, thick, center=True, shadow=PAPER_DK)

    def _paper_card(self, frame, x1, y1, x2, y2, border=(150, 180, 205), r=22):
        rrect(frame, (x1 + 4, y1 + 8), (x2 + 4, y2 + 8), (0, 0, 0), r=r, alpha=0.22)
        rrect(frame, (x1, y1), (x2, y2), (250, 253, 255), r=r, border=border, bw=3)

    def _draw_confetti(self, frame):
        t = self._frame_count
        for x, y, r, col, is_star in self._confetti:
            yy = int((y + t * (0.4 + r * 0.12)) % (H - 20)) + 10
            xx = int(x + 6 * math.sin((t + x) / 18.0))
            if is_star:
                star(frame, xx, yy, r + 3, col)
            else:
                cv2.circle(frame, (xx, yy), r, col, -1, cv2.LINE_AA)
                cv2.circle(frame, (xx - r // 3, yy - r // 3), max(1, r // 4), _lighten(col, 80), -1, cv2.LINE_AA)

    def _draw_scene_panel(self, frame, x1, y1, x2, y2):
        """Tarjeta con cielo y pasto para el mono de la intro."""
        self._paper_card(frame, x1, y1, x2, y2, border=BROWN, r=22)
        rrect(frame, (x1 + 5, y1 + 5), (x2 - 5, y2 - 5), (250, 228, 190), r=18)           # cielo
        ground_y = y2 - 48
        grass = (110, 200, 110)
        rrect(frame, (x1 + 5, ground_y), (x2 - 5, y2 - 5), grass, r=18)                   # pasto
        cv2.rectangle(frame, (x1 + 5, ground_y), (x2 - 5, ground_y + 20), grass, -1)      # esquinas de arriba rectas
        cv2.line(frame, (x1 + 5, ground_y), (x2 - 5, ground_y), (70, 160, 70), 3, cv2.LINE_AA)
        cloud(frame, x1 + 25, y1 + 85, 1.3)
        cloud(frame, x2 - 85, y1 + 200, 1.2)
        cloud(frame, x1 + 20, y1 + 190, 0.8)

    def _render_intro(self, frame):
        frame[:] = self._bg_base
        self._draw_scene_panel(frame, 30, 62, 340, 440)
        draw_monkey_scene(frame, 170, 402, 285, 'climb', 0.65, self._frame_count, ms=1.3)
        speech_bubble(frame, 275, 175, 'HI!' if self.is_english else 'HOLA!', (178, 190))

        cx = 540
        if self.is_english:
            lines = (('THE', INK), ('word', TEAL), ('GAME', PURPLE))
        else:
            lines = (('EL JUEGO', INK), ('de las', TEAL), ('PALABRAS', PURPLE))

        self._title(frame, lines[0][0], cx, 115, lines[0][1], 2.0, 5)
        put(frame, lines[1][0], cx, 165, 1.8, lines[1][1], 2, center=True, font=SCRIPT)
        self._title(frame, lines[2][0], cx, 235, lines[2][1], 1.8, 5)

        for i in range(8):
            x = cx - 192 + i * 48
            rrect(frame, (x, 262), (x + 36, 270), BROWN, r=4)
        pencil(frame, 688, 168, -45, 0.95)

        sub = 'Guess the word letter by letter' if self.is_english else 'Adivina la palabra letra por letra'
        put(frame, sub, cx, 318, 0.62, INK_DIM, 1, center=True)

        for btn in self.buttons.values():
            btn.draw(frame)

    def _render_level(self, frame):
        frame[:] = self._bg_base
        self._title(frame, self._t('ELIGE TU NIVEL', 'CHOOSE YOUR LEVEL'), W // 2, 84, INK, 1.4, 3)
        for i in range(7):
            x = W // 2 - 126 + i * 38
            rrect(frame, (x, 98), (x + 26, 104), BROWN, r=3)

        for i, (lvl, cx) in enumerate((('facil', 160), ('medio', 400), ('dificil', 640))):
            cfg = LEVEL_CONFIG[lvl]
            col = cfg['color']
            x1, x2, y1, y2 = cx - 105, cx + 105, 118, 384
            self._paper_card(frame, x1, y1, x2, y2, border=col, r=18)
            rrect(frame, (x1 + 3, y1 + 3), (x2 - 3, y1 + 58), col, r=15)
            cv2.rectangle(frame, (x1 + 3, y1 + 34), (x2 - 3, y1 + 58), col, -1)
            put(frame, cfg[f'label_{self.language}'], cx, y1 + 40, 0.95, WHITE, 2, center=True,
                shadow=_mul(col, 0.5))

            for s in range(3):
                on = s <= i
                star(frame, cx + (s - 1) * 40, y1 + 94, 15, YELLOW if on else GRAY_OFF,
                     outline=_mul(YELLOW, 0.6) if on else GRAY_DK)

            nw, nl = cfg['words'], cfg['lives']
            words = f"{nw} " + (self._t('palabras', 'words') if nw > 1 else self._t('palabra', 'word'))
            put(frame, words, cx, y1 + 138, 0.62, INK, 1, center=True)
            for h in range(nl):
                heart(frame, cx + int((h - (nl - 1) / 2) * 24), y1 + 162, 9, RED, outline=RED_DK)
            put(frame, f"{nl} " + self._t('vidas', 'lives'), cx, y1 + 198, 0.55, INK_DIM, 1, center=True)

        for btn in self.buttons.values():
            btn.draw(frame)
        draw_close_button(frame)

    def _render_hint_panel(self, frame):
        px, py, pw, ph = self._hint_geometry()

        # Fondo oscurecido
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (W, H), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)

        # Panel
        rrect(frame, (px + 4, py + 8), (px + pw + 4, py + ph + 8), (0, 0, 0), r=18, alpha=0.3)
        rrect(frame, (px, py), (px + pw, py + ph), (252, 248, 220), r=18, border=PURPLE, bw=4)

        # Titulo y separador
        put(frame, self._t('PISTA', 'HINT'), px + pw // 2, py + 48, 1.1, PURPLE, 3, center=True, shadow=PAPER_DK)
        cv2.line(frame, (px + 60, py + 66), (px + pw - 60, py + 66), PURPLE, 2, cv2.LINE_AA)

        # Boton cerrar (X) arriba a la derecha
        draw_close_button(frame, c=(px + pw - 35, py + 35))

        # Definicion auto-ajustada y centrada
        draw_fitted_text(frame, self.current_def, px + pw // 2, py + 76, pw - 80, ph - 76 - 20, INK, 2)

    def _render_playing(self, frame):
        frame[:] = self._bg_jungle
        cfg = LEVEL_CONFIG[self.level]
        draw_close_button(frame)

        label = cfg[f'label_{self.language}']
        (tw, th), _ = cv2.getTextSize(label, FONT, 0.7, 2)
        rrect(frame, (64, 13), (64 + tw + 28, 47), cfg['color'], r=17, border=_mul(cfg['color'], 0.7), bw=2)
        put(frame, label, 64 + 14, 13 + (34 + th) // 2, 0.7, WHITE, 2, shadow=_mul(cfg['color'], 0.5))

        prog = f"{self._t('Palabra', 'Word')} {self.word_index + 1}/{len(self.word_list)}"
        pw_ = text_width(prog, 0.62, 1)
        rrect(frame, (64 + tw + 28 + 8, 16), (64 + tw + 28 + 14 + pw_ + 10, 46), WHITE, r=15, alpha=0.9)
        put(frame, prog, 64 + tw + 28 + 14, 38, 0.62, INK, 1)

        rrect(frame, (W - 38 - (self.max_lives - 1) * 28 - 20, 64), (W - 14, 98), WHITE, r=17, alpha=0.88)
        lives_left = self.max_lives - self.errors
        for i in range(self.max_lives):
            cx = W - 38 - (self.max_lives - 1 - i) * 28
            if i < lives_left:
                heart(frame, cx, 80, 9, RED, outline=RED_DK)
            else:
                heart(frame, cx, 80, 9, GRAY_OFF, outline=GRAY_DK)

        word = self.current_word
        unique_letters = {c for c in word if c != ' '}
        total_needed = len(unique_letters)
        correct_guesses = len([c for c in self.guessed if c in unique_letters])
        cf = correct_guesses / total_needed if total_needed > 0 else 0
        ef = self.errors / self.max_lives if self.max_lives > 0 else 0
        # El mono sube con los aciertos y baja con los errores
        p = max(0.0, min(1.0, cf - 0.5 * ef))
        self._draw_leaves(frame)
        draw_monkey_scene(frame, 170, 362, 250, 'climb', p, self._frame_count, ms=1.1, ground=False)

        # Errores: bloque blanco con una ranura por cada vida
        wrong = [l for l in sorted(self.guessed) if l not in word]
        bx1, by1, bx2, by2 = 52, 378, 288, 458
        rrect(frame, (bx1 + 3, by1 + 6), (bx2 + 3, by2 + 6), (0, 0, 0), r=16, alpha=0.2)
        rrect(frame, (bx1, by1), (bx2, by2), WHITE, r=16, border=(200, 214, 222), bw=2)
        put(frame, self._t('ERRORES', 'MISTAKES'), 170, 399, 0.5, INK_DIM, 1, center=True)
        slots = self.max_lives
        for i in range(slots):
            wx = 170 + int((i - (slots - 1) / 2) * 36)
            if i < len(wrong):
                cv2.circle(frame, (wx, 432), 15, RED_DK, -1, cv2.LINE_AA)
                cv2.circle(frame, (wx, 430), 15, RED, -1, cv2.LINE_AA)
                put(frame, wrong[i], wx, 436, 0.55, WHITE, 2, center=True)
            else:
                cv2.circle(frame, (wx, 430), 15, GRAY_OFF, 2, cv2.LINE_AA)

        # Palabra (guiones + letras acertadas)
        n = len(word)
        base_w, base_gap = 46, 10
        ideal = n * base_w + (n - 1) * base_gap
        sc = min(1.0, 460 / ideal) if ideal > 0 else 1.0
        slot_w, gap = int(base_w * sc), int(base_gap * sc)
        total = n * slot_w + (n - 1) * gap
        sx = 310 + (460 - total) // 2
        dash_y = 226
        for i, ch in enumerate(word):
            if ch == ' ':
                continue
            x = sx + i * (slot_w + gap)
            rrect(frame, (x, dash_y), (x + slot_w, dash_y + 7), BROWN, r=3)
            if ch in self.guessed:
                fs = 1.6 * sc
                (lw, lh), _ = cv2.getTextSize(ch, FONT, fs, 3)
                put(frame, ch, x + (slot_w - lw) // 2, dash_y - 12, fs, PURPLE if ch in self.hint_letters else INK, 3, shadow=PAPER_DK)

        put(frame, self._t('Toca una letra para adivinar', 'Touch a letter to guess'),
            540, 276, 0.58, INK_DIM, 1, center=True)
        for btn in self.buttons.values():
            btn.draw(frame)

        if self.show_hint_text:
            self._render_hint_panel(frame)

    def _result_base(self, frame, confetti, dead=False):
        frame[:] = self._bg_base
        if confetti:
            self._draw_confetti(frame)
        x1, y1, x2, y2 = self.CARD
        self._paper_card(frame, x1, y1, x2, y2)
        # Gana: el mono agarro la banana. Pierde: se cayo y quedo mareado.
        draw_monkey_scene(frame, 275, 405, 240, 'dizzy' if dead else 'win',
                          t=self._frame_count, ms=1.15)

    def _render_word_result(self, frame):
        self._result_base(frame, confetti=True)
        cx = self.CONTENT_CX
        self._title(frame, self._t('Correcto!', 'Correct!'), cx, 140, GREEN, 1.5, 3)
        put(frame, self._t('La palabra era:', 'The word was:'), cx, 190, 0.75, INK, 1, center=True)
        fs = min(1.3, 360 / max(1, text_width(self.current_word, 1.0, 3)))
        put(frame, self.current_word, cx, 242, fs, PURPLE, 3, center=True, shadow=PAPER_DK)

        nxt = self._t(f'Siguiente: palabra {self.word_index + 2}/{len(self.word_list)}',
                      f'Next: word {self.word_index + 2}/{len(self.word_list)}')
        put(frame, nxt, cx, 284, 0.55, INK_DIM, 1, center=True)

        for btn in self.buttons.values():
            btn.draw(frame)
        draw_close_button(frame)

    def _render_level_complete(self, frame):
        self._result_base(frame, confetti=True)
        cx = self.CONTENT_CX
        self._title(frame, self._t('NIVEL COMPLETADO!', 'LEVEL COMPLETE!'), cx, 128, GREEN, 1.1, 3)

        lvl_idx = LEVEL_ORDER.index(self.level)
        for s in range(3):
            on = s <= lvl_idx
            star(frame, cx + (s - 1) * 62, 176, 22, YELLOW if on else GRAY_OFF,
                 outline=_mul(YELLOW, 0.6) if on else GRAY_DK)

        cfg = LEVEL_CONFIG[self.level]
        msg = self._t(f'Completaste el nivel {cfg["label_es"]}', f'You completed the {cfg["label_en"]} level')
        put(frame, msg, cx, 226, 0.72, INK, 1, center=True)

        if lvl_idx < len(LEVEL_ORDER) - 1:
            nxt = LEVEL_CONFIG[LEVEL_ORDER[lvl_idx + 1]]
            hint = self._t(f'Quieres intentar el nivel {nxt["label_es"]}?', f'Try the {nxt["label_en"]} level?')
            put(frame, hint, cx, 256, 0.64, PURPLE, 2, center=True)
        else:
            put(frame, self._t('Eres un campeon!', 'You are a champion!'), cx, 256, 0.8, PURPLE, 2, center=True)

        put(frame, f"{self._t('Errores', 'Mistakes')}: {self.total_errors}", cx, 282, 0.55, INK_DIM, 1, center=True)

        for btn in self.buttons.values():
            btn.draw(frame)
        draw_close_button(frame)

    def _render_game_over(self, frame):
        self._result_base(frame, confetti=False, dead=True)
        cx = self.CONTENT_CX
        self._title(frame, 'GAME OVER!', cx, 135, RED, 1.4, 3)
        put(frame, self._t('La palabra era:', 'The word was:'), cx, 180, 0.7, INK, 1, center=True)
        fs = min(1.0, 360 / max(1, text_width(self.current_word, 1.0, 3)))
        put(frame, self.current_word, cx, 224, fs, PURPLE, 3, center=True, shadow=PAPER_DK)

        for btn in self.buttons.values():
            btn.draw(frame)
        draw_close_button(frame)

    def _render_confirm_overlay(self, frame):
        frame[:] = (frame * 0.35).astype(np.uint8)
        pw, ph = 440, 200
        px, py = (W - pw) // 2, (H - ph) // 2
        self._paper_card(frame, px, py, px + pw, py + ph, r=20)
        put(frame, self._t('Seguro que quieres salir?', 'Are you sure you want to quit?'),
            W // 2, py + 76, 0.8, INK, 1, center=True)
        for btn in self.confirm_buttons.values():
            btn.draw(frame)

    def _render(self, frame):
        renderers = {
            'intro': self._render_intro,
            'level': self._render_level,
            'playing': self._render_playing,
            'word_result': self._render_word_result,
            'level_complete': self._render_level_complete,
            'game_over': self._render_game_over,
        }
        fn = renderers.get(self.screen)
        if fn:
            fn(frame)
        if not self.show_exit_confirm and not (self.screen == 'playing' and self.show_hint_text):
            self._draw_audio(frame)
        if self.show_exit_confirm:
            self._render_confirm_overlay(frame)

    # ------------- Loop principal (hilo principal, requerido por OpenCV/Qt) -------------
    def run_display_main_thread(self):
        win_name = 'YAREN2 - Juego de las Palabras'

        while rclpy.ok():
            while not self._active and rclpy.ok():
                time.sleep(0.05)
            if not rclpy.ok():
                break

            cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
            cv2.setWindowProperty(win_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            cv2.setWindowProperty(win_name, cv2.WND_PROP_TOPMOST, 1)
            cv2.setMouseCallback(win_name, self._on_mouse)

            def _force_focus():
                time.sleep(0.5)
                try:
                    subprocess.run(["xdotool", "search", "--name", win_name, "windowactivate", "--sync",
                                    "windowfocus"], check=False, capture_output=True)
                except FileNotFoundError:
                    pass
            threading.Thread(target=_force_focus, daemon=True).start()

            with self._lock:
                self._reset_game_state()
            frame = np.zeros((H, W, 3), dtype=np.uint8)
            self._music.play()

            while self._active and rclpy.ok():
                try:
                    if cv2.getWindowProperty(win_name, cv2.WND_PROP_VISIBLE) < 1:
                        self._exit_to_menu()
                        break
                except Exception:
                    self._exit_to_menu()
                    break

                with self._lock:
                    self._render(frame)
                self._frame_count += 1

                cv2.imshow(win_name, frame)
                key = cv2.waitKey(1000 // FPS) & 0xFF
                if key == 27:
                    self._exit_to_menu()
                    break
                self._handle_key(key)

            self._music.stop()
            try:
                cv2.destroyAllWindows()
                cv2.waitKey(1)
            except Exception:
                pass
            time.sleep(0.2)


def main(args=None):
    rclpy.init(args=args)
    node = PalabrasNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.run_display_main_thread()
    except KeyboardInterrupt:
        print("Interrupcion recibida, cerrando...")
    finally:
        node._active = False
        node._music.stop()
        cv2.destroyAllWindows()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()