#!/usr/bin/env python3
"""
paint_node.py - YAREN PAINT (LifecycleNode + OpenCV 800x480 a pantalla completa)
Diseno original (tema claro) con la estructura de memoria_node.py.
Todo se dibuja en RGB y se convierte a BGR solo al mostrar/guardar.
"""
import json
import math
import os
import random
import subprocess
import threading
import time
import unicodedata
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String

try:
    from PIL import Image, ImageDraw, ImageFont
    HAVE_PIL = True
except ImportError:
    HAVE_PIL = False

from yaren_paint.drawings import (DRAWINGS, hit_test, outline_points,
                                  is_fillable, to_screen, to_design)

# ─── Geometria (frame 800x480, escala de la UI original de 1024x600) ─────────
W, H = 800, 480
S = W / 1024


def sc(v):
    return int(v * S)


TOP_H, BOT_H = sc(64), sc(84)
CANVAS = (20, TOP_H + 12, W - 40, H - TOP_H - BOT_H - 26)   # x, y, w, h
CX, CY, CW, CH = CANVAS
TOL, CELL = int(26 * S), int(26 * S) + 4

# ─── Paleta original (RGB) ────────────────────────────────────────────────────
BG, BAR, WHITE, INK = (245, 247, 252), (225, 232, 245), (255, 255, 255), (40, 50, 80)
PALETTE = [(220, 40, 40), (255, 140, 0), (250, 220, 30), (40, 170, 70), (40, 120, 230),
           (130, 60, 200), (240, 120, 180), (110, 70, 40), (0, 0, 0)]
SIZES = [3, 8, 17]
FD = cv2.FONT_HERSHEY_DUPLEX

EN = {
    "las paredes": "the walls", "el techo": "the roof", "la puerta": "the door",
    "una ventana": "a window", "la otra ventana": "the other window", "el sol": "the sun",
    "el pasto": "the grass", "la cara del sol": "the face of the sun",
    "un rayo del sol": "a sun ray", "un ojo": "an eye", "el otro ojo": "the other eye",
    "la sonrisa": "the smile", "el tallo": "the stem", "una hoja": "a leaf",
    "un pétalo": "a petal", "el centro de la flor": "the center of the flower",
    "la cola": "the tail", "la aleta": "the fin", "el cuerpo": "the body", "el ojo": "the eye",
    "la boca": "the mouth", "una burbuja": "a bubble", "otra burbuja": "another bubble",
    "una burbuja pequeña": "a small bubble", "el tronco": "the trunk",
    "la copa de la izquierda": "the left crown", "la copa de la derecha": "the right crown",
    "la copa de arriba": "the top crown", "una manzana": "an apple",
    "otra manzana": "another apple", "una manzana más": "one more apple",
    "la oreja izquierda": "the left ear", "la oreja derecha": "the right ear",
    "la cabeza": "the head", "la nariz": "the nose", "un bigote": "a whisker",
    "otro bigote": "another whisker",
    "una casa": "a house", "un sol feliz": "a happy sun", "una flor": "a flower",
    "un pez": "a fish", "un árbol": "a tree", "un gato": "a cat",
    "Casa": "House", "Sol": "Sun", "Flor": "Flower", "Pez": "Fish",
    "Árbol": "Tree", "Gato": "Cat",
}

# =============================================================================
#  TEXTO (Pillow con DejaVu Bold -> conserva acentos; fallback cv2 ASCII)
# =============================================================================
FONT_PATHS = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    "/usr/share/fonts/truetype/freefont/FreeSansBold.ttf",
]
_fonts, _masks = {}, {}


def _font(size):
    if size not in _fonts:
        f = None
        for p in FONT_PATHS:
            if os.path.isfile(p):
                try:
                    f = ImageFont.truetype(p, size)
                    break
                except Exception:
                    pass
        _fonts[size] = f
    return _fonts[size]


def _ascii(s):
    return unicodedata.normalize("NFKD", s).encode("ascii", "ignore").decode()


def _mask(text, size):
    key = (text, size)
    if key not in _masks:
        m = None
        f = _font(size)
        if f is not None and text.strip():
            l, t, r, b = f.getbbox(text)
            asc, desc = f.getmetrics()
            if r > l:
                img = Image.new("L", (r + 2, asc + desc + 2), 0)
                ImageDraw.Draw(img).text((0, 0), text, font=f, fill=255)
                m = np.asarray(img, dtype=np.float32)[:, max(l, 0):r] / 255.0
        _masks[key] = m
    return _masks[key]


def _use_pil(size):
    return HAVE_PIL and _font(size) is not None


def tsize(text, size):
    if _use_pil(size):
        m = _mask(text, size)
        return (0, size) if m is None else (m.shape[1], m.shape[0])
    (w, h), _ = cv2.getTextSize(_ascii(text), FD, size / 32.0, 2 if size >= 26 else 1)
    return w, h


def put_tl(img, text, x, y, size, color):
    if _use_pil(size):
        m = _mask(text, size)
        if m is None:
            return
        h, w = m.shape
        x0, y0, x1, y1 = max(0, x), max(0, y), min(W, x + w), min(H, y + h)
        if x1 <= x0 or y1 <= y0:
            return
        mm = m[y0 - y:y1 - y, x0 - x:x1 - x][..., None]
        roi = img[y0:y1, x0:x1].astype(np.float32)
        img[y0:y1, x0:x1] = (roi * (1 - mm) + np.array(color, np.float32) * mm).astype(np.uint8)
    else:
        tw, th = tsize(text, size)
        cv2.putText(img, _ascii(text), (x, y + th), FD, size / 32.0, color,
                    2 if size >= 26 else 1, cv2.LINE_AA)


def put_c(img, text, cx, cy, size, color):
    w, h = tsize(text, size)
    put_tl(img, text, cx - w // 2, cy - h // 2, size, color)


def put_ml(img, text, x, cy, size, color):
    w, h = tsize(text, size)
    put_tl(img, text, x, cy - h // 2, size, color)


def fit_size(text, max_w, start, lo=9):
    s = start
    while s > lo and tsize(text, s)[0] > max_w:
        s -= 1
    return s


# =============================================================================
#  HELPERS DE DIBUJO
# =============================================================================
def _fill(img, p1, p2, color, r):
    x1, y1 = p1
    x2, y2 = p2
    r = max(1, min(r, (x2 - x1) // 2, (y2 - y1) // 2))
    cv2.rectangle(img, (x1 + r, y1), (x2 - r, y2), color, -1)
    cv2.rectangle(img, (x1, y1 + r), (x2, y2 - r), color, -1)
    for cx, cy in ((x1 + r, y1 + r), (x2 - r, y1 + r), (x1 + r, y2 - r), (x2 - r, y2 - r)):
        cv2.circle(img, (cx, cy), r, color, -1, cv2.LINE_AA)


def rrect(img, p1, p2, color, r=10, border=None, t=3):
    if border is not None:
        _fill(img, p1, p2, border, r)
        p1, p2, r = (p1[0] + t, p1[1] + t), (p2[0] - t, p2[1] - t), max(1, r - t)
    _fill(img, p1, p2, color, r)


def button(img, rect, label, bg, fg=WHITE, border=None, size=sc(22)):
    x1, y1, x2, y2 = rect
    rrect(img, (x1, y1), (x2, y2), bg, 12, border=border, t=3)
    put_c(img, label, (x1 + x2) // 2, (y1 + y2) // 2,
          fit_size(label, (x2 - x1) - 10, size), fg)


def viewport(rect):
    x, y, w, h = rect
    s = min(w / 800, h / 600)
    return s, x + (w - 800 * s) / 2, y + (h - 600 * s) / 2


VP = viewport(CANVAS)                                  # coordenadas de pantalla
VP_LOCAL = (VP[0], VP[1] - CX, VP[2] - CY)             # coordenadas del canvas


def _pt(vp, p):
    return int(vp[1] + p[0] * vp[0]), int(vp[2] + p[1] * vp[0])


def draw_shape(img, shape, vp, color, thick):
    """thick=-1 rellena (solo formas cerradas)."""
    kind, p, _ = shape
    s = vp[0]
    if kind == "circle":
        cv2.circle(img, _pt(vp, p[:2]), max(1, int(p[2] * s)), color, thick, cv2.LINE_AA)
    elif kind == "ellipse":
        cv2.ellipse(img, _pt(vp, p[:2]), (max(1, int(p[2] * s)), max(1, int(p[3] * s))),
                    0, 0, 360, color, thick, cv2.LINE_AA)
    elif kind == "poly":
        pts = np.array([_pt(vp, q) for q in p], np.int32)
        if thick < 0:
            cv2.fillPoly(img, [pts], color, cv2.LINE_AA)
        else:
            cv2.polylines(img, [pts], True, color, thick, cv2.LINE_AA)
    elif kind == "line" and thick > 0:
        cv2.polylines(img, [np.array([_pt(vp, q) for q in p], np.int32)],
                      False, color, thick, cv2.LINE_AA)


ICON_PATH = "/home/roberto/robotis_ws/src/YAREN2/yaren_paint/icono/pintor.png"
_icon_cache = {}


def load_icon(size):
    if size not in _icon_cache:
        img = cv2.imread(ICON_PATH, cv2.IMREAD_UNCHANGED)
        if img is None:
            _icon_cache[size] = None
        else:
            if img.ndim == 2:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGRA)
            elif img.shape[2] == 3:
                img = cv2.cvtColor(img, cv2.COLOR_BGR2BGRA)
            img = cv2.resize(img, (size, size), interpolation=cv2.INTER_AREA)
            rgb = cv2.cvtColor(np.ascontiguousarray(img[:, :, :3]), cv2.COLOR_BGR2RGB)
            alpha = img[:, :, 3:4].astype(np.float32) / 255.0
            _icon_cache[size] = (rgb, alpha)
    return _icon_cache[size]


def put_icon(img, x, y, size):
    ic = load_icon(size)
    if ic is None:
        return False
    rgb, alpha = ic
    x0, y0, x1, y1 = max(0, x), max(0, y), min(W, x + size), min(H, y + size)
    if x1 <= x0 or y1 <= y0:
        return True
    a = alpha[y0 - y:y1 - y, x0 - x:x1 - x]
    c = rgb[y0 - y:y1 - y, x0 - x:x1 - x].astype(np.float32)
    roi = img[y0:y1, x0:x1].astype(np.float32)
    img[y0:y1, x0:x1] = (roi * (1 - a) + c * a).astype(np.uint8)
    return True


def R(x, y, w, h):
    return (x, y, x + w, y + h)


def _in(r, x, y):
    return r[0] <= x <= r[2] and r[1] <= y <= r[3]


# ─── Rectangulos fijos ────────────────────────────────────────────────────────
MENU_R = R(10, sc(8), sc(130), TOP_H - sc(16))
OTHER_R = R(sc(150), sc(8), sc(170), TOP_H - sc(16))
QUIT_R = R(W - sc(150), H - sc(60), sc(140), sc(48))
_bw, _bh = sc(640), sc(96)
MODE_RECTS = [(k, R((W - _bw) // 2, sc(185) + i * (_bh + sc(20)), _bw, _bh))
              for i, k in enumerate(("free", "guided", "color"))]


def _lib_cells(n):
    cols, gap = 4, sc(12)
    rows = -(-n // cols)
    cw = (CW - gap * (cols - 1)) // cols
    ch = (CH - gap * (rows - 1)) // rows
    return [R(CX + (i % cols) * (cw + gap), CY + (i // cols) * (ch + gap), cw, ch)
            for i in range(n)]


LIB_CELLS = _lib_cells(len(DRAWINGS) + 1)


# =============================================================================
#  NODO PRINCIPAL
# =============================================================================
class PaintNode(LifecycleNode):

    def __init__(self):
        super().__init__('paint_node')
        self.declare_parameter('autostart', False)
        self._active = False
        self.is_english = False
        self._cmds = []
        self._cmd_lock = threading.Lock()
        self._mode_pub = self._state_pub = self._lang_sub = self._cmd_sub = None
        self._color = PALETTE[4]
        self._size_idx = 0
        self._reset_all()

    # ─────────────────────────────────────────────────────────────────────────
    #  LIFECYCLE
    # ─────────────────────────────────────────────────────────────────────────
    def on_configure(self, state):
        self.get_logger().info('PaintNode: configurando...')
        try:
            self._mode_pub = self.create_publisher(String, '/yaren_mode', 1)
            self._state_pub = self.create_publisher(String, '/yaren/paint/state', 10)
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self._lang_sub = self.create_subscription(
                Bool, '/yaren/is_english', self._lang_cb, qos)
            self._cmd_sub = self.create_subscription(
                String, '/yaren/paint/command', self._cmd_cb, 10)
            if not HAVE_PIL:
                self.get_logger().warn('Pillow no instalado: textos sin tildes (pip install pillow)')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'on_configure error: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        self.get_logger().info('PaintNode: ACTIVO')
        self._reset_all()
        self._active = True
        return super().on_activate(state)

    def on_deactivate(self, state):
        self.get_logger().info('PaintNode: INACTIVO')
        self._active = False
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        for pub in (self._mode_pub, self._state_pub):
            if pub:
                self.destroy_publisher(pub)
        for sub in (self._lang_sub, self._cmd_sub):
            if sub:
                self.destroy_subscription(sub)
        cv2.destroyAllWindows()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self._active = False
        cv2.destroyAllWindows()
        return TransitionCallbackReturn.SUCCESS

    # ─────────────────────────────────────────────────────────────────────────
    #  ROS
    # ─────────────────────────────────────────────────────────────────────────
    def _lang_cb(self, msg):
        self.is_english = msg.data

    def _cmd_cb(self, msg):
        with self._cmd_lock:
            self._cmds.append(msg.data)

    def _pop_cmds(self):
        with self._cmd_lock:
            out, self._cmds = self._cmds, []
        return out

    def _publish_idle(self):
        if self._mode_pub:
            try:
                self._mode_pub.publish(String(data='idle'))
            except Exception:
                pass

    def _publish_state(self):
        if not self._state_pub:
            return
        try:
            d = {
                "app": "yaren_paint", "screen": self._screen,
                "drawing": DRAWINGS[self._drawing]["id"] if self._drawing is not None else None,
                "step": self._step,
                "steps": len(self._shapes()) if self._screen == "guided" else 0,
                "progress": round(self._progress, 2),
                "finished": self._finished, "message": self._msg,
            }
            self._state_pub.publish(String(data=json.dumps(d, ensure_ascii=False)))
        except Exception:
            pass

    # ─────────────────────────────────────────────────────────────────────────
    #  ESTADO
    # ─────────────────────────────────────────────────────────────────────────
    def tr(self, es, en):
        return en if self.is_english else es

    def _tn(self, text):
        return EN.get(text, text) if self.is_english else text

    def _reset_all(self):
        self._screen = "menu"        # menu | library | free | guided | color
        self._pending = None
        self._drawing = None
        self._msg = ""
        self._eraser = False
        self._stroking = False
        self._last = (0, 0)
        self._canvas = np.full((CH, CW, 3), 255, np.uint8)
        self._grid = {}
        self._step = 0
        self._step_pts = []
        self._cov = []
        self._progress = 0.0
        self._finished = False
        self._fills = []

    def _shapes(self):
        return DRAWINGS[self._drawing]["shapes"]

    def _go_menu(self):
        self._screen = "menu"
        self._msg = self.tr("¿Qué quieres hacer ahora?", "What do you want to do now?")

    def _exit_to_idle(self):
        self._publish_idle()
        self._active = False

    def _open_library(self, mode):
        self._pending = mode
        self._screen = "library"
        if mode == "guided":
            self._msg = self.tr("Elige el dibujo que quieras y lo dibujamos juntos",
                                "Pick a drawing and we'll draw it together")
        else:
            self._msg = self.tr("Elige el dibujo que quieras y lo coloreamos",
                                "Pick a drawing and we'll color it in")

    def _start_mode(self, mode, idx=None):
        self._screen, self._drawing = mode, idx
        self._eraser = False
        self._stroking = False
        self._canvas = np.full((CH, CW, 3), 255, np.uint8)
        self._grid = {}
        self._step = 0
        self._progress = 0.0
        self._finished = False
        self._fills = []
        if mode == "free":
            self._msg = self.tr("Dibuja lo que quieras. ¡Soy toda ojos!",
                                "Draw anything you like. I'm all eyes!")
        elif mode == "guided":
            title = DRAWINGS[idx]["title"]
            self._begin_step(0, intro=self.tr(f"¡Dibujemos {title}! ",
                                              f"Let's draw {self._tn(title)}! "))
        elif mode == "color":
            self._fills = [None] * len(self._shapes())
            title = DRAWINGS[idx]["title"]
            self._msg = self.tr(f"Colorea {title}. Toca una parte para pintarla",
                                f"Color in {self._tn(title)}. Tap a part to paint it")

    # ── modo guiado ──────────────────────────────────────────────────────────
    def _begin_step(self, i, intro="", praise=False):
        shapes = self._shapes()
        self._step = i
        self._grid = {}
        self._step_pts = [to_screen(VP, p) for p in outline_points(shapes[i])]
        self._cov = [False] * len(self._step_pts)
        self._progress = 0.0
        name, n = shapes[i][2], len(shapes)
        ok = self.tr("¡Muy bien! ", "Well done! ") if praise else ""
        self._msg = intro + ok + self.tr(f"Paso {i + 1} de {n}: dibuja {name}",
                                         f"Step {i + 1} of {n}: draw {self._tn(name)}")

    def _advance(self, skipped=False):
        nxt = self._step + 1
        if nxt >= len(self._shapes()):
            self._step = len(self._shapes())
            self._finished = True
            self._step_pts, self._cov = [], []
            self._msg = self.tr("¡Lo lograste! Quedó muy bonito. ¡Ahora pruébalo en modo colorear!",
                                "You did it! Looks great. Now try it in color mode!")
        else:
            self._begin_step(nxt, praise=not skipped)

    def _record(self, a, b):
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 5))
        for i in range(n + 1):
            t = i / n
            p = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
            self._grid.setdefault((int(p[0] // CELL), int(p[1] // CELL)), []).append(p)

    def _is_covered(self, p):
        cx, cy = int(p[0] // CELL), int(p[1] // CELL)
        t2 = TOL * TOL
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for q in self._grid.get((cx + dx, cy + dy), ()):
                    if (q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2 <= t2:
                        return True
        return False

    def _update_guided(self):
        if self._screen != "guided" or self._finished or not self._step_pts:
            return
        for k, p in enumerate(self._step_pts):
            if not self._cov[k] and self._is_covered(p):
                self._cov[k] = True
        self._progress = sum(self._cov) / len(self._cov)
        if self._progress >= 0.8:
            self._advance()

    # ── trazo ────────────────────────────────────────────────────────────────
    def _brush(self):
        w = SIZES[self._size_idx]
        return (WHITE, w * 2) if self._eraser else (self._color, w)

    @staticmethod
    def _local(p):
        return p[0] - CX, p[1] - CY

    def _stamp(self, pos):
        col, w = self._brush()
        cv2.circle(self._canvas, self._local(pos), max(1, w // 2), col, -1, cv2.LINE_AA)
        if self._screen == "guided":
            self._record(pos, pos)
            self._update_guided()

    def _stroke_to(self, a, b):
        col, w = self._brush()
        cv2.line(self._canvas, self._local(a), self._local(b), col, w, cv2.LINE_AA)
        cv2.circle(self._canvas, self._local(b), max(1, w // 2), col, -1, cv2.LINE_AA)
        if self._screen == "guided":
            self._record(a, b)

    # ── coloreo ──────────────────────────────────────────────────────────────
    def _click_color(self, pos):
        q = to_design(VP, pos)
        shapes = self._shapes()
        for i in range(len(shapes) - 1, -1, -1):
            if is_fillable(shapes[i]) and hit_test(shapes[i], q):
                self._fills[i] = None if self._eraser else self._color
                break
        done = all(f is not None for sh, f in zip(shapes, self._fills) if is_fillable(sh))
        if done and not self._finished:
            self._msg = self.tr("¡Terminaste de colorear! ¡Qué obra de arte! Guárdalo si quieres",
                                "All colored in! What a masterpiece! Save it if you like")
        elif not done:
            self._msg = self.tr("¡Qué lindo color!", "Nice color!")
        self._finished = done

    def _compose_color(self):
        img = np.full((CH, CW, 3), 255, np.uint8)
        shapes = self._shapes()
        for sh, f in zip(shapes, self._fills):
            if is_fillable(sh):
                draw_shape(img, sh, VP_LOCAL, f if f is not None else WHITE, -1)
        for sh in shapes:
            draw_shape(img, sh, VP_LOCAL, (20, 20, 20), max(2, sc(5)))
        return img

    def _save(self):
        if self._screen not in ("free", "guided", "color"):
            return
        folder = os.path.expanduser("~/yaren_paint")
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, datetime.now().strftime("yaren_paint_%Y%m%d_%H%M%S.png"))
        img = self._compose_color() if self._screen == "color" else self._canvas
        cv2.imwrite(path, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        self._msg = self.tr("¡Guardé tu dibujo!", "Saved your drawing!")
        self.get_logger().info(f'Guardado en {path}')

    # ─────────────────────────────────────────────────────────────────────────
    #  ENTRADA (mouse / tactil)
    # ─────────────────────────────────────────────────────────────────────────
    def _toolbar(self):
        y1, y2 = H - BOT_H + sc(10), H - sc(10)
        x, items = 10, []
        for i in range(len(PALETTE)):
            items.append(("color", i, (x, y1, x + sc(46), y2), None))
            x += sc(48)
        x += sc(8)

        def add(kind, val, w, label=None):
            nonlocal x
            items.append((kind, val, (x, y1, x + sc(w), y2), label))
            x += sc(w) + sc(10)

        en = self.is_english
        if self._screen in ("free", "guided"):
            for i in range(len(SIZES)):
                add("size", i, 50)
        if self._screen in ("free", "color"):
            add("eraser", 0, 90, "Eraser" if en else "Borrador")
        if self._screen == "free":
            add("clear", 0, 140, "Clear all" if en else "Borrar todo")
        if self._screen == "guided":
            add("skip", 0, 130, "Skip step" if en else "Saltar paso")
        if self._screen in ("guided", "color"):
            add("restart", 0, 120, "Restart" if en else "Reiniciar")
        add("save", 0, 110, "Save" if en else "Guardar")
        return items

    def _click_tool(self, kind, val):
        if kind == "color":
            self._color, self._eraser = PALETTE[val], False
        elif kind == "size":
            self._size_idx = val
        elif kind == "eraser":
            self._eraser = not self._eraser
        elif kind == "clear":
            self._start_mode("free")
            self._msg = self.tr("¡Pizarra limpia! Dibuja lo que quieras",
                                "Clean board! Draw anything you like")
        elif kind == "restart":
            self._start_mode(self._screen, self._drawing)
        elif kind == "skip" and not self._finished:
            self._advance(skipped=True)
        elif kind == "save":
            self._save()

    def _on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self._on_down((x, y))
        elif event == cv2.EVENT_MOUSEMOVE:
            if self._stroking and (flags & cv2.EVENT_FLAG_LBUTTON):
                self._stroke_to(self._last, (x, y))
                self._last = (x, y)
                self._update_guided()
        elif event == cv2.EVENT_LBUTTONUP:
            self._stroking = False

    def _on_down(self, pos):
        x, y = pos
        scr = self._screen
        if scr != "menu" and _in(MENU_R, x, y):
            self._go_menu()
            return
        if scr in ("guided", "color") and _in(OTHER_R, x, y):
            self._open_library(scr)
            return
        if scr == "menu":
            for key, r in MODE_RECTS:
                if _in(r, x, y):
                    if key == "free":
                        self._start_mode("free")
                    else:
                        self._open_library(key)
                    return
            if _in(QUIT_R, x, y):
                self._exit_to_idle()
            return
        if scr == "library":
            for i, r in enumerate(LIB_CELLS):
                if _in(r, x, y):
                    idx = i if i < len(DRAWINGS) else random.randrange(len(DRAWINGS))
                    self._start_mode(self._pending, idx)
                    return
            return
        for kind, val, r, _ in self._toolbar():
            if _in(r, x, y):
                self._click_tool(kind, val)
                return
        if _in((CX, CY, CX + CW, CY + CH), x, y):
            if scr in ("free", "guided"):
                self._stroking, self._last = True, pos
                self._stamp(pos)
            elif scr == "color":
                self._click_color(pos)

    def _run_command(self, cmd):
        cmd = cmd.strip().lower()
        if cmd == "menu":
            self._go_menu()
        elif cmd in ("mode:free", "free", "libre"):
            self._start_mode("free")
        elif cmd in ("mode:guided", "guided", "guiado"):
            self._open_library("guided")
        elif cmd in ("mode:color", "color", "colorear"):
            self._open_library("color")
        elif cmd.startswith("pick:"):
            mode = self._screen if self._screen in ("guided", "color") else self._pending
            name = cmd[5:].strip()
            ids = [d["id"] for d in DRAWINGS]
            idx = ids.index(name) if name in ids else (int(name) if name.isdigit() else None)
            if mode and idx is not None and idx < len(DRAWINGS):
                self._start_mode(mode, idx)
        elif cmd == "clear" and self._screen == "free":
            self._click_tool("clear", 0)
        elif cmd == "next" and self._screen == "guided":
            self._click_tool("skip", 0)
        elif cmd == "save":
            self._save()
        elif cmd == "quit":
            self._exit_to_idle()

    # ─────────────────────────────────────────────────────────────────────────
    #  RENDER (todo en RGB)
    # ─────────────────────────────────────────────────────────────────────────
    def _render(self):
        f = np.empty((H, W, 3), np.uint8)
        f[:] = BG
        scr = self._screen
        if scr == "menu":
            self._draw_menu(f)
        elif scr == "library":
            self._draw_library(f)
        else:
            rrect(f, (CX - 3, CY - 3), (CX + CW + 3, CY + CH + 3), (120, 130, 150), 8)
            f[CY:CY + CH, CX:CX + CW] = self._compose_color() if scr == "color" else self._canvas
            if scr == "guided":
                self._draw_guide(f)
            self._draw_toolbar(f)
        if scr != "menu":
            self._draw_topbar(f)
        return f

    def _draw_topbar(self, f):
        cv2.rectangle(f, (0, 0), (W, TOP_H), BAR, -1)
        if self._screen != "menu":
            button(f, MENU_R, self.tr("< Menú", "< Menu"), (90, 100, 130))
        if self._screen in ("guided", "color"):
            button(f, OTHER_R, self.tr("Otro dibujo", "Another"), (90, 100, 130))
        fx, fy, fr = sc(345), TOP_H // 2, sc(22)
        cv2.circle(f, (fx, fy), fr, (60, 190, 200), -1, cv2.LINE_AA)
        for dx in (-8, 8):
            cv2.circle(f, (fx + int(dx * S), fy - int(4 * S)), max(2, int(5 * S)),
                       WHITE, -1, cv2.LINE_AA)
        cv2.ellipse(f, (fx, fy + int(1 * S)), (int(9 * S), int(7 * S)), 0, 10, 170,
                    WHITE, 2, cv2.LINE_AA)
        bx1 = sc(385)
        rrect(f, (bx1, sc(8)), (W - 10, TOP_H - sc(8)), WHITE, 12)
        msg = self._msg or self.tr("¡Hola! Soy Yaren. ¿Qué quieres hacer hoy?",
                                   "Hi! I'm Yaren. What do you want to do today?")
        size = next((sc(s) for s in (30, 26, 22, 18, 15)
                     if tsize(msg, sc(s))[0] <= (W - 10 - bx1) - 24), sc(15))
        put_ml(f, msg, bx1 + 12, TOP_H // 2, size, INK)

    def _draw_menu(self, f):
        title, ts = "YAREN PAINT", sc(60)
        tw, _ = tsize(title, ts)
        isz, gap, cy = sc(100), sc(16), sc(72)
        x0 = (W - (isz + gap + tw)) // 2
        if put_icon(f, x0, cy - isz // 2, isz):
            put_ml(f, title, x0 + isz + gap, cy, ts, (40, 80, 170))
        else:
            put_c(f, title, W // 2, cy, ts, (40, 80, 170))
        specs = {
            "free": (self.tr("1 · Dibujo libre", "1 · Free draw"), (40, 170, 110)),
            "guided": (self.tr("2 · Dibujar con Yaren", "2 · Draw with Yaren"), (40, 120, 230)),
            "color": (self.tr("3 · Colorear", "3 · Color in"), (240, 130, 30)),
        }
        for key, r in MODE_RECTS:
            label, col = specs[key]
            button(f, r, label, col, size=sc(32))
        button(f, QUIT_R, self.tr("Salir", "Exit"), (150, 60, 60))

    def _draw_library(self, f):
        for i, (x1, y1, x2, y2) in enumerate(LIB_CELLS):
            rrect(f, (x1, y1), (x2, y2), WHITE, 14, border=(170, 180, 200), t=2)
            cx, label_h = (x1 + x2) // 2, sc(34)
            if i < len(DRAWINGS):
                vp = viewport((x1 + 8, y1 + 8, x2 - x1 - 16, y2 - y1 - 16 - label_h))
                for sh in DRAWINGS[i]["shapes"]:
                    draw_shape(f, sh, vp, (30, 30, 30), 1)
                put_c(f, self._tn(DRAWINGS[i]["label"]), cx, y2 - label_h // 2 - 2,
                      sc(22), INK)
            else:
                put_c(f, "?", cx, (y1 + y2) // 2 - label_h // 2, sc(60), (240, 130, 30))
                put_c(f, self.tr("¡Sorpresa!", "Surprise!"), cx, y2 - label_h // 2 - 2,
                      sc(22), INK)

    def _draw_guide(self, f):
        shapes = self._shapes()
        for i in range(min(self._step, len(shapes))):
            draw_shape(f, shapes[i], VP, (190, 190, 190), 2)
        if not self._finished and self._step < len(shapes):
            draw_shape(f, shapes[self._step], VP, (170, 200, 250), 1)
            for p, c in zip(self._step_pts, self._cov):
                cv2.circle(f, (int(p[0]), int(p[1])), max(2, sc(5)),
                           (60, 200, 90) if c else (60, 120, 230), -1, cv2.LINE_AA)
            if self._step_pts:
                p0 = self._step_pts[0]
                cv2.circle(f, (int(p0[0]), int(p0[1])), sc(9), (255, 140, 0), 2, cv2.LINE_AA)

    def _draw_toolbar(self, f):
        cv2.rectangle(f, (0, H - BOT_H), (W, H), BAR, -1)
        dark = (20, 20, 20)
        for kind, val, r, label in self._toolbar():
            x1, y1, x2, y2 = r
            if kind == "color":
                sel = (not self._eraser) and self._color == PALETTE[val]
                rrect(f, (x1, y1), (x2, y2), PALETTE[val], 10,
                      border=dark if sel else None, t=3)
            elif kind == "size":
                rrect(f, (x1, y1), (x2, y2), WHITE, 10,
                      border=dark if self._size_idx == val else None, t=3)
                cv2.circle(f, ((x1 + x2) // 2, (y1 + y2) // 2),
                           max(2, SIZES[val] // 2 + 1), (40, 40, 40), -1, cv2.LINE_AA)
            elif kind == "eraser":
                button(f, r, label, (120, 120, 140), border=dark if self._eraser else None)
            else:
                col = {"clear": (200, 70, 70), "restart": (200, 120, 40),
                       "skip": (120, 120, 140), "save": (40, 150, 90)}[kind]
                button(f, r, label, col)

    # ─────────────────────────────────────────────────────────────────────────
    #  VENTANA (hilo principal)
    # ─────────────────────────────────────────────────────────────────────────
    @staticmethod
    def _force_focus(win_name):
        time.sleep(0.5)
        try:
            subprocess.run(["xdotool", "search", "--name", win_name,
                            "windowactivate", "--sync", "windowfocus"],
                           check=False, capture_output=True)
        except FileNotFoundError:
            pass

    def run_display_main_thread(self):
        win = 'YAREN2 - Paint'
        while rclpy.ok():
            while not self._active and rclpy.ok():
                time.sleep(0.05)
            if not rclpy.ok():
                break

            cv2.namedWindow(win, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(win, W, H)
            cv2.setWindowProperty(win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            cv2.setWindowProperty(win, cv2.WND_PROP_TOPMOST, 1)
            cv2.setMouseCallback(win, self._on_mouse)
            threading.Thread(target=self._force_focus, args=(win,), daemon=True).start()

            last_pub = 0.0
            while self._active and rclpy.ok():
                for c in self._pop_cmds():
                    self._run_command(c)
                cv2.imshow(win, cv2.cvtColor(self._render(), cv2.COLOR_RGB2BGR))
                key = cv2.waitKey(16) & 0xFF
                if key == 27:   # ESC
                    if self._screen == "menu":
                        self._exit_to_idle()
                    else:
                        self._go_menu()
                try:
                    if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                        self.get_logger().info("Ventana cerrada. Publicando idle...")
                        self._exit_to_idle()
                        break
                except Exception:
                    self._exit_to_idle()
                    break
                now = time.time()
                if now - last_pub > 0.5:
                    self._publish_state()
                    last_pub = now

            try:
                cv2.destroyAllWindows()
                cv2.waitKey(1)
            except Exception:
                pass
            print("PaintNode desactivado, esperando reactivacion...")
            time.sleep(0.2)


# ─── LEVELS_BLOCK: biblioteca por niveles (bajo / medio / alto) ──────────────
from yaren_paint.drawings_levels import EXTRA
for _d in DRAWINGS:
    _d.setdefault("level", "bajo")
_have = {_d["id"] for _d in DRAWINGS}
DRAWINGS.extend(e for e in EXTRA if e["id"] not in _have)

LEVELS = ("bajo", "medio", "alto")
LEVEL_NAMES = {"bajo": ("Fácil", "Easy"), "medio": ("Medio", "Medium"), "alto": ("Difícil", "Hard")}
LEVEL_COLORS = {"bajo": (40, 170, 70), "medio": (240, 130, 30), "alto": (200, 60, 60)}
TAB_H = sc(40)
TAB_RECTS = {lv: R(CX + i * (sc(150) + sc(10)), CY, sc(150), TAB_H)
             for i, lv in enumerate(LEVELS)}


def _level_idx(lv):
    return [i for i, d in enumerate(DRAWINGS) if d.get("level", "bajo") == lv]


def _level_cells(lv):
    n = len(_level_idx(lv)) + (1 if lv == "bajo" else 0)
    cols, gap = 4, sc(12)
    rows = max(1, -(-n // cols))
    top = CY + TAB_H + gap
    avail = CY + CH - top
    cw = (CW - gap * (cols - 1)) // cols
    ch = min((avail - gap * (rows - 1)) // rows, sc(190))
    return [R(CX + (i % cols) * (cw + gap), top + (i // cols) * (ch + gap), cw, ch)
            for i in range(n)]


def _draw_library_lv(self, f):
    lv = getattr(self, "_lib_level", "bajo")
    for k, r in TAB_RECTS.items():
        es, en = LEVEL_NAMES[k]
        on = k == lv
        button(f, r, self.tr(es, en), LEVEL_COLORS[k] if on else (150, 160, 180),
               border=(20, 20, 20) if on else None, size=sc(20))
    idxs = _level_idx(lv)
    for j, (x1, y1, x2, y2) in enumerate(_level_cells(lv)):
        rrect(f, (x1, y1), (x2, y2), WHITE, 14, border=(170, 180, 200), t=2)
        cx, label_h = (x1 + x2) // 2, sc(34)
        if j < len(idxs):
            d = DRAWINGS[idxs[j]]
            vp = viewport((x1 + 8, y1 + 8, x2 - x1 - 16, y2 - y1 - 16 - label_h))
            for sh in d["shapes"]:
                draw_shape(f, sh, vp, (30, 30, 30), 1)
            put_c(f, self._tn(d["label"]), cx, y2 - label_h // 2 - 2, sc(22), INK)
        else:
            put_c(f, "?", cx, (y1 + y2) // 2 - label_h // 2, sc(60), (240, 130, 30))
            put_c(f, self.tr("¡Sorpresa!", "Surprise!"), cx, y2 - label_h // 2 - 2, sc(22), INK)


_orig_on_down = PaintNode._on_down


def _on_down_lv(self, pos):
    if self._screen == "library":
        x, y = pos
        if _in(MENU_R, x, y):
            return _orig_on_down(self, pos)
        lv = getattr(self, "_lib_level", "bajo")
        for k, r in TAB_RECTS.items():
            if _in(r, x, y):
                self._lib_level = k
                return
        idxs = _level_idx(lv)
        for j, r in enumerate(_level_cells(lv)):
            if _in(r, x, y):
                idx = idxs[j] if j < len(idxs) else random.choice(idxs)
                self._start_mode(self._pending, idx)
                return
        return
    return _orig_on_down(self, pos)


PaintNode._draw_library = _draw_library_lv
PaintNode._on_down = _on_down_lv


# ─── GUIDE_TOGGLE: boton Guia ON/OFF en el modo guiado ───────────────────────
def _toolbar_gt(self):
    compact = self._screen == "guided"
    y1, y2 = H - BOT_H + sc(10), H - sc(10)
    sw, st, g = (40, 42, 8) if compact else (46, 48, 10)
    x, items = 10, []
    for i in range(len(PALETTE)):
        items.append(("color", i, (x, y1, x + sc(sw), y2), None))
        x += sc(st)
    x += sc(8)

    def add(kind, val, w, label=None):
        nonlocal x
        items.append((kind, val, (x, y1, x + sc(w), y2), label))
        x += sc(w) + sc(g)

    en = self.is_english
    on = getattr(self, "_show_guide", True)
    if self._screen in ("free", "guided"):
        for i in range(len(SIZES)):
            add("size", i, 44 if compact else 50)
    if self._screen in ("free", "color"):
        add("eraser", 0, 90, "Eraser" if en else "Borrador")
    if self._screen == "free":
        add("clear", 0, 140, "Clear all" if en else "Borrar todo")
    if self._screen == "guided":
        add("skip", 0, 96, "Skip step" if en else "Saltar paso")
        add("guide", 0, 110, ("With guide" if on else "No guide") if en else ("Con guía" if on else "Sin guía"))
    if self._screen in ("guided", "color"):
        add("restart", 0, 96 if compact else 120, "Restart" if en else "Reiniciar")
    add("save", 0, 90 if compact else 110, "Save" if en else "Guardar")
    return items


def _draw_toolbar_gt(self, f):
    cv2.rectangle(f, (0, H - BOT_H), (W, H), BAR, -1)
    dark = (20, 20, 20)
    for kind, val, r, label in self._toolbar():
        x1, y1, x2, y2 = r
        if kind == "color":
            sel = (not self._eraser) and self._color == PALETTE[val]
            rrect(f, (x1, y1), (x2, y2), PALETTE[val], 10,
                  border=dark if sel else None, t=3)
        elif kind == "size":
            rrect(f, (x1, y1), (x2, y2), WHITE, 10,
                  border=dark if self._size_idx == val else None, t=3)
            cv2.circle(f, ((x1 + x2) // 2, (y1 + y2) // 2),
                       max(2, SIZES[val] // 2 + 1), (40, 40, 40), -1, cv2.LINE_AA)
        elif kind == "eraser":
            button(f, r, label, (120, 120, 140), border=dark if self._eraser else None)
        elif kind == "guide":
            on = getattr(self, "_show_guide", True)
            button(f, r, label, (40, 120, 230) if on else (120, 120, 140))
        else:
            col = {"clear": (200, 70, 70), "restart": (200, 120, 40),
                   "skip": (120, 120, 140), "save": (40, 150, 90)}[kind]
            button(f, r, label, col)


_orig_click_tool = PaintNode._click_tool
_orig_start_mode = PaintNode._start_mode
_orig_draw_guide = PaintNode._draw_guide


def _click_tool_gt(self, kind, val):
    if kind == "guide":
        self._show_guide = not getattr(self, "_show_guide", True)
        if self._show_guide:
            self._msg = self.tr("Guía activada", "Guide on")
        else:
            self._msg = self.tr("Guía oculta: así quedó tu dibujo",
                                "Guide hidden: this is your drawing")
        return
    return _orig_click_tool(self, kind, val)


def _start_mode_gt(self, mode, idx=None):
    _orig_start_mode(self, mode, idx)
    self._show_guide = True


def _draw_guide_gt(self, f):
    if getattr(self, "_show_guide", True):
        _orig_draw_guide(self, f)


PaintNode._toolbar = _toolbar_gt
PaintNode._draw_toolbar = _draw_toolbar_gt
PaintNode._click_tool = _click_tool_gt
PaintNode._start_mode = _start_mode_gt
PaintNode._draw_guide = _draw_guide_gt


# ─── LINEART_BLOCK: dibujo original (lineas) encima de los colores ───────────
LINEART_DIR = "/home/roberto/robotis_ws/src/YAREN2/yaren_paint/lineart"
_la_raw, _la_cache = {}, {}


def _la_mask(did, s):
    key = (did, round(s, 3))
    if key not in _la_cache:
        if did not in _la_raw:
            _la_raw[did] = cv2.imread(os.path.join(LINEART_DIR, did + ".png"),
                                      cv2.IMREAD_GRAYSCALE)
        m = _la_raw[did]
        if m is None:
            _la_cache[key] = None
        else:
            w, h = max(1, int(800 * s)), max(1, int(600 * s))
            a = cv2.resize(m, (w, h), interpolation=cv2.INTER_AREA).astype(np.float32) / 255.0
            _la_cache[key] = np.clip(a * 1.7, 0, 1)[..., None]
    return _la_cache[key]


def put_lineart(img, did, vp, color):
    a = _la_mask(did, vp[0])
    if a is None:
        return False
    x, y = int(vp[1]), int(vp[2])
    ih, iw = img.shape[:2]
    h, w = a.shape[:2]
    x0, y0, x1, y1 = max(0, x), max(0, y), min(iw, x + w), min(ih, y + h)
    if x1 <= x0 or y1 <= y0:
        return True
    aa = a[y0 - y:y1 - y, x0 - x:x1 - x]
    roi = img[y0:y1, x0:x1].astype(np.float32)
    img[y0:y1, x0:x1] = (roi * (1 - aa) + np.array(color, np.float32) * aa).astype(np.uint8)
    return True


_orig_compose = PaintNode._compose_color


def _compose_color_la(self):
    did = DRAWINGS[self._drawing]["id"]
    if _la_mask(did, VP_LOCAL[0]) is None:
        return _orig_compose(self)
    img = np.full((CH, CW, 3), 255, np.uint8)
    for sh, f in zip(self._shapes(), self._fills):
        if is_fillable(sh) and f is not None:
            draw_shape(img, sh, VP_LOCAL, f, -1)
    put_lineart(img, did, VP_LOCAL, (20, 20, 20))
    return img


def _draw_library_la(self, f):
    lv = getattr(self, "_lib_level", "bajo")
    for k, r in TAB_RECTS.items():
        es, en = LEVEL_NAMES[k]
        on = k == lv
        button(f, r, self.tr(es, en), LEVEL_COLORS[k] if on else (150, 160, 180),
               border=(20, 20, 20) if on else None, size=sc(20))
    idxs = _level_idx(lv)
    for j, (x1, y1, x2, y2) in enumerate(_level_cells(lv)):
        rrect(f, (x1, y1), (x2, y2), WHITE, 14, border=(170, 180, 200), t=2)
        cx, label_h = (x1 + x2) // 2, sc(34)
        if j < len(idxs):
            d = DRAWINGS[idxs[j]]
            vp = viewport((x1 + 8, y1 + 8, x2 - x1 - 16, y2 - y1 - 16 - label_h))
            if not put_lineart(f, d["id"], vp, (30, 30, 30)):
                for sh in d["shapes"]:
                    draw_shape(f, sh, vp, (30, 30, 30), 1)
            put_c(f, self._tn(d["label"]), cx, y2 - label_h // 2 - 2, sc(22), INK)
        else:
            put_c(f, "?", cx, (y1 + y2) // 2 - label_h // 2, sc(60), (240, 130, 30))
            put_c(f, self.tr("¡Sorpresa!", "Surprise!"), cx, y2 - label_h // 2 - 2, sc(22), INK)


PaintNode._compose_color = _compose_color_la
PaintNode._draw_library = _draw_library_la


# ─── GUIDE_LEFTOVERS: la guia no se pierde en lo que falto de un paso ────────
ADV_AT = 0.95   # fraccion del contorno para pasar al siguiente paso

_orig_advance = PaintNode._advance
_orig_record = PaintNode._record
_orig_start_mode_lo = PaintNode._start_mode
_orig_draw_guide_lo = PaintNode._draw_guide


def _cov_all(self, p):
    g = self.__dict__.setdefault("_all_grid", {})
    cx, cy = int(p[0] // CELL), int(p[1] // CELL)
    t2 = TOL * TOL
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for q in g.get((cx + dx, cy + dy), ()):
                if (q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2 <= t2:
                    return True
    return False


def _record_lo(self, a, b):
    _orig_record(self, a, b)
    g = self.__dict__.setdefault("_all_grid", {})
    n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 5))
    for i in range(n + 1):
        t = i / n
        p = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        g.setdefault((int(p[0] // CELL), int(p[1] // CELL)), []).append(p)


def _update_guided_lo(self):
    if self._screen != "guided":
        return
    left = getattr(self, "_left", [])
    if left:
        self._left = [p for p in left if not _cov_all(self, p)]
    if self._finished or not self._step_pts:
        return
    for k, p in enumerate(self._step_pts):
        if not self._cov[k] and self._is_covered(p):
            self._cov[k] = True
    self._progress = sum(self._cov) / len(self._cov)
    if self._progress >= ADV_AT:
        self._advance()


def _advance_lo(self, skipped=False):
    if not skipped and self._step_pts:
        self._left = getattr(self, "_left", []) + [
            p for p, c in zip(self._step_pts, self._cov) if not c]
    _orig_advance(self, skipped)
    left = getattr(self, "_left", [])
    if left and self._finished:
        self._msg = self.tr("¡Casi! Repasa los puntos naranjas que faltan",
                            "Almost! Retrace the orange dots")
    elif left:
        self._msg += self.tr(" · repasa los puntos naranjas", " · retrace the orange dots")


def _start_mode_lo(self, mode, idx=None):
    _orig_start_mode_lo(self, mode, idx)
    self._left = []
    self._all_grid = {}


def _draw_guide_lo(self, f):
    _orig_draw_guide_lo(self, f)
    if getattr(self, "_show_guide", True):
        for p in getattr(self, "_left", []):
            cv2.circle(f, (int(p[0]), int(p[1])), max(2, sc(5)),
                       (255, 140, 0), -1, cv2.LINE_AA)


PaintNode._record = _record_lo
PaintNode._update_guided = _update_guided_lo
PaintNode._advance = _advance_lo
PaintNode._start_mode = _start_mode_lo
PaintNode._draw_guide = _draw_guide_lo


# ─── PALETTE_SHADES: tonos por color + goma en todos los modos ───────────────
def _shades(base):
    if base == (0, 0, 0):
        return [(215, 215, 215), (170, 170, 170), (125, 125, 125),
                (85, 85, 85), (45, 45, 45), (0, 0, 0)]

    def mix(a, t):
        return tuple(int(c + (a - c) * t) for c in base)
    return [mix(255, .75), mix(255, .5), mix(255, .25), base, mix(0, .3), mix(0, .55)]


def _toolbar_pal(self):
    scr = self._screen
    cp = scr == "guided"
    y1, y2 = H - BOT_H + sc(10), H - sc(10)
    sw, st, g = (36, 38, 8) if cp else (46, 48, 10)
    x, items = 10, []

    for i in range(len(PALETTE)):
        items.append(("color", i, (x, y1, x + sc(sw), y2), None))
        x += sc(st)
    x += sc(8)

    def add(kind, val, w, label=None):
        nonlocal x
        items.append((kind, val, (x, y1, x + sc(w), y2), label))
        x += sc(w) + sc(g)

    en = self.is_english
    on = getattr(self, "_show_guide", True)

    # Ancho uniforme para todos los botones de texto
    bw = 100 if cp else 130

    if scr in ("free", "guided"):
        for i in range(len(SIZES)):
            add("size", i, 40 if cp else 50)

    if scr == "guided":
        add("skip", 0, bw, "Skip" if en else "Saltar")
        add("guide", 0, bw, ("With guide" if on else "No guide") if en else ("Con guía" if on else "Sin guía"))

    add("eraser", 0, bw, "Eraser" if en else "Borrador")

    if scr == "free":
        add("clear", 0, bw, "Clear all" if en else "Borrar todo")

    if scr in ("guided", "color"):
        add("restart", 0, bw, "Restart" if en else "Reiniciar")

    add("save", 0, bw, "Save" if en else "Guardar")

    return items


def _pop_geom(self):
    v = getattr(self, "_pop", None)
    if v is None:
        return None
    tb = [it for it in self._toolbar() if it[0] == "color" and it[1] == v]
    if not tb:
        return None
    cxm = (tb[0][2][0] + tb[0][2][2]) // 2
    n, sw, gap, pad = 6, sc(44), sc(6), sc(8)
    total = n * sw + (n - 1) * gap + 2 * pad
    x0 = max(4, min(W - total - 4, cxm - total // 2))
    y2 = H - BOT_H - sc(6)
    y1 = y2 - sw - 2 * pad
    items = []
    for i, c in enumerate(_shades(PALETTE[v])):
        x = x0 + pad + i * (sw + gap)
        items.append((c, (x, y1 + pad, x + sw, y2 - pad)))
    return (x0, y1, x0 + total, y2), items


def _draw_toolbar_pal(self, f):
    cv2.rectangle(f, (0, H - BOT_H), (W, H), BAR, -1)
    dark = (20, 20, 20)
    fam = getattr(self, "_fam", 4)
    for kind, val, r, label in self._toolbar():
        x1, y1, x2, y2 = r
        if kind == "color":
            sel = (not self._eraser) and fam == val
            col = self._color if sel else getattr(self, "_fam_shade", {}).get(val, PALETTE[val])
            rrect(f, (x1, y1), (x2, y2), col, 10, border=dark if sel else None, t=3)
        elif kind == "size":
            rrect(f, (x1, y1), (x2, y2), WHITE, 10,
                  border=dark if self._size_idx == val else None, t=3)
            cv2.circle(f, ((x1 + x2) // 2, (y1 + y2) // 2),
                       max(2, SIZES[val] // 2 + 1), (40, 40, 40), -1, cv2.LINE_AA)
        elif kind == "eraser":
            button(f, r, label, (120, 120, 140), border=dark if self._eraser else None)
        elif kind == "guide":
            on = getattr(self, "_show_guide", True)
            button(f, r, label, (40, 120, 230) if on else (120, 120, 140))
        else:
            col = {"clear": (200, 70, 70), "restart": (200, 120, 40),
                   "skip": (120, 120, 140), "save": (40, 150, 90)}[kind]
            button(f, r, label, col)
    geom = _pop_geom(self)
    if geom:
        bg, items = geom
        rrect(f, (bg[0], bg[1]), (bg[2], bg[3]), WHITE, 12, border=(120, 130, 150), t=2)
        for c, r in items:
            sel = (not self._eraser) and tuple(self._color) == tuple(c)
            rrect(f, (r[0], r[1]), (r[2], r[3]), c, 8,
                  border=dark if sel else (190, 195, 210), t=3 if sel else 1)


_prev_click_tool = PaintNode._click_tool
_prev_on_down = PaintNode._on_down
_prev_record = PaintNode._record


def _click_tool_pal(self, kind, val):
    if kind == "color":
        if getattr(self, "_just_closed", None) == val:
            self._just_closed = None
            return
        self._pop = val
        self._fam = val
        self._eraser = False
        self._color = self.__dict__.setdefault("_fam_shade", {}).get(val, PALETTE[val])
        return
    self._pop = None
    return _prev_click_tool(self, kind, val)


def _on_down_pal(self, pos):
    x, y = pos
    self._just_closed = None
    if getattr(self, "_pop", None) is not None:
        geom = _pop_geom(self) if self._screen in ("free", "guided", "color") else None
        if geom:
            for c, r in geom[1]:
                if _in(r, x, y):
                    self._color, self._eraser = c, False
                    self.__dict__.setdefault("_fam_shade", {})[self._pop] = c
                    self._pop = None
                    return
        self._just_closed = self._pop
        self._pop = None
        if _in((CX, CY, CX + CW, CY + CH), x, y):
            return
    return _prev_on_down(self, pos)


def _brush_pal(self):
    w = SIZES[self._size_idx]
    return (WHITE, max(12, w * 3)) if self._eraser else (self._color, w)


def _record_pal(self, a, b):
    if self._eraser:
        return
    _prev_record(self, a, b)


def _wrap_close_popup(name):
    prev = getattr(PaintNode, name)

    def w(self, *a, **k):
        self._pop = None
        return prev(self, *a, **k)
    setattr(PaintNode, name, w)


for _n in ("_go_menu", "_start_mode", "_open_library"):
    _wrap_close_popup(_n)

PaintNode._toolbar = _toolbar_pal
PaintNode._draw_toolbar = _draw_toolbar_pal
PaintNode._click_tool = _click_tool_pal
PaintNode._on_down = _on_down_pal
PaintNode._brush = _brush_pal
PaintNode._record = _record_pal


# =============================================================================
#  THEME_BLOCK: diseno plano (laminas de animalitos / sellos / figuras) + QR
#  - Fondo crema (menu verde lima), colores planos sin contornos
#  - Menu con botones tipo SELLO POSTAL y figuras geometricas con carita
#  - Guardar: un sello con tu dibujo viaja a un sobre -> QR para verlo en el celular
#    (servidor HTTP local en ~/yaren_paint; celular y robot en la misma red)
#  Pide:  pip install qrcode   (o segno).  Variables opcionales:
#    YAREN_PAINT_PORT=8765   YAREN_PAINT_HOST=<ip o nombre a usar en el QR>
# =============================================================================
import socket
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler

CREAM, CREAM_DK, PAPER = (252, 238, 208), (232, 212, 168), (255, 252, 244)
LIME, NAVY = (206, 221, 118), (33, 58, 84)
TEAL, MUST, ORNG, CORAL = (64, 166, 148), (250, 181, 35), (246, 124, 52), (240, 98, 98)
GRN, PURP, BLUE, PINKH = (102, 168, 70), (98, 96, 160), (54, 96, 170), (238, 156, 196)
OUT, YEL_DK = NAVY, CREAM_DK
AA = cv2.LINE_AA
TILE_COLS = [(255, 228, 120), (247, 164, 158), (190, 190, 232), (150, 215, 245),
             (250, 188, 218), (160, 225, 190), (255, 200, 150)]
SAVE_DIR = os.path.expanduser("~/yaren_paint")
SHARE_PORT = int(os.environ.get("YAREN_PAINT_PORT", "8765"))
SAVE_ANIM, SAVE_TIMEOUT = 1.8, 90.0
SV_DONE_R = R(110, 366, 240, 46)

# Menu: columna derecha (las figuras ocupan la izquierda)
MODE_RECTS = [(k, R(365, 140 + i * 92, 410, 78)) for i, k in enumerate(("free", "guided", "color"))]


def _dark(c, k=0.72):
    return tuple(int(v * k) for v in c)


def _confetti(bg, seed, n, menu=False, lib=False):
    rnd = random.Random(seed)
    cols = [CORAL, MUST, TEAL, BLUE, PINKH, ORNG, GRN]
    for _ in range(n):
        x, y, c, k = rnd.randint(8, W - 8), rnd.randint(8, H - 8), rnd.choice(cols), rnd.random()
        if menu and x > 335 and y < 425:      # no tapar el titulo ni los botones
            continue
        if lib and y > 395:                   # las colinas quedan limpias
            continue
        if k < 0.34:                                   # asterisco
            r = rnd.randint(7, 11)
            for a in (0, 60, 120):
                dx, dy = r * math.cos(math.radians(a)), r * math.sin(math.radians(a))
                cv2.line(bg, (int(x - dx), int(y - dy)), (int(x + dx), int(y + dy)), c, 2, AA)
        elif k < 0.67:                                 # punto
            cv2.circle(bg, (x, y), rnd.randint(3, 5), c, -1, AA)
        else:                                          # rayita
            a = math.radians(rnd.randint(0, 179))
            p1 = (int(x - 8 * math.cos(a)), int(y - 8 * math.sin(a)))
            p2 = (int(x + 8 * math.cos(a)), int(y + 8 * math.sin(a)))
            cv2.line(bg, p1, p2, c, 4, AA)
            cv2.circle(bg, p1, 2, c, -1, AA)
            cv2.circle(bg, p2, 2, c, -1, AA)


def _build_bg(kind):
    bg = np.empty((H, W, 3), np.uint8)
    bg[:] = LIME if kind == "menu" else CREAM
    if kind == "menu":      # formas grandes tono sobre tono
        for cx, cy, r, c in ((330, 440, 150, (196, 213, 106)), (760, 40, 110, (214, 228, 134)),
                             (40, 60, 90, (214, 228, 134)), (560, 300, 60, (201, 217, 113))):
            cv2.circle(bg, (cx, cy), r, c, -1, AA)
    elif kind == "lib":     # colinas + formas suaves
        for cx, cy, r, c in ((720, 70, 130, (250, 228, 190)), (60, 250, 110, (250, 228, 190)),
                             (420, 260, 90, (251, 232, 198))):
            cv2.circle(bg, (cx, cy), r, c, -1, AA)
        cv2.ellipse(bg, (210, 525), (380, 115), 0, 0, 360, (188, 224, 172), -1, AA)
        cv2.ellipse(bg, (640, 530), (400, 105), 0, 0, 360, (160, 206, 188), -1, AA)
    _confetti(bg, {"menu": 4, "lib": 5}.get(kind, 9), {"menu": 34, "lib": 40}.get(kind, 26), kind == "menu", kind == "lib")
    return bg


_BG_PLAIN, _BG_MENU, _BG_LIB = _build_bg("plain"), _build_bg("menu"), _build_bg("lib")


def _flat_button(img, rect, label, bg, fg=WHITE, border=None, size=sc(22), dx=0):
    x1, y1, x2, y2 = rect
    if border is not None:                           # seleccionado: aro mostaza
        rrect(img, (x1 - 4, y1 - 4), (x2 + 4, y2 + 8), MUST, 17)
    rrect(img, (x1, y1 + 4), (x2, y2 + 4), _dark(bg), 14)
    rrect(img, (x1, y1), (x2, y2), bg, 14)
    if label:
        sz = fit_size(label, (x2 - x1) - 14 - 2 * abs(dx), size)
        put_c(img, label, (x1 + x2) // 2 + dx, (y1 + y2) // 2, sz, fg)


button = _flat_button     # todas las pantallas usan ahora el boton plano


def _ease_out_back(x):
    c1 = 1.70158
    return 1 + (c1 + 1) * (x - 1) ** 3 + c1 * (x - 1) ** 2


def _wrap(text, size, maxw):
    lines, cur = [], ""
    for w in text.split():
        trial = (cur + " " + w).strip()
        if tsize(trial, size)[0] > maxw and cur:
            lines.append(cur)
            cur = w
        else:
            cur = trial
    if cur:
        lines.append(cur)
    return lines


# ── Figuras con carita (estilo laminas planas) ───────────────────────────────
def _tl(f, pts, w, col=NAVY):
    cv2.polylines(f, [np.array([(int(x), int(y)) for x, y in pts], np.int32)], False, col, w, AA)


def _poly(f, pts, col):
    cv2.fillPoly(f, [np.array([(int(x), int(y)) for x, y in pts], np.int32)], col, AA)


def _hand(f, x, y, s, col):
    x, y = int(x), int(y)
    for a in (-105, -75, -45):
        ex, ey = x + 14 * s * math.cos(math.radians(a)), y + 14 * s * math.sin(math.radians(a))
        cv2.line(f, (x, y), (int(ex), int(ey)), NAVY, max(2, int(3.5 * s)), AA)
    cv2.circle(f, (x, y), max(3, int(7 * s)), col, -1, AA)


def _arm(f, p0, p1, s, col):
    cv2.line(f, (int(p0[0]), int(p0[1])), (int(p1[0]), int(p1[1])), col, max(3, int(8 * s)), AA)
    _hand(f, p1[0], p1[1], s, col)


def _legs(f, cx, y0, gy, sep, s, col):
    w = max(4, 12 * s)
    for sx in (-1, 1):
        x = cx + sx * sep
        _poly(f, [(x - w / 2, y0), (x + w / 2, y0), (x + w / 2 - 2 * s, gy - 4 * s), (x - w / 2 - 2 * s, gy - 4 * s)], col)
        cv2.ellipse(f, (int(x + sx * 5 * s - 2 * s), int(gy - 3 * s)), (int(13 * s), int(5 * s)), 0, 0, 360, col, -1, AA)


def _eye(f, x, y, w, th=2):
    cv2.ellipse(f, (int(x), int(y)), (max(2, int(w)), max(2, int(w * .7))), 0, 15, 165, NAVY, th, AA)


def _smile(f, x, y, w, th=2):
    cv2.ellipse(f, (int(x), int(y)), (max(2, int(w)), max(2, int(w * .8))), 0, 20, 160, NAVY, th, AA)


def _happy_face(f, cx, cy, s, gap=15):
    th = max(2, int(2.5 * s))
    _eye(f, cx - gap * s, cy, 6 * s, th)
    _eye(f, cx + gap * s, cy, 6 * s, th)
    _smile(f, cx, cy + 9 * s, 9 * s, th)


def _hexagon(f, cx, cy, r, col, rot=15):
    _poly(f, [(cx + r * math.cos(math.radians(rot + i * 60)), cy + r * math.sin(math.radians(rot + i * 60)))
              for i in range(6)], col)


def friend_circle(f, cx, gy, s=1.0, t=0.0, ph=0.0):
    r = int(60 * s)
    cy = int(gy - 30 * s - r + 3 * s * math.sin(t * 3 + ph))
    _legs(f, cx, cy + r - 8 * s, gy, r * 0.38, s, ORNG)
    _arm(f, (cx + r * .85, cy), (cx + r * 1.3, cy - r * .55 + 4 * s * math.sin(t * 4 + ph)), s, ORNG)
    cv2.circle(f, (cx, cy), r, ORNG, -1, AA)
    brown = (120, 40, 10)
    for sx in (-1, 1):
        cv2.line(f, (int(cx + sx * r * .3), int(cy - r * .32)), (int(cx + sx * r * .3), int(cy - r * .12)),
                 brown, max(2, int(3 * s)), AA)
    cv2.ellipse(f, (cx, int(cy + r * .16)), (int(r * .3), int(r * .2)), 0, 0, 360, NAVY, -1, AA)
    cv2.ellipse(f, (cx, int(cy + r * .24)), (int(r * .16), int(r * .09)), 0, 0, 360, PINKH, -1, AA)


def friend_hex(f, cx, gy, s=1.0, t=0.0, ph=0.0):
    r = int(46 * s)
    cy = int(gy - 28 * s - r + 3 * s * math.sin(t * 3 + ph))
    _legs(f, cx, cy + r * .6, gy, r * 0.35, s, PINKH)
    _arm(f, (cx - r * .8, cy), (cx - r * 1.35, cy - r * .6 + 4 * s * math.sin(t * 4 + ph)), s, PINKH)
    _hexagon(f, cx, cy, r, PINKH)
    _happy_face(f, cx, cy - r * .1, s * .9, 13)


# ── Personajes planos (laminas de figuras con carita) ────────────────────────
BLU2, YEL2, RED2 = (36, 76, 172), (252, 196, 32), (250, 100, 34)
PNK2, BOOT = (250, 134, 192), (244, 150, 200)


def _limb(f, p0, p1, w, col):
    w = max(2, int(w))
    cv2.line(f, (int(p0[0]), int(p0[1])), (int(p1[0]), int(p1[1])), col, w, AA)
    for p in (p0, p1):
        cv2.circle(f, (int(p[0]), int(p[1])), w // 2, col, -1, AA)


def _googly(f, cx, cy, gap, r, lx=1.0, ly=0.3):
    for sx in (-1, 1):
        x = cx + sx * gap
        cv2.ellipse(f, (int(x), int(cy)), (max(2, int(r)), max(2, int(r * 1.15))), 0, 0, 360, PAPER, -1, AA)
        cv2.circle(f, (int(x + lx * r * .35), int(cy + ly * r * .3)), max(2, int(r * .58)), NAVY, -1, AA)


def char_mountain(f, cx, gy, s=1.0, t=0.0, ph=0.0):
    w = math.sin(t * 4 + ph)
    base = gy - 56 * s
    for sx in (-1, 1):
        x = cx + sx * 18 * s
        lift = 4 * s * max(0.0, w * sx)
        _limb(f, (x, base), (x, gy - 12 * s - lift), 7 * s, BLU2)
        rrect(f, (int(x - 10 * s), int(gy - 15 * s - lift)), (int(x + 10 * s), int(gy - lift)), BOOT,
              max(2, int(5 * s)))
    _poly(f, [(cx - 62 * s, base), (cx + 62 * s, base), (cx, base - 100 * s)], BLU2)
    _poly(f, [(cx - 60 * s, base), (cx - 44 * s, base - 50 * s), (cx - 24 * s, base - 24 * s)], BLU2)
    _poly(f, [(cx + 60 * s, base), (cx + 42 * s, base - 46 * s), (cx + 22 * s, base - 24 * s)], BLU2)
    _poly(f, [(cx - 4 * s, base - 98 * s), (cx - 30 * s, base - 52 * s), (cx - 16 * s, base - 42 * s),
              (cx + 2 * s, base - 70 * s)], (18, 40, 110))
    _googly(f, cx, base - 34 * s, 16 * s, 7 * s, 0.0, 0.0)
    cv2.circle(f, (int(cx), int(base - 22 * s)), max(2, int(4 * s)), YEL2, -1, AA)
    _smile(f, cx, base - 16 * s, 9 * s, max(2, int(2.5 * s)))


def char_blob(f, cx, gy, s=1.0, t=0.0, ph=0.0):
    sw = math.sin(t * 3 + ph)
    r = 40 * s
    cy = gy - 72 * s - 3 * s * abs(sw)
    _limb(f, (cx + 10 * s, cy + r * .7), (cx + 8 * s, gy - 4 * s), 9 * s, YEL2)
    _limb(f, (cx + 8 * s, gy - 4 * s), (cx + 26 * s, gy - 4 * s), 9 * s, YEL2)
    _limb(f, (cx - 8 * s, cy + r * .7), (cx - 44 * s, cy + r * .5 + 14 * s + 8 * s * sw), 9 * s, YEL2)
    _limb(f, (cx - r * .6, cy - r * .6), (cx - r * 1.25, cy - r * 1.35 - 5 * s * sw), 8 * s, YEL2)
    _limb(f, (cx + r * .7, cy + r * .1), (cx + r * 1.5, cy + r * .85 + 5 * s * sw), 8 * s, YEL2)
    cv2.circle(f, (int(cx), int(cy)), int(r), RED2, -1, AA)
    for a in range(0, 360, 36):
        cv2.circle(f, (int(cx + r * .93 * math.cos(math.radians(a))), int(cy + r * .93 * math.sin(math.radians(a)))),
                   max(2, int(r * .2)), RED2, -1, AA)
    rnd = random.Random(7)
    for _ in range(9):
        ox, oy = rnd.uniform(-.5, .5) * r, rnd.uniform(-.5, .5) * r
        cv2.ellipse(f, (int(cx + ox), int(cy + oy)),
                    (max(3, int(rnd.uniform(.2, .4) * r)), max(3, int(rnd.uniform(.15, .3) * r))),
                    rnd.randint(0, 180), 0, rnd.randint(200, 330), (222, 58, 32), 1, AA)
    _googly(f, cx - 2 * s, cy - 3 * s, 13 * s, 7.5 * s, 0.9, 0.5)


def char_flower(f, cx, gy, s=1.0, t=0.0, ph=0.0):
    sw = math.sin(t * 3 + ph)
    top = gy - 150 * s
    hx, hy = cx + 2 * s * sw, top + 36 * s
    _limb(f, (cx - 24 * s, top + 80 * s), (cx - 52 * s, top + 112 * s + 4 * s * sw), 7 * s, BLU2)
    _limb(f, (cx + 24 * s, top + 80 * s), (cx + 52 * s, top + 112 * s - 4 * s * sw), 7 * s, BLU2)
    _poly(f, [(cx - 26 * s, top + 68 * s), (cx + 26 * s, top + 68 * s), (cx + 30 * s, gy), (cx + 3 * s, gy),
              (cx, top + 106 * s), (cx - 3 * s, gy), (cx - 30 * s, gy)], BLU2)
    for a in range(0, 360, 60):
        cv2.circle(f, (int(hx + 24 * s * math.cos(math.radians(a + 15))), int(hy + 24 * s * math.sin(math.radians(a + 15)))),
                   max(3, int(17 * s)), YEL2, -1, AA)
    cv2.circle(f, (int(hx), int(hy)), int(26 * s), YEL2, -1, AA)
    cv2.ellipse(f, (int(hx), int(hy)), (int(23 * s), int(18 * s)), 0, 0, 360, PAPER, -1, AA)
    for sx in (-1, 1):
        cv2.circle(f, (int(hx + sx * 8 * s), int(hy - 3 * s)), max(2, int(2.8 * s)), NAVY, -1, AA)
    _smile(f, hx, hy + 3 * s, 6 * s, max(2, int(2 * s)))


def char_star(f, cx, gy, s=1.0, t=0.0, ph=0.0):
    sw = math.sin(t * 3 + ph)
    r = 56 * s
    cy = gy - 76 * s
    for sx in (-1, 1):
        _poly(f, [(cx + sx * 6 * s, cy + r * .4), (cx + sx * 26 * s, cy + r * .4),
                  (cx + sx * (34 + 5 * sw * sx) * s, gy), (cx + sx * (14 + 5 * sw * sx) * s, gy)], (248, 110, 30))
    pts = []
    for i in range(10):
        ang = -math.pi / 2 + i * math.pi / 5
        rr = r if i % 2 == 0 else r * 0.46
        pts.append((cx + rr * math.cos(ang), cy + rr * math.sin(ang)))
    _poly(f, pts, YEL2)
    _googly(f, cx, cy, 13 * s, 7 * s, -0.8, 0.2)
    cv2.circle(f, (int(cx - 4 * s), int(cy + 9 * s)), max(2, int(3 * s)), (248, 110, 30), -1, AA)


def char_stack(f, cx, gy, s=1.0, t=0.0, ph=0.0):
    sw = math.sin(t * 4 + ph)
    top = gy - 130 * s
    _limb(f, (cx - 14 * s, gy - 54 * s), (cx - 28 * s, gy - 28 * s), 7 * s, BLU2)
    _limb(f, (cx - 28 * s, gy - 28 * s), (cx - 18 * s, gy - 3 * s), 7 * s, BLU2)
    _limb(f, (cx + 14 * s, gy - 54 * s), (cx + 30 * s, gy - 30 * s + 5 * s * sw), 7 * s, BLU2)
    _limb(f, (cx + 30 * s, gy - 30 * s + 5 * s * sw), (cx + 24 * s, gy - 3 * s), 7 * s, BLU2)
    for sx in (-1, 1):
        pts = [(cx + sx * 40 * s, top + 44 * s), (cx + sx * 54 * s, top + 14 * s - 3 * s * sw),
               (cx + sx * 36 * s, top - 6 * s), (cx + sx * 8 * s, top - 2 * s)]
        cv2.polylines(f, [np.array([(int(x), int(y)) for x, y in pts], np.int32)], False, BLU2,
                      max(3, int(7 * s)), AA)
    for i in range(3):
        y = top + 8 * s + i * 20 * s
        rrect(f, (int(cx - 40 * s), int(y)), (int(cx + 40 * s), int(y + 28 * s)), PNK2, max(3, int(11 * s)))
        cv2.line(f, (int(cx - 28 * s), int(y + 9 * s)), (int(cx + 24 * s), int(y + 7 * s)), (255, 184, 218), 2, AA)
    _googly(f, cx, top + 40 * s, 12 * s, 6 * s, -0.5, -0.6)
    _smile(f, cx, top + 47 * s, 6 * s, max(2, int(2 * s)))


def char_dome(f, cx, gy, s=1.0, t=0.0, ph=0.0):
    sw = math.sin(t * 3 + ph)
    for sx in (-1, 1):
        _limb(f, (cx + sx * 10 * s, gy - 46 * s), (cx + sx * 12 * s, gy - 2 * s), 7 * s, YEL2)
    _poly(f, [(cx - 36 * s, gy - 90 * s), (cx + 36 * s, gy - 90 * s), (cx, gy - 44 * s)], BLU2)
    for sx in (-1, 1):
        _limb(f, (cx + sx * 56 * s, gy - 86 * s), (cx + sx * 68 * s, gy - 100 * s + 4 * s * sw), 7 * s, YEL2)
    for dx in (-20, 0, 20):
        cv2.circle(f, (int(cx + dx * s), int(gy - 148 * s + abs(dx) * .12 * s)), max(3, int(11 * s)), RED2, -1, AA)
    cv2.ellipse(f, (int(cx), int(gy - 88 * s)), (int(62 * s), int(62 * s)), 0, 180, 360, RED2, -1, AA)
    for sx in (-1, 1):
        cv2.ellipse(f, (int(cx + sx * 15 * s), int(gy - 108 * s)), (int(7 * s), int(5 * s)), 0, 20, 160, NAVY,
                    max(2, int(2.2 * s)), AA)
    cv2.circle(f, (int(cx), int(gy - 96 * s)), max(2, int(4 * s)), NAVY, 2, AA)


def _sparkle(f, x, y, r, col):
    _poly(f, [(x, y - r), (x + r * .25, y - r * .25), (x + r, y), (x + r * .25, y + r * .25), (x, y + r),
              (x - r * .25, y + r * .25), (x - r, y), (x - r * .25, y - r * .25)], col)


def _draw_friends(f, t):
    """Poster de personajes (menu, lado izquierdo)."""
    s = 0.82
    cast = ((char_mountain, 92, 150), (char_blob, 252, 150),
            (char_flower, 92, 312), (char_star, 252, 312),
            (char_stack, 92, 468), (char_dome, 252, 468))
    for i, (fn, x, gy) in enumerate(cast):
        fn(f, x, gy, s, t, i * 1.1)
    for i, (x, y, c) in enumerate(((168, 60, MUST), (30, 232, CORAL), (176, 390, TEAL), (318, 250, BLUE))):
        _sparkle(f, x, y, 7 + 3 * math.sin(t * 3 + i), c)


def _draw_lib_deco(f, t):
    """Personajitos sobre las colinas (fondo de la biblioteca)."""
    cast = (char_star, char_blob, char_dome, char_mountain, char_flower, char_stack, char_star)
    for i, (fn, x) in enumerate(zip(cast, (62, 170, 282, 400, 520, 640, 752))):
        fn(f, x, 464, 0.40, t, i * 1.3)
    for i, (x, y, c) in enumerate(((470, 74, MUST), (560, 86, CORAL), (690, 68, TEAL), (772, 88, BLUE))):
        _sparkle(f, x, y, 6 + 3 * math.sin(t * 3 + i), c)


# ── Iconos planos de los botones ─────────────────────────────────────────────
def icon_easel(f, cx, cy, bgc):
    for dx in (-17, 0, 17):
        _tl(f, [(cx + dx * 0.4, cy + 10), (cx + dx, cy + 27)], 4, NAVY)
    rrect(f, (cx - 19, cy - 24), (cx + 19, cy + 12), PAPER, 4)
    _tl(f, [(cx - 11, cy), (cx - 4, cy - 12), (cx + 3, cy - 2), (cx + 12, cy - 14)], 3, CORAL)


def icon_pencil(f, cx, cy, bgc):
    a = math.radians(-45)
    ca, sa = math.cos(a), math.sin(a)

    def tr(pts):
        return [(cx - 6 + (x * ca - y * sa) * .7, cy + 6 + (x * sa + y * ca) * .7) for x, y in pts]
    for pts, col in (([(-24, 0), (0, -10), (0, 10)], CREAM), ([(-24, 0), (-13, -4.5), (-13, 4.5)], NAVY),
                     ([(0, -10), (46, -10), (46, 10), (0, 10)], MUST), ([(46, -10), (56, -10), (56, 10), (46, 10)], (200, 205, 215)),
                     ([(56, -10), (68, -10), (68, 10), (56, 10)], PINKH)):
        _poly(f, tr(pts), col)


def icon_palette(f, cx, cy, bgc):
    cv2.ellipse(f, (cx, cy), (25, 20), 0, 0, 360, PAPER, -1, AA)
    for dx, dy, c in ((-13, -4, CORAL), (-4, -11, MUST), (7, -10, GRN), (14, -1, BLUE)):
        cv2.circle(f, (cx + dx, cy + dy), 4, c, -1, AA)
    cv2.circle(f, (cx + 4, cy + 8), 5, bgc, -1, AA)


def _stamp_button(f, r, label, col, icon, bgc=LIME):
    x1, y1, x2, y2 = r
    rrect(f, (x1 + 2, y1 + 6), (x2 + 2, y2 + 6), _dark(bgc, .82), 10)
    rrect(f, (x1, y1), (x2, y2), PAPER, 10)
    for x in range(x1 + 9, x2 - 4, 12):
        cv2.circle(f, (x, y1), 4, bgc, -1, AA)
        cv2.circle(f, (x, y2), 4, bgc, -1, AA)
    for y in range(y1 + 9, y2 - 4, 12):
        cv2.circle(f, (x1, y), 4, bgc, -1, AA)
        cv2.circle(f, (x2, y), 4, bgc, -1, AA)
    rrect(f, (x1 + 8, y1 + 8), (x2 - 8, y2 - 8), col, 7)
    cy = (y1 + y2) // 2
    icon(f, x1 + 54, cy, col)
    put_ml(f, label, x1 + 98, cy, fit_size(label, (x2 - x1) - 118, sc(32)), WHITE)


# ── Pantallas ────────────────────────────────────────────────────────────────
def _draw_menu_theme(self, f):
    t = time.time()
    _draw_friends(f, t)
    tile, gap, x, k = 38, 4, 352, 0
    cols = [TEAL, MUST, CORAL, BLUE, GRN, PURP, ORNG, TEAL, CORAL, MUST]
    for word in ("YAREN", "PAINT"):
        for ch in word:
            yy = 38 + int(3 * math.sin(t * 2.5 + k * 0.7))
            rrect(f, (x, yy + 4), (x + tile, yy + tile + 4), _dark(cols[k]), 9)
            rrect(f, (x, yy), (x + tile, yy + tile), cols[k], 9)
            put_c(f, ch, x + tile // 2, yy + tile // 2, sc(36), WHITE)
            x += tile + gap
            k += 1
        x += 12
    put_c(f, self.tr("¡Dibuja, aprende y diviértete!", "Draw, learn and have fun!"), 568, 106, sc(24), NAVY)
    specs = {
        "free": (self.tr("Dibujo libre", "Free draw"), TEAL, icon_easel),
        "guided": (self.tr("Dibujar con Yaren", "Draw with Yaren"), BLUE, icon_pencil),
        "color": (self.tr("Colorear", "Color in"), CORAL, icon_palette),
    }
    for key, r in MODE_RECTS:
        label, col, icon = specs[key]
        _stamp_button(f, r, label, col, icon)
    button(f, QUIT_R, self.tr("Salir", "Exit"), NAVY)


def _draw_topbar_theme(self, f):
    cv2.rectangle(f, (0, 0), (W, TOP_H), NAVY, -1)
    button(f, MENU_R, self.tr("< Menú", "< Menu"), CORAL)
    if self._screen in ("guided", "color"):
        button(f, OTHER_R, self.tr("Otro dibujo", "Another"), TEAL)
    fx, fy, fr = sc(345), TOP_H // 2, sc(23)
    cv2.circle(f, (fx, fy), fr, MUST, -1, AA)
    _eye(f, fx - fr * .38, fy - 2, fr * .22, 2)
    _eye(f, fx + fr * .38, fy - 2, fr * .22, 2)
    _smile(f, fx, fy + fr * .22, fr * .34, 2)
    bx1 = sc(385)
    rrect(f, (bx1, sc(8)), (W - 10, TOP_H - sc(8)), PAPER, 12)
    _poly(f, [(bx1 + 3, TOP_H // 2 - 7), (bx1 - 11, TOP_H // 2), (bx1 + 3, TOP_H // 2 + 7)], PAPER)
    msg = self._msg or self.tr("¡Hola! Soy Yaren. ¿Qué quieres hacer hoy?",
                               "Hi! I'm Yaren. What do you want to do today?")
    size = next((sc(s) for s in (30, 26, 22, 18, 15)
                 if tsize(msg, sc(s))[0] <= (W - 10 - bx1) - 28), sc(15))
    put_ml(f, msg, bx1 + 14, TOP_H // 2, size, NAVY)


def _draw_library_theme(self, f):
    _draw_lib_deco(f, time.time())
    lv = getattr(self, "_lib_level", "bajo")
    for k, r in TAB_RECTS.items():
        es, en = LEVEL_NAMES[k]
        on = k == lv
        button(f, r, self.tr(es, en), LEVEL_COLORS[k] if on else (170, 165, 150),
               border=True if on else None, size=sc(20))
    idxs = _level_idx(lv)
    for j, (x1, y1, x2, y2) in enumerate(_level_cells(lv)):
        col = TILE_COLS[j % len(TILE_COLS)]
        rrect(f, (x1, y1 + 5), (x2, y2 + 5), _dark(col, .8), 16)
        rrect(f, (x1, y1), (x2, y2), col, 16)
        cx, label_h = (x1 + x2) // 2, sc(34)
        if j < len(idxs):
            d = DRAWINGS[idxs[j]]
            rrect(f, (x1 + 7, y1 + 7), (x2 - 7, y2 - label_h - 2), PAPER, 11)
            vp = viewport((x1 + 12, y1 + 12, x2 - x1 - 24, y2 - y1 - 24 - label_h))
            if not put_lineart(f, d["id"], vp, (30, 30, 30)):
                for sh in d["shapes"]:
                    draw_shape(f, sh, vp, (30, 30, 30), 1)
            label = self._tn(d["label"])
        else:
            rrect(f, (x1 + 7, y1 + 7), (x2 - 7, y2 - label_h - 2), CORAL, 11)
            put_c(f, "?", cx, (y1 + y2 - label_h) // 2, sc(64), PAPER)
            label = self.tr("¡Sorpresa!", "Surprise!")
        put_c(f, label, cx, y2 - label_h // 2 - 1, fit_size(label, x2 - x1 - 16, sc(20)), NAVY)


def _swatch(f, r, col, sel):
    x1, y1, x2, y2 = r
    cx, cy, rad = (x1 + x2) // 2, (y1 + y2) // 2, min(x2 - x1, y2 - y1) // 2 - 1
    if sel:
        cv2.circle(f, (cx, cy), rad + 4, PAPER, -1, AA)
        cv2.circle(f, (cx, cy), rad + 1, NAVY, -1, AA)
    cv2.circle(f, (cx, cy), rad, col, -1, AA)


def _draw_toolbar_theme(self, f):
    cv2.rectangle(f, (0, H - BOT_H), (W, H), NAVY, -1)
    fam = getattr(self, "_fam", 4)
    for kind, val, r, label in self._toolbar():
        x1, y1, x2, y2 = r
        if kind == "color":
            sel = (not self._eraser) and fam == val
            col = self._color if sel else getattr(self, "_fam_shade", {}).get(val, PALETTE[val])
            _swatch(f, r, col, sel)
        elif kind == "size":
            sel = self._size_idx == val
            if sel:
                rrect(f, (x1 - 3, y1 - 3), (x2 + 3, y2 + 3), MUST, 13)
            rrect(f, (x1, y1), (x2, y2), PAPER, 10)
            cv2.circle(f, ((x1 + x2) // 2, (y1 + y2) // 2), max(2, SIZES[val] // 2 + 1), NAVY, -1, AA)
        elif kind == "eraser":
            button(f, r, label, PURP, border=True if self._eraser else None)
        elif kind == "guide":
            on = getattr(self, "_show_guide", True)
            button(f, r, label, TEAL if on else PURP)
        else:
            col = {"clear": CORAL, "restart": ORNG, "skip": PURP, "save": GRN}[kind]
            button(f, r, label, col)
    geom = _pop_geom(self)
    if geom:
        bg, items = geom
        rrect(f, (bg[0], bg[1] + 4), (bg[2], bg[3] + 4), _dark(CREAM, .8), 14)
        rrect(f, (bg[0], bg[1]), (bg[2], bg[3]), PAPER, 14)
        for c, r in items:
            sel = (not self._eraser) and tuple(self._color) == tuple(c)
            if sel:
                x1, y1, x2, y2 = r
                cx, cy, rad = (x1 + x2) // 2, (y1 + y2) // 2, min(x2 - x1, y2 - y1) // 2
                cv2.circle(f, (cx, cy), rad + 3, NAVY, -1, AA)
            _swatch(f, r, c, False)


def _render_theme(self):
    scr = self._screen
    f = {"menu": _BG_MENU, "library": _BG_LIB}.get(scr, _BG_PLAIN).copy()
    if scr == "menu":
        self._draw_menu(f)
    elif scr == "library":
        self._draw_library(f)
    else:
        rrect(f, (CX - 5, CY + 1), (CX + CW + 5, CY + CH + 11), CREAM_DK, 14)
        rrect(f, (CX - 5, CY - 5), (CX + CW + 5, CY + CH + 5), PAPER, 14)
        f[CY:CY + CH, CX:CX + CW] = self._compose_color() if scr == "color" else self._canvas
        if scr == "guided":
            self._draw_guide(f)
        self._draw_toolbar(f)
    if scr != "menu":
        self._draw_topbar(f)
    if getattr(self, "_sv", None):
        self._draw_save_overlay(f)
    return f


# ── Compartir por QR: servidor HTTP local ────────────────────────────────────
_SHARE_HTML = """<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>YAREN PAINT</title>
<style>body{margin:0;background:#fceed0;font-family:sans-serif;text-align:center;color:#213a54}
h1{margin:18px 0 8px}img{max-width:92%;border-radius:18px;box-shadow:0 6px 0 #e8d4a8;background:#fff}
a{display:inline-block;margin:16px;padding:12px 26px;background:#40a694;color:#fff;
border-radius:30px;text-decoration:none;font-weight:bold}</style></head>
<body><h1>YAREN PAINT</h1><img src="/@IMG@"><br>
<a href="/@IMG@" download>Descargar / Download</a></body></html>"""


class _ShareHandler(SimpleHTTPRequestHandler):
    def __init__(self, *a, **k):
        super().__init__(*a, directory=SAVE_DIR, **k)

    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_HEAD(self):
        self._send(404, b"", "text/plain")

    def do_GET(self):
        p = self.path.split("?")[0]
        if p.startswith("/v/"):
            name = os.path.basename(p[3:])
            if name.endswith(".png") and os.path.isfile(os.path.join(SAVE_DIR, name)):
                return self._send(200, _SHARE_HTML.replace("@IMG@", name).encode(),
                                  "text/html; charset=utf-8")
        elif p.count("/") == 1 and p.endswith(".png"):
            return super().do_GET()
        self._send(404, b"Not found", "text/plain")


def _lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("10.255.255.255", 1))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


def _ensure_server(self):
    if getattr(self, "_share_srv", None):
        return self._share_port
    os.makedirs(SAVE_DIR, exist_ok=True)
    for port in range(SHARE_PORT, SHARE_PORT + 6):
        try:
            srv = ThreadingHTTPServer(("0.0.0.0", port), _ShareHandler)
        except OSError:
            continue
        self._share_srv, self._share_port = srv, port
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        return port
    return None


def _qr_matrix(text):
    try:
        import qrcode
        qr = qrcode.QRCode(border=0, error_correction=qrcode.constants.ERROR_CORRECT_M)
        qr.add_data(text)
        qr.make(fit=True)
        return np.array(qr.get_matrix(), dtype=np.uint8)
    except Exception:
        pass
    try:
        import segno
        return np.array([list(r) for r in segno.make(text, error="m", micro=False).matrix], dtype=np.uint8)
    except Exception:
        pass
    try:
        enc = cv2.QRCodeEncoder.create()
        dark = enc.encode(text) < 128
        ys, xs = np.where(dark)
        return dark[ys.min():ys.max() + 1, xs.min():xs.max() + 1].astype(np.uint8)
    except Exception:
        return None


def _qr_image(text, px=232):
    m = _qr_matrix(text)
    if m is None:
        return None
    scale = max(2, px // (m.shape[0] + 8))
    big = np.kron(m, np.ones((scale, scale), np.uint8))
    rgb = np.where(big[..., None] == 1, np.array((10, 15, 45), np.uint8), np.array((255, 255, 255), np.uint8))
    q = 4 * scale
    return np.pad(rgb.astype(np.uint8), ((q, q), (q, q), (0, 0)), constant_values=255)


def _share_url(self, name):
    port = _ensure_server(self)
    if port is None:
        return None
    host = os.environ.get("YAREN_PAINT_HOST") or _lan_ip()
    return f"http://{host}:{port}/v/{name}", f"{host}:{port}"


# ── Guardar: animacion + QR ──────────────────────────────────────────────────
def _save_theme(self):
    if self._screen not in ("free", "guided", "color"):
        return
    os.makedirs(SAVE_DIR, exist_ok=True)
    name = datetime.now().strftime("yaren_paint_%Y%m%d_%H%M%S.png")
    path = os.path.join(SAVE_DIR, name)
    img = (self._compose_color() if self._screen == "color" else self._canvas).copy()
    cv2.imwrite(path, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    self._msg = self.tr("¡Guardé tu dibujo!", "Saved your drawing!")
    self.get_logger().info(f'Guardado en {path}')
    url = None
    try:
        url = _share_url(self, name)
    except Exception as e:
        self.get_logger().warn(f'No se pudo compartir: {e}')
    qr = _qr_image(url[0]) if url else None
    self._pop = None
    self._sv = {"t0": time.time(), "thumb": img, "qr": qr, "hostport": url[1] if url else "",
                "path": path, "offline": bool(url and url[1].startswith("127."))}


def _paste(f, img, cx, cy):
    h, w = img.shape[:2]
    x, y = int(cx - w / 2), int(cy - h / 2)
    x0, y0, x1, y1 = max(0, x), max(0, y), min(W, x + w), min(H, y + h)
    if x1 > x0 and y1 > y0:
        f[y0:y1, x0:x1] = img[y0 - y:y1 - y, x0 - x:x1 - x]


def _stamp(f, img, cx, cy, w, col=TEAL, card=CREAM):
    """Sello postal con tu dibujo dentro (borde perforado)."""
    h = max(2, int(w * img.shape[0] / img.shape[1]))
    m = max(0.4, min(1.0, w / 200))
    pad, ins, pr, st = int(16 * m), int(8 * m), max(2, int(3.6 * m)), max(6, int(11 * m))
    x1, y1, x2, y2 = int(cx - w / 2 - pad), int(cy - h / 2 - pad), int(cx + w / 2 + pad), int(cy + h / 2 + pad)
    rrect(f, (x1 + 3, y1 + 6), (x2 + 3, y2 + 6), _dark(card, .82), 6)
    rrect(f, (x1, y1), (x2, y2), PAPER, 6)
    for x in range(x1 + st // 2 + 2, x2 - 2, st):
        cv2.circle(f, (x, y1), pr, card, -1, AA)
        cv2.circle(f, (x, y2), pr, card, -1, AA)
    for y in range(y1 + st // 2 + 2, y2 - 2, st):
        cv2.circle(f, (x1, y), pr, card, -1, AA)
        cv2.circle(f, (x2, y), pr, card, -1, AA)
    rrect(f, (x1 + ins, y1 + ins), (x2 - ins, y2 - ins), col, 4)
    _paste(f, cv2.resize(img, (max(2, int(w)), h), interpolation=cv2.INTER_AREA), cx, cy)


def _envelope(f, cx, cy, thumb, e, u):
    rrect(f, (cx - 105, cy - 62), (cx + 105, cy + 78), _dark(TEAL, .8), 12)
    rrect(f, (cx - 105, cy - 68), (cx + 105, cy + 68), TEAL, 12)
    _poly(f, [(cx - 100, cy + 66), (cx, cy - 4), (cx + 100, cy + 66)], (82, 186, 168))
    _poly(f, [(cx - 102, cy - 66), (cx + 102, cy - 66), (cx, cy + 6)], (46, 140, 126))
    sw = 200 - 144 * e
    sx = 250 + (cx + 66 - 250) * e
    sy = 215 + (cy - 30 - 215) * e
    _stamp(f, thumb, sx, sy, sw, MUST, TEAL if e > 0.98 else CREAM)
    if u > 0.85:    # matasellos
        a = min(1.0, (u - 0.85) / 0.15)
        pc = (int(cx + 52), int(cy - 20))
        cv2.circle(f, pc, int(24 * a), NAVY, 2, AA)
        for i in range(3):
            y = cy - 32 + i * 9
            _tl(f, [(cx + 14 - 2, y), (cx + 4 - 4, y + 3), (cx - 10, y), (cx - 22, y + 3)], 2, NAVY)


def _draw_saving(self, f, sv, t):
    u = min(1.0, t / SAVE_ANIM)
    e = u * u * (3 - 2 * u)
    n = int(t * 3) % 4
    put_c(f, self.tr("Guardando", "Saving") + "." * n + " " * (3 - n), 400, 92, sc(50), NAVY)
    friend_circle(f, 150, 420, 0.74, t, 0.0)
    friend_hex(f, 655, 420, 0.8, t, 1.6)
    _envelope(f, 545, 235, sv["thumb"], e, u)
    rrect(f, (220, 352), (580, 374), CREAM_DK, 11)
    wfill = int(360 * u)
    if wfill > 14:
        rrect(f, (220, 352), (220 + wfill, 374), TEAL, 11)


def _draw_saved(self, f, sv, tq):
    k = _ease_out_back(min(1.0, tq / 0.55))
    if tq < 2.6:   # confeti plano
        rnd = random.Random(11)
        cols = [CORAL, MUST, TEAL, BLUE, PINKH]
        for i in range(26):
            x0, sp, r = rnd.randint(85, 715), rnd.uniform(70, 150), rnd.randint(3, 6)
            yy = 55 + (tq * sp + rnd.randint(0, 300)) % 370
            if yy < 425:
                cv2.circle(f, (int(x0 + 8 * math.sin(tq * 4 + i)), int(yy)), r, cols[i % 5], -1, AA)
    title = self.tr("¡Guardado!", "Saved!")
    tw = tsize(title, sc(52))[0]
    put_c(f, title, 238, 92, sc(52), NAVY)
    bx = 238 - tw // 2 - 32
    cv2.circle(f, (bx, 92), 21, TEAL, -1, AA)
    _tl(f, [(bx - 9, 92), (bx - 2, 99), (bx + 10, 85)], 4, WHITE)
    _stamp(f, sv["thumb"], 238, 190, 220, MUST, CREAM)
    y = 276
    for line in _wrap(self.tr("Escanea el código QR con tu celular para ver tu dibujo",
                              "Scan the QR code with your phone to see your drawing"), sc(22), 280):
        put_c(f, line, 238, y, sc(22), NAVY)
        y += sc(28)
    for line in _wrap(self.tr("El celular y el robot deben estar en la misma red WiFi",
                              "Your phone and the robot must be on the same WiFi"), sc(17), 270):
        put_c(f, line, 238, y + 4, sc(17), (110, 120, 140))
        y += sc(21)
    button(f, SV_DONE_R, self.tr("¡Listo!", "Done!"), GRN, size=sc(30))

    qr = sv["qr"]
    cx, cy = 560, 232
    if qr is None:
        rrect(f, (430, 112), (690, 352), PAPER, 16)
        for i, line in enumerate(_wrap(self.tr("No se pudo crear el enlace. El dibujo quedó guardado en la carpeta yaren_paint.",
                                               "Could not create the link. Your drawing was saved in the yaren_paint folder."),
                                       sc(20), 220)):
            put_c(f, line, cx, 170 + i * sc(26), sc(20), NAVY)
    else:
        qh, qw = qr.shape[:2]
        side = qw + 20
        kk = max(0.05, k)
        sd = int(side * kk)
        rrect(f, (cx - sd // 2, cy - sd // 2 + 6), (cx + sd // 2, cy + sd // 2 + 6), _dark(CREAM, .8), 16)
        rrect(f, (cx - sd // 2, cy - sd // 2), (cx + sd // 2, cy + sd // 2), WHITE, 16)
        q = qr if abs(kk - 1) < 1e-3 else cv2.resize(qr, (max(2, int(qw * kk)), max(2, int(qh * kk))),
                                                       interpolation=cv2.INTER_NEAREST)
        _paste(f, q, cx, cy)
        if sv["offline"]:
            put_c(f, self.tr("Sin red WiFi: conecta el robot", "No WiFi: connect the robot"),
                  cx, cy + side // 2 + 22, sc(16), CORAL)
        else:
            put_c(f, sv["hostport"], cx, cy + side // 2 + 22, sc(16), (110, 120, 140))


def _draw_save_overlay(self, f):
    sv = self._sv
    t = time.time() - sv["t0"]
    if t > SAVE_TIMEOUT:
        self._sv = None
        return
    f[:] = (f.astype(np.float32) * 0.35).astype(np.uint8)
    rrect(f, (75, 54), (735, 444), (10, 10, 25), 26)
    rrect(f, (70, 45), (730, 435), CREAM, 26)
    if t < SAVE_ANIM:
        self._draw_saving(f, sv, t)
    else:
        self._draw_saved(f, sv, t - SAVE_ANIM)


_prev_on_down_sv = PaintNode._on_down


def _on_down_sv(self, pos):
    sv = getattr(self, "_sv", None)
    if sv:
        if time.time() - sv["t0"] >= SAVE_ANIM and _in(SV_DONE_R, *pos):
            self._sv = None
        return
    return _prev_on_down_sv(self, pos)


def _wrap_close_save(name):
    prev = getattr(PaintNode, name)

    def w(self, *a, **k):
        self._sv = None
        return prev(self, *a, **k)
    setattr(PaintNode, name, w)


for _n in ("_reset_all", "_go_menu", "_open_library"):
    _wrap_close_save(_n)

PaintNode._draw_menu = _draw_menu_theme
PaintNode._draw_topbar = _draw_topbar_theme
PaintNode._draw_library = _draw_library_theme
PaintNode._draw_toolbar = _draw_toolbar_theme
PaintNode._render = _render_theme
PaintNode._save = _save_theme
PaintNode._on_down = _on_down_sv
PaintNode._draw_saving = _draw_saving
PaintNode._draw_saved = _draw_saved
PaintNode._draw_save_overlay = _draw_save_overlay


# =============================================================================
#  WOOD_BLOCK: estilo madera + pasto (referencias "Sketch Concept" y panel crema)
#  - Menu y biblioteca: fondo de tablones de madera, panel crema con borde naranja,
#    pestana brillante arriba y arbustos en las esquinas
#  - Dibujo/colorear/guiado: fondo de pasto, lienzo con marco de madera,
#    letrero de madera para el mensaje, barra inferior en panel crema
#  - Solo 4 colores (al tocarlos muestran tonos de fuerte a claro) + boton "mas colores"
#  Se inserta ANTES de la seccion MAIN (reemplaza cualquier KUMI_BLOCK anterior).
# =============================================================================

# ── Paleta (se reasignan los nombres globales: el overlay de guardado/QR
#    y las demas pantallas se recolorean solas) ──────────────────────────────
WOOD_BG, WOOD_LN = (252, 212, 140), (244, 194, 118)
CREAM, CREAM_DK, PAPER = (255, 245, 224), (238, 200, 140), (255, 252, 244)
NAVY = (122, 70, 18)                       # texto marron sobre crema
TEAL, MUST, ORNG, CORAL = (40, 180, 160), (255, 214, 40), (246, 124, 30), (240, 90, 80)
GRN, PURP, BLUE, PINKH = (104, 190, 20), (150, 110, 220), (40, 130, 240), (248, 150, 190)
ORG_BR, ORG_DK, TAB_C = (226, 140, 44), (190, 104, 20), (254, 208, 124)
WOOD, WOOD_DK, BROWN = (236, 170, 56), (184, 112, 24), (150, 84, 16)
GRASS, GRASS_FL = (140, 226, 40), (118, 206, 24)
TILE_COLS = [(255, 232, 170), (200, 236, 170), (190, 226, 250), (250, 200, 210),
             (226, 210, 250), (255, 214, 160), (190, 240, 220)]
LEVEL_COLORS = {"bajo": (96, 184, 0), "medio": ORNG, "alto": PURP}

MAIN_IDX = [0, 2, 3, 4]                    # rojo, amarillo, verde, azul
EXTRA_IDX = [i for i in range(len(PALETTE)) if i not in MAIN_IDX]


def _tint(c, a):
    return tuple(int(v + (255 - v) * a) for v in c)


def _shade_list(base):
    """Tonos de fuerte (color base) a claro."""
    if base == (0, 0, 0):
        return [(0, 0, 0), (60, 60, 60), (110, 110, 110), (160, 160, 160), (200, 200, 200), (228, 228, 228)]
    return [base] + [_tint(base, a) for a in (.2, .4, .58, .74, .88)]


# ── Boton gordo ──────────────────────────────────────────────────────────────
def _wood_button(img, rect, label, bg, fg=WHITE, border=None, size=sc(22), dx=0):
    x1, y1, x2, y2 = rect
    if border is not None:                                    # seleccionado: aro amarillo
        rrect(img, (x1 - 4, y1 - 4), (x2 + 4, y2 + 8), MUST, 17)
    rrect(img, (x1, y1 + 4), (x2, y2 + 4), _dark(bg, .68), 14)
    rrect(img, (x1, y1), (x2, y2), bg, 14)
    rrect(img, (x1 + 6, y1 + 4), (x2 - 6, y1 + 10), _tint(bg, .28), 3)
    if label:
        sz = fit_size(label, (x2 - x1) - 14 - 2 * abs(dx), size)
        cx, cy = (x1 + x2) // 2 + dx, (y1 + y2) // 2
        put_c(img, label, cx, cy + 2, sz, _dark(bg, .6))
        put_c(img, label, cx, cy, sz, fg)


button = _wood_button


# ── Piezas del estilo ────────────────────────────────────────────────────────
def _panel(f, r, rad=26):
    x1, y1, x2, y2 = r
    rrect(f, (x1, y1 + 6), (x2, y2 + 6), (222, 166, 92), rad)
    rrect(f, (x1, y1), (x2, y2), CREAM, rad, border=ORG_BR, t=4)


def _tab(f, x1, y1, x2, y2, text, size):
    """Pestana brillante que sobresale del panel."""
    rrect(f, (x1, y1 + 5), (x2, y2 + 5), (222, 166, 92), 26)
    rrect(f, (x1, y1), (x2, y2), TAB_C, 26, border=ORG_BR, t=4)
    rrect(f, (x1 + 16, y1 + 9), (x2 - 16, y1 + 9 + (y2 - y1) // 3), (255, 232, 180), 9)
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    sz = fit_size(text, (x2 - x1) - 50, size)
    put_c(f, text, cx, cy + 3, sz, BROWN)
    put_c(f, text, cx, cy, sz, WHITE)


def _plank(f, x1, y1, x2, y2):
    """Letrero de madera con clavos."""
    rrect(f, (x1, y1 + 5), (x2, y2 + 5), WOOD_DK, 10)
    rrect(f, (x1, y1), (x2, y2), WOOD, 10)
    for gx, gy, gw in ((x1 + 70, y1 + 9, 30), (x2 - 110, y2 - 9, 42), (x1 + 240, y1 + 12, 24)):
        cv2.ellipse(f, (gx, gy), (gw, 2), 0, 0, 360, _dark(WOOD, .9), -1, AA)
    cy = (y1 + y2) // 2
    for nx in (x1 + 12, x2 - 12):
        cv2.circle(f, (nx, cy), 4, BROWN, -1, AA)
        cv2.circle(f, (nx - 1, cy - 1), 1, _tint(BROWN, .5), -1, AA)


def _leaf(bg, x, y, ang, h, w, col):
    a = math.radians(ang)
    tx, ty = x + h * math.sin(a), y - h * math.cos(a)
    pts = np.array([(int(x - w / 2), int(y)), (int(tx), int(ty)), (int(x + w / 2), int(y))], np.int32)
    cv2.fillPoly(bg, [pts], col, AA)
    cv2.circle(bg, (int(tx), int(ty)), max(1, int(w / 3)), col, -1, AA)


def _tuft(bg, x, y, s=1.0, col=(84, 176, 24)):
    for ang, k in ((-28, .8), (0, 1.0), (28, .8)):
        _leaf(bg, x, y, ang, 26 * s * k, 12 * s, col)


def _bush(bg, cx, by, s=1.0):
    for dx, dy, r, c in ((-26, -14, 22, (84, 168, 20)), (0, -26, 26, (84, 168, 20)),
                         (26, -14, 22, (84, 168, 20)), (-10, -8, 20, (116, 198, 30)),
                         (12, -8, 20, (116, 198, 30))):
        cv2.circle(bg, (int(cx + dx * s), int(by + dy * s)), int(r * s), c, -1, AA)
    for dx, dy in ((-24, -26), (-8, -40), (10, -38), (26, -26), (0, -20), (-14, -12), (16, -12)):
        x, y = int(cx + dx * s), int(by + dy * s)
        cv2.line(bg, (x - 5, y + 4), (x + 5, y - 3), (176, 236, 80), 2, AA)


# ── Fondos ───────────────────────────────────────────────────────────────────
def _build_wood():
    bg = np.empty((H, W, 3), np.uint8)
    bg[:] = WOOD_BG
    for y in range(40, H, 52):
        cv2.line(bg, (0, y), (W, y), WOOD_LN, 3)
        cv2.line(bg, (0, y + 3), (W, y + 3), (255, 224, 160), 1)
    rnd = random.Random(5)
    for _ in range(16):
        cv2.ellipse(bg, (rnd.randint(20, W - 20), rnd.randint(20, H - 20)),
                    (rnd.randint(20, 50), 3), 0, 0, 360, (248, 202, 128), -1, AA)
    _bush(bg, 34, H + 2, 1.15)
    _bush(bg, W - 34, H + 2, 1.15)
    return bg


def _build_grass():
    bg = np.empty((H, W, 3), np.uint8)
    bg[:] = GRASS
    rnd = random.Random(8)
    for _ in range(9):
        cv2.ellipse(bg, (rnd.randint(20, W - 20), rnd.randint(20, H - 20)),
                    (rnd.randint(30, 64), rnd.randint(9, 15)), 0, 0, 360, GRASS_FL, -1, AA)
    _tuft(bg, 186, 47, 1.0, (250, 120, 30))
    _tuft(bg, 206, 47, .8, (84, 176, 24))
    return bg


_BG_WOOD, _BG_GRASS = _build_wood(), _build_grass()


# ── Menu ─────────────────────────────────────────────────────────────────────
MODE_RECTS = [(k, R(190, 145 + i * 87, 420, 72)) for i, k in enumerate(("free", "guided", "color"))]
QUIT_R = R(340, 430, 120, 38)


def _draw_menu_w(self, f):
    _panel(f, (60, 62, 740, 418))
    _tab(f, 215, 24, 585, 96, "YAREN PAINT", 44)
    put_c(f, self.tr("¡Dibuja, aprende y diviértete!", "Draw, learn and have fun!"),
          400, 124, 20, NAVY)
    specs = {
        "free": (self.tr("Dibujo libre", "Free draw"), GRN, icon_easel),
        "guided": (self.tr("Dibujar con Yaren", "Draw with Yaren"), BLUE, icon_pencil),
        "color": (self.tr("Colorear", "Color in"), ORNG, icon_palette),
    }
    for key, r in MODE_RECTS:
        label, col, icon = specs[key]
        x1, y1, x2, y2 = r
        cy = (y1 + y2) // 2
        _wood_button(f, r, "", col)
        icon(f, x1 + 52, cy, col)
        sz = fit_size(label, (x2 - x1) - 120, 30)
        put_ml(f, label, x1 + 100, cy + 2, sz, _dark(col, .6))
        put_ml(f, label, x1 + 100, cy, sz, WHITE)
    _wood_button(f, QUIT_R, self.tr("Salir", "Exit"), (214, 84, 64), size=18)


# ── Biblioteca ───────────────────────────────────────────────────────────────
def _draw_library_w(self, f):
    _panel(f, (4, 50, W - 4, 418))
    lv = getattr(self, "_lib_level", "bajo")
    for k, r in TAB_RECTS.items():
        es, en = LEVEL_NAMES[k]
        on = k == lv
        _wood_button(f, r, self.tr(es, en), LEVEL_COLORS[k] if on else (206, 176, 128),
                     border=True if on else None, size=16)
    idxs = _level_idx(lv)
    for j, (x1, y1, x2, y2) in enumerate(_level_cells(lv)):
        col = TILE_COLS[j % len(TILE_COLS)]
        rrect(f, (x1, y1 + 4), (x2, y2 + 4), (222, 166, 92), 16)
        rrect(f, (x1, y1), (x2, y2), col, 16, border=ORG_BR, t=3)
        cx, label_h = (x1 + x2) // 2, sc(34)
        if j < len(idxs):
            d = DRAWINGS[idxs[j]]
            rrect(f, (x1 + 8, y1 + 8), (x2 - 8, y2 - label_h - 2), PAPER, 11)
            vp = viewport((x1 + 12, y1 + 12, x2 - x1 - 24, y2 - y1 - 24 - label_h))
            if not put_lineart(f, d["id"], vp, (30, 30, 30)):
                for sh in d["shapes"]:
                    draw_shape(f, sh, vp, (30, 30, 30), 1)
            label = self._tn(d["label"])
        else:
            rrect(f, (x1 + 8, y1 + 8), (x2 - 8, y2 - label_h - 2), PAPER, 11)
            put_c(f, "?", cx, (y1 + y2 - label_h) // 2, sc(64), ORNG)
            label = self.tr("¡Sorpresa!", "Surprise!")
        put_c(f, label, cx, y2 - label_h // 2 - 1, fit_size(label, x2 - x1 - 16, sc(20)), NAVY)


# ── Barra superior: botones sobre el pasto + letrero de madera ───────────────
def _draw_topbar_w(self, f):
    _wood_button(f, MENU_R, self.tr("< Menú", "< Menu"), BLUE, size=16)
    if self._screen in ("guided", "color"):
        _wood_button(f, OTHER_R, self.tr("Otro dibujo", "Another"), ORNG, size=16)
    x1, y1, x2, y2 = 262, 5, W - 10, TOP_H - 7
    _plank(f, x1, y1, x2, y2)
    msg = self._msg or self.tr("¡Hola! Soy Yaren. ¿Qué quieres hacer hoy?",
                               "Hi! I'm Yaren. What do you want to do today?")
    size = next((sc(s) for s in (30, 26, 22, 18, 15)
                 if tsize(msg, sc(s))[0] <= (x2 - x1) - 48), sc(15))
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    put_c(f, msg, cx, cy + 2, size, BROWN)
    put_c(f, msg, cx, cy, size, WHITE)


# ── Barra de herramientas: 4 colores + "mas colores" ─────────────────────────
def _toolbar_w(self):
    scr, en = self._screen, self.is_english
    y1, y2 = H - BOT_H + 8, H - 14
    x, items = 12, []
    for i in MAIN_IDX:
        items.append(("color", i, (x, y1, x + 32, y2), None))
        x += 38
    items.append(("more", 0, (x, y1, x + 34, y2), None))
    x += 34 + 12
    if scr in ("free", "guided"):
        for i in range(len(SIZES)):
            items.append(("size", i, (x, y1, x + 32, y2), None))
            x += 36
        x += 6
    on = getattr(self, "_show_guide", True)
    txt = []
    if scr == "guided":
        txt += [("skip", "Skip" if en else "Saltar"),
                ("guide", ("With guide" if on else "No guide") if en else ("Con guía" if on else "Sin guía"))]
    txt.append(("eraser", "Eraser" if en else "Borrador"))
    if scr == "free":
        txt.append(("clear", "Clear all" if en else "Borrar todo"))
    if scr in ("guided", "color"):
        txt.append(("restart", "Restart" if en else "Reiniciar"))
    txt.append(("save", "Save" if en else "Guardar"))
    gap = 5
    w = min(118, (W - 12 - x - gap * (len(txt) - 1)) // len(txt))
    for kind, label in txt:
        items.append((kind, 0, (x, y1, x + w, y2), label))
        x += w + gap
    return items


def _swatch_w(f, r, col, sel):
    x1, y1, x2, y2 = r
    cx, cy, rad = (x1 + x2) // 2, (y1 + y2) // 2, min(x2 - x1, y2 - y1) // 2 - 1
    if sel:
        cv2.circle(f, (cx, cy), rad + 4, BROWN, -1, AA)
        cv2.circle(f, (cx, cy), rad + 2, WHITE, -1, AA)
    cv2.circle(f, (cx, cy), rad, (60, 60, 60) if col == (0, 0, 0) else _dark(col, .8), -1, AA)
    cv2.circle(f, (cx, cy), rad - 2, col, -1, AA)
    cv2.circle(f, (cx - rad // 3, cy - rad // 3), max(2, rad // 5), _tint(col, .5), -1, AA)


def _popup_geom(anchor, n):
    """Panel sobre la barra, centrado en el boton 'anchor', con n casillas."""
    cell, gap, pad = 44, 8, 10
    total = n * cell + (n - 1) * gap + 2 * pad
    cxm = (anchor[0] + anchor[2]) // 2
    x0 = max(6, min(W - total - 6, cxm - total // 2))
    y2 = H - BOT_H - 6
    y1 = y2 - cell - 2 * pad
    rects = [(x0 + pad + k * (cell + gap), y1 + pad, x0 + pad + k * (cell + gap) + cell, y1 + pad + cell)
             for k in range(n)]
    return (x0, y1, x0 + total, y2), rects, cxm


def _more_geom(self):
    mr = next(r for k, v, r, l in self._toolbar() if k == "more")
    bg, rects, cxm = _popup_geom(mr, len(EXTRA_IDX))
    return bg, list(zip(EXTRA_IDX, rects)), cxm


def _shade_geom(self):
    so = self._shade_open
    anchor = next(r for k, v, r, l in self._toolbar() if k == "color" and v == so)
    shades = _shade_list(PALETTE[so])
    bg, rects, cxm = _popup_geom(anchor, len(shades))
    return bg, list(zip(shades, rects)), cxm


def _draw_toolbar_w(self, f):
    _panel(f, (4, H - BOT_H + 1, W - 4, H - 9), 18)
    fam = getattr(self, "_fam", 4)
    opened = getattr(self, "_more_open", False)
    for kind, val, r, label in self._toolbar():
        x1, y1, x2, y2 = r
        if kind == "color":
            sel = (not self._eraser) and fam == val
            col = self._color if sel else self.__dict__.setdefault("_fam_shade", {}).get(val, PALETTE[val])
            _swatch_w(f, r, col, sel)
        elif kind == "more":
            cx, cy, rad = (x1 + x2) // 2, (y1 + y2) // 2, 16
            sel = (not self._eraser) and fam in EXTRA_IDX
            if sel or opened:
                cv2.circle(f, (cx, cy), rad + 4, BROWN if sel else MUST, -1, AA)
                cv2.circle(f, (cx, cy), rad + 2, WHITE, -1, AA)
            n = len(EXTRA_IDX)
            for k, idx in enumerate(EXTRA_IDX):
                cv2.ellipse(f, (cx, cy), (rad, rad), 0, k * 360 / n - 90, (k + 1) * 360 / n - 90,
                            PALETTE[idx], -1, AA)
            cv2.circle(f, (cx, cy), 8, self._color if sel else WHITE, -1, AA)
            if not sel:
                cv2.line(f, (cx - 4, cy), (cx + 4, cy), BROWN, 2, AA)
                cv2.line(f, (cx, cy - 4), (cx, cy + 4), BROWN, 2, AA)
        elif kind == "size":
            if self._size_idx == val:
                rrect(f, (x1 - 3, y1 - 3), (x2 + 3, y2 + 3), MUST, 13)
            rrect(f, (x1, y1), (x2, y2), (250, 224, 176), 10)
            cv2.circle(f, ((x1 + x2) // 2, (y1 + y2) // 2), max(2, SIZES[val] // 2 + 1), BROWN, -1, AA)
        elif kind == "eraser":
            _wood_button(f, r, label, PURP, border=True if self._eraser else None, size=16)
        elif kind == "guide":
            on = getattr(self, "_show_guide", True)
            _wood_button(f, r, label, TEAL if on else (170, 150, 120), size=16)
        else:
            col = {"clear": CORAL, "restart": ORNG, "skip": BLUE, "save": GRN}[kind]
            _wood_button(f, r, label, col, size=16)


def _popup_frame(f, bg, cxm):
    _panel(f, bg, 16)
    _poly(f, [(cxm - 9, bg[3] + 5), (cxm + 9, bg[3] + 5), (cxm, bg[3] + 15)], ORG_BR)
    _poly(f, [(cxm - 7, bg[3] + 4), (cxm + 7, bg[3] + 4), (cxm, bg[3] + 11)], CREAM)


def _draw_more_w(self, f):
    bg, cells, cxm = _more_geom(self)
    _popup_frame(f, bg, cxm)
    for idx, r in cells:
        _swatch_w(f, r, PALETTE[idx], (not self._eraser) and tuple(self._color) == PALETTE[idx])


def _draw_shades_w(self, f):
    bg, cells, cxm = _shade_geom(self)
    _popup_frame(f, bg, cxm)
    for col, r in cells:
        _swatch_w(f, r, col, (not self._eraser) and tuple(self._color) == col)


# ── Marco de madera para el lienzo ───────────────────────────────────────────
def _canvas_frame(f):
    x1, y1, x2, y2 = CX, CY, CX + CW, CY + CH
    rrect(f, (x1 - 9, y1 - 4), (x2 + 9, y2 + 13), BROWN, 16)
    rrect(f, (x1 - 9, y1 - 9), (x2 + 9, y2 + 9), (232, 126, 26), 16)
    rrect(f, (x1 - 4, y1 - 4), (x2 + 4, y2 + 4), (176, 96, 18), 8)


def _render_w(self):
    scr = self._screen
    f = (_BG_WOOD if scr in ("menu", "library") else _BG_GRASS).copy()
    if scr == "menu":
        self._draw_menu(f)
    elif scr == "library":
        self._draw_library(f)
    else:
        _canvas_frame(f)
        f[CY:CY + CH, CX:CX + CW] = self._compose_color() if scr == "color" else self._canvas
        if scr == "guided":
            self._draw_guide(f)
        self._draw_toolbar(f)
    if scr != "menu":
        self._draw_topbar(f)
    if scr in ("free", "guided", "color"):
        if getattr(self, "_more_open", False):
            self._draw_more(f)
        elif getattr(self, "_shade_open", None) is not None:
            self._draw_shades(f)
    if getattr(self, "_sv", None):
        self._draw_save_overlay(f)
    return f


# ── Logica: boton "mas colores" ──────────────────────────────────────────────
_prev_click_tool_w = PaintNode._click_tool
_prev_on_down_w = PaintNode._on_down


def _click_tool_w(self, kind, val):
    if kind == "color":                           # abre los tonos de fuerte a claro
        self._fam, self._eraser = val, False
        self._color = self.__dict__.setdefault("_fam_shade", {}).get(val, PALETTE[val])
        self._more_open = False
        self._shade_open = val
        return
    if kind == "more":
        self._shade_open = None
        self._more_open = not getattr(self, "_more_open", False)
        return
    self._more_open = False
    self._shade_open = None
    return _prev_click_tool_w(self, kind, val)


def _on_down_w(self, pos):
    mo = getattr(self, "_more_open", False)
    so = getattr(self, "_shade_open", None)
    if (getattr(self, "_sv", None) or (not mo and so is None)
            or self._screen not in ("free", "guided", "color")):
        return _prev_on_down_w(self, pos)
    x, y = pos
    if mo:
        for idx, r in _more_geom(self)[1]:
            if _in(r, x, y):
                self._color, self._eraser, self._fam = PALETTE[idx], False, idx
                self._more_open = False
                return
        anchor = next(r for k, v, r, l in self._toolbar() if k == "more")
    else:
        for col, r in _shade_geom(self)[1]:
            if _in(r, x, y):
                self._color, self._eraser = col, False
                self.__dict__.setdefault("_fam_shade", {})[so] = col
                self._shade_open = None
                return
        anchor = next(r for k, v, r, l in self._toolbar() if k == "color" and v == so)
    self._more_open = False
    self._shade_open = None
    if _in(anchor, x, y) or _in((CX, CY, CX + CW, CY + CH), x, y):
        return                                   # solo cierra el panel
    return _prev_on_down_w(self, pos)


def _wrap_close_more(name):
    prev = getattr(PaintNode, name)

    def w(self, *a, **k):
        self._more_open = False
        self._shade_open = None
        return prev(self, *a, **k)
    setattr(PaintNode, name, w)


for _n in ("_reset_all", "_go_menu", "_start_mode", "_open_library"):
    _wrap_close_more(_n)

PaintNode._draw_menu = _draw_menu_w
PaintNode._draw_library = _draw_library_w
PaintNode._draw_topbar = _draw_topbar_w
PaintNode._toolbar = _toolbar_w
PaintNode._draw_toolbar = _draw_toolbar_w
PaintNode._draw_more = _draw_more_w
PaintNode._draw_shades = _draw_shades_w
PaintNode._render = _render_w
PaintNode._click_tool = _click_tool_w
PaintNode._on_down = _on_down_w


# =============================================================================
#  CLEAN_BLOCK: botones a la derecha, colores grandes, marco visible, paleta calma
# =============================================================================
BTN, BTN_OFF = (200, 120, 40), (190, 168, 138)
SAVE_C, CLR_C = (80, 170, 40), (214, 84, 64)
GRASS, GRASS_FL = (150, 214, 96), (136, 200, 82)


def _build_grass2():
    bg = np.empty((H, W, 3), np.uint8)
    bg[:] = GRASS
    rnd = random.Random(8)
    for _ in range(9):
        cv2.ellipse(bg, (rnd.randint(20, W - 20), rnd.randint(20, H - 20)),
                    (rnd.randint(30, 64), rnd.randint(9, 15)), 0, 0, 360, GRASS_FL, -1, AA)
    return bg


_BG_GRASS = _build_grass2()


def _canvas_frame(f):
    x1, y1, x2, y2 = CX, CY, CX + CW, CY + CH
    rrect(f, (x1 - 12, y1 - 6), (x2 + 12, y2 + 14), (120, 66, 12), 18)     # sombra
    rrect(f, (x1 - 12, y1 - 12), (x2 + 12, y2 + 12), (214, 118, 24), 18)   # madera
    rrect(f, (x1 - 8, y1 - 8), (x2 + 8, y2 + 8), (246, 168, 70), 14)       # brillo
    rrect(f, (x1 - 3, y1 - 3), (x2 + 3, y2 + 3), (150, 84, 16), 8)         # borde interno


def _toolbar_r(self):
    scr, en = self._screen, self.is_english
    d = 38
    cy = (H - BOT_H + 1 + H - 9) // 2
    y1, y2 = cy - d // 2, cy + d // 2
    x, items = 18, []
    for i in MAIN_IDX:
        items.append(("color", i, (x, y1, x + d, y2), None))
        x += d + 10
    items.append(("more", 0, (x, y1, x + d, y2), None))
    x += d + 16
    if scr in ("free", "guided"):
        for i in range(len(SIZES)):
            items.append(("size", i, (x, y1, x + 30, y2), None))
            x += 34
    on = getattr(self, "_show_guide", True)
    txt = []
    if scr == "guided":
        txt += [("skip", "Skip" if en else "Saltar"),
                ("guide", ("Guide" if on else "No guide") if en else ("Con guía" if on else "Sin guía"))]
    txt.append(("eraser", "Eraser" if en else "Borrador"))
    if scr == "free":
        txt.append(("clear", "Clear all" if en else "Borrar todo"))
    if scr in ("guided", "color"):
        txt.append(("restart", "Restart" if en else "Reiniciar"))
    txt.append(("save", "Save" if en else "Guardar"))
    gap, right = 6, W - 18
    n = len(txt)
    w = min(112, (right - x - 8 - gap * (n - 1)) // n)
    x = right - (n * w + gap * (n - 1))                  # pegados a la derecha
    for kind, label in txt:
        items.append((kind, 0, (x, y1, x + w, y2), label))
        x += w + gap
    return items


def _draw_toolbar_r(self, f):
    _panel(f, (4, H - BOT_H + 1, W - 4, H - 9), 18)
    fam = getattr(self, "_fam", 4)
    opened = getattr(self, "_more_open", False)
    for kind, val, r, label in self._toolbar():
        x1, y1, x2, y2 = r
        if kind == "color":
            sel = (not self._eraser) and fam == val
            col = self._color if sel else self.__dict__.setdefault("_fam_shade", {}).get(val, PALETTE[val])
            _swatch_w(f, r, col, sel)
        elif kind == "more":
            cx, cy, rad = (x1 + x2) // 2, (y1 + y2) // 2, (x2 - x1) // 2 - 1
            sel = (not self._eraser) and fam in EXTRA_IDX
            if sel or opened:
                cv2.circle(f, (cx, cy), rad + 4, BROWN if sel else MUST, -1, AA)
                cv2.circle(f, (cx, cy), rad + 2, WHITE, -1, AA)
            n = len(EXTRA_IDX)
            for k, idx in enumerate(EXTRA_IDX):
                cv2.ellipse(f, (cx, cy), (rad, rad), 0, k * 360 / n - 90, (k + 1) * 360 / n - 90,
                            PALETTE[idx], -1, AA)
            cv2.circle(f, (cx, cy), rad // 2, self._color if sel else WHITE, -1, AA)
            if not sel:
                cv2.line(f, (cx - 4, cy), (cx + 4, cy), BROWN, 2, AA)
                cv2.line(f, (cx, cy - 4), (cx, cy + 4), BROWN, 2, AA)
        elif kind == "size":
            if self._size_idx == val:
                rrect(f, (x1 - 3, y1 - 3), (x2 + 3, y2 + 3), MUST, 13)
            rrect(f, (x1, y1), (x2, y2), (250, 224, 176), 10)
            cv2.circle(f, ((x1 + x2) // 2, (y1 + y2) // 2), max(2, SIZES[val] // 2 + 1), BROWN, -1, AA)
        elif kind == "eraser":
            _wood_button(f, r, label, BTN, border=True if self._eraser else None, size=16)
        elif kind == "guide":
            _wood_button(f, r, label, BTN if getattr(self, "_show_guide", True) else BTN_OFF, size=16)
        else:
            col = {"clear": CLR_C, "restart": BTN, "skip": BTN, "save": SAVE_C}[kind]
            _wood_button(f, r, label, col, size=16)


PaintNode._toolbar = _toolbar_r
PaintNode._draw_toolbar = _draw_toolbar_r


# =============================================================================
#  CLEAN2_BLOCK: menos botones, tipografia uniforme, borrador + "borrar todo",
#                un solo boton de grosor, colores mas separados en colorear
# =============================================================================
DRAW_SCREENS = ("free", "guided", "color")


def _toolbar_c(self):
    scr, en = self._screen, self.is_english
    d = 38
    gap = 30 if scr == "color" else 20
    cy = (H - BOT_H + 1 + H - 9) // 2
    y1, y2 = cy - d // 2, cy + d // 2
    x, items = 18, []
    for i in MAIN_IDX:
        items.append(("color", i, (x, y1, x + d, y2), None))
        x += d + gap
    items.append(("more", 0, (x, y1, x + d, y2), None))
    x += d + gap
    if scr in ("free", "guided"):
        items.append(("size", self._size_idx, (x, y1, x + d, y2), None))
    on = getattr(self, "_show_guide", True)
    txt = []
    if scr == "guided":
        txt.append(("guide", ("Guide" if on else "No guide") if en else ("Con guía" if on else "Sin guía")))
    txt.append(("eraser", "Eraser" if en else "Borrador"))
    txt.append(("save", "Save" if en else "Guardar"))
    w, g = 120, 8
    x = W - 18 - (len(txt) * w + g * (len(txt) - 1))
    for kind, label in txt:
        items.append((kind, 0, (x, y1, x + w, y2), label))
        x += w + g
    return items


def _erase_geom(self):
    er = next((r for k, v, r, l in self._toolbar() if k == "eraser"), None)
    if er is None:
        return None
    pw, ph = 170, 58
    cxm = (er[0] + er[2]) // 2
    x0 = max(6, min(W - pw - 6, cxm - pw // 2))
    y2 = H - BOT_H - 6
    y1 = y2 - ph
    return (x0, y1, x0 + pw, y2), (x0 + 10, y1 + 9, x0 + pw - 10, y2 - 9), cxm, er


def _draw_toolbar_c(self, f):
    _draw_toolbar_r(self, f)
    if getattr(self, "_erase_open", False) and self._screen in DRAW_SCREENS:
        g = _erase_geom(self)
        if g:
            bg, btn, cxm, _ = g
            _popup_frame(f, bg, cxm)
            _wood_button(f, btn, "Clear all" if self.is_english else "Borrar todo", CLR_C, size=16)


_prev_click_tool_c = PaintNode._click_tool
_prev_on_down_c = PaintNode._on_down


def _click_tool_c(self, kind, val):
    if kind == "eraser":
        self._more_open = False
        self._shade_open = None
        if not self._eraser:
            self._eraser, self._erase_open = True, True
        elif not getattr(self, "_erase_open", False):
            self._erase_open = True
        else:
            self._eraser, self._erase_open = False, False
        return
    self._erase_open = False
    if kind == "size":
        self._more_open = False
        self._shade_open = None
        self._size_idx = (self._size_idx + 1) % len(SIZES)
        return
    return _prev_click_tool_c(self, kind, val)


def _on_down_c(self, pos):
    if getattr(self, "_sv", None) or not getattr(self, "_erase_open", False):
        return _prev_on_down_c(self, pos)
    g = _erase_geom(self) if self._screen in DRAW_SCREENS else None
    if g and _in(g[1], *pos):
        self._erase_open = False
        self._click_tool("clear" if self._screen == "free" else "restart", 0)
        return
    self._erase_open = bool(g and _in(g[3], *pos))   # tocar el borrador alterna
    return _prev_on_down_c(self, pos)


def _wrap_close_erase(name):
    prev = getattr(PaintNode, name)

    def w(self, *a, **k):
        self._erase_open = False
        return prev(self, *a, **k)
    setattr(PaintNode, name, w)


for _n in ("_reset_all", "_go_menu", "_start_mode", "_open_library"):
    _wrap_close_erase(_n)

PaintNode._toolbar = _toolbar_c
PaintNode._draw_toolbar = _draw_toolbar_c
PaintNode._click_tool = _click_tool_c
PaintNode._on_down = _on_down_c


# ==== SKETCH_BLOCK_START
# =============================================================================
#  SKETCH_BLOCK: interior estilo "Sketch" (fondo lima, panel crema, deshacer/
#  rehacer, herramientas a la derecha, bandeja amarilla de colores con manchas)
#  Solo cambia las pantallas free / guided / color. Menu y biblioteca igual.
# =============================================================================
import copy

# -- la biblioteca conserva su geometria original
LIB_X, LIB_Y, LIB_W, LIB_H = CX, CY, CW, CH


def _level_cells(lv):
    n = len(_level_idx(lv)) + (1 if lv == "bajo" else 0)
    cols, gap = 4, sc(12)
    rows = max(1, -(-n // cols))
    top = LIB_Y + TAB_H + gap
    avail = LIB_Y + LIB_H - top
    cw = (LIB_W - gap * (cols - 1)) // cols
    ch = min((avail - gap * (rows - 1)) // rows, sc(190))
    return [R(LIB_X + (i % cols) * (cw + gap), top + (i // cols) * (ch + gap), cw, ch)
            for i in range(n)]


# -- nueva geometria del lienzo (el panel crema ES el lienzo)
CX, CY, CW, CH = 26, 62, 652, 336
CANVAS = (CX, CY, CW, CH)
VP = viewport((CX, CY + 40, CW, CH - 40))          # deja libre la fila de botones
VP_LOCAL = (VP[0], VP[1] - CX, VP[2] - CY)

# -- colores principales como en la referencia
PALETTE[0] = (244, 67, 74)
PALETTE[2] = (255, 205, 56)
PALETTE[3] = (30, 214, 134)
PALETTE[4] = (30, 140, 236)

SK_PANEL = (255, 246, 228)
SK_YEL, SK_YEL_LT = (255, 200, 50), (250, 226, 160)
SK_INK, SK_LINE = (92, 40, 48), (78, 20, 32)
SK_GREEN, SK_SHADOW = (136, 224, 54), (92, 178, 34)
_PAPER_F = np.array([256, 247, 229], np.uint16)


def _paper(c):
    """Blanco -> crema (multiplicar), asi el lienzo se funde con el panel."""
    return ((c.astype(np.uint16) * _PAPER_F) >> 8).astype(np.uint8)


def _build_sketch_bg():
    bg = np.empty((H, W, 3), np.uint8)
    bg[:] = SK_GREEN
    for cx, cy, r, c in ((10, 450, 120, (40, 192, 98)), (800, 150, 85, (52, 200, 100)),
                         (790, 470, 130, (160, 230, 70)), (400, 510, 140, (150, 228, 64))):
        cv2.circle(bg, (cx, cy), r, c, -1, AA)
    rnd = random.Random(21)
    for _ in range(14):
        cv2.circle(bg, (rnd.randint(8, W - 8), rnd.randint(8, H - 8)), rnd.randint(3, 6),
                   (176, 238, 96), -1, AA)
    cv2.circle(bg, (748, 32), 24, (255, 214, 30), -1, AA)          # solcito
    for a in range(0, 360, 30):
        cv2.ellipse(bg, (748, 32), (33, 33), 0, a, a + 16, (255, 170, 30), 3, AA)
    return bg


_BG_SKETCH = _build_sketch_bg()


# ── piezas ───────────────────────────────────────────────────────────────────
def _blit_pm(f, pm, a, cx, cy):
    h, w = a.shape[:2]
    x, y = cx - w // 2, cy - h // 2
    x0, y0, x1, y1 = max(0, x), max(0, y), min(W, x + w), min(H, y + h)
    if x1 <= x0 or y1 <= y0:
        return
    aa = a[y0 - y:y1 - y, x0 - x:x1 - x]
    pp = pm[y0 - y:y1 - y, x0 - x:x1 - x]
    roi = f[y0:y1, x0:x1].astype(np.float32)
    f[y0:y1, x0:x1] = np.clip(roi * (1 - aa) + pp, 0, 255).astype(np.uint8)


def _build_tool(kind, N=72):
    S2 = N * 2
    rgb = np.zeros((S2, S2, 3), np.uint8)
    al = np.zeros((S2, S2, 3), np.uint8)
    WH = (255, 255, 255)

    def rr(x1, y1, x2, y2, col, r):
        rrect(rgb, (x1, y1), (x2, y2), col, r)
        rrect(al, (x1, y1), (x2, y2), WH, r)

    def pl(pts, col):
        p = np.array(pts, np.int32)
        cv2.fillPoly(rgb, [p], col, AA)
        cv2.fillPoly(al, [p], WH, AA)

    if kind == "size":                                   # pincel rayado
        ang = -30
        rr(54, 16, 90, 88, (246, 246, 255), 16)
        for k in range(3):
            y = 30 + k * 14
            pl([(54, y + 9), (90, y), (90, y + 8), (54, y + 17)], (70, 70, 230))
        rr(50, 86, 94, 102, (196, 202, 222), 6)
        rr(56, 100, 88, 128, (52, 52, 170), 12)
    elif kind == "roller":                               # brocha roja
        ang = 28
        rr(66, 14, 78, 62, (236, 238, 250), 6)
        rr(44, 60, 100, 82, (250, 250, 255), 8)
        rr(40, 80, 104, 126, (236, 62, 70), 16)
        rr(50, 88, 62, 118, (255, 140, 140), 5)
    elif kind == "pencil":                               # lapiz
        ang = -35
        rr(58, 14, 86, 36, (244, 126, 160), 8)
        rr(58, 34, 86, 42, (200, 206, 222), 2)
        rr(58, 40, 86, 96, (255, 206, 46), 3)
        rr(58, 40, 68, 96, (242, 170, 22), 2)
        pl([(58, 96), (86, 96), (72, 128)], (255, 226, 182))
        pl([(66, 114), (78, 114), (72, 128)], (74, 52, 44))
    else:                                                # borrador
        ang = -20
        rr(22, 44, 122, 100, (246, 186, 214), 18)
        rr(22, 44, 72, 100, (255, 255, 255), 18)
        cv2.rectangle(rgb, (56, 44), (72, 100), (255, 255, 255), -1)
        cv2.rectangle(rgb, (72, 44), (76, 100), (236, 150, 190), -1)
    a = al[..., :1].astype(np.float32) / 255.0
    pm = rgb.astype(np.float32) * a
    M = cv2.getRotationMatrix2D((S2 / 2, S2 / 2), ang, 1.0)
    pm = cv2.warpAffine(pm, M, (S2, S2), flags=cv2.INTER_LINEAR)
    a = cv2.warpAffine(a, M, (S2, S2), flags=cv2.INTER_LINEAR)
    pm = cv2.resize(pm, (N, N), interpolation=cv2.INTER_AREA)
    a = cv2.resize(a, (N, N), interpolation=cv2.INTER_AREA)[..., None]
    return pm, a


_TOOL_CACHE = {}


def _tool_icon(kind):
    if kind not in _TOOL_CACHE:
        _TOOL_CACHE[kind] = _build_tool(kind)
    return _TOOL_CACHE[kind]


_RB = {}


def _rainbow_pm(sz):
    if sz not in _RB:
        r = sz / 2.0
        yy, xx = np.mgrid[0:sz, 0:sz]
        dx, dy = xx + .5 - r, yy + .5 - r
        hsv = np.zeros((sz, sz, 3), np.uint8)
        hsv[..., 0] = ((np.arctan2(dy, dx) + np.pi) / (2 * np.pi) * 179).astype(np.uint8)
        hsv[..., 1], hsv[..., 2] = 175, 255
        rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB).astype(np.float32)
        a = np.clip((r - 1.5 - np.hypot(dx, dy)) / 3.0, 0, 1).astype(np.float32)
        a = cv2.GaussianBlur(a, (0, 0), 1.0)
        _RB[sz] = (rgb * a[..., None], a[..., None])
    return _RB[sz]


def _blob_shape(f, cx, cy, r, col):
    cv2.ellipse(f, (cx, cy), (int(r), int(r * .78)), 0, 0, 360, col, -1, AA)
    for dx, dy, k in ((-.55, .05, .6), (.55, .02, .6), (0, -.42, .52), (0, .44, .52)):
        cv2.circle(f, (int(cx + dx * r), int(cy + dy * r)), int(r * k), col, -1, AA)


def _swatch_blob(f, r, col, sel):
    x1, y1, x2, y2 = r
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    rad = int(min(x2 - x1, y2 - y1) / 2 * .84)
    if sel:
        cy -= 3
        _blob_shape(f, cx, cy, rad + 6, (190, 110, 24))
        _blob_shape(f, cx, cy, rad + 3, WHITE)
        rad += 1
    _blob_shape(f, cx, cy + 3, rad, _dark(col, .78))
    _blob_shape(f, cx, cy, rad, col)
    cv2.ellipse(f, (cx - int(rad * .35), cy - int(rad * .35)),
                (max(2, int(rad * .3)), max(2, int(rad * .16))), -25, 0, 360,
                _tint(col, .45), -1, AA)


_swatch_w = _swatch_blob          # los paneles de tonos / mas colores usan manchas


def _popup_frame(f, bg, cxm):
    x1, y1, x2, y2 = bg
    rrect(f, (x1, y1 + 5), (x2, y2 + 5), (226, 186, 90), 18)
    rrect(f, (x1, y1), (x2, y2), SK_PANEL, 18, border=(255, 208, 70), t=4)
    _poly(f, [(cxm - 10, y2 - 1), (cxm + 10, y2 - 1), (cxm, y2 + 11)], (255, 208, 70))
    _poly(f, [(cxm - 6, y2 - 3), (cxm + 6, y2 - 3), (cxm, y2 + 6)], SK_PANEL)


# ── iconos de la fila superior ───────────────────────────────────────────────
def _sk_circle(f, cx, cy, col):
    cv2.circle(f, (cx, cy + 3), 19, _dark(col, .8), -1, AA)
    cv2.circle(f, (cx, cy), 19, col, -1, AA)


def _ic_back(f, cx, cy, c):
    _limb(f, (cx + 7, cy), (cx - 7, cy), 5, c)
    _limb(f, (cx - 7, cy), (cx, cy - 8), 5, c)
    _limb(f, (cx - 7, cy), (cx, cy + 8), 5, c)


def _ic_arrow(f, cx, cy, col, flip):
    if not flip:
        cv2.ellipse(f, (cx + 1, cy + 4), (11, 11), 0, 180, 380, col, 5, AA)
        cv2.circle(f, (cx + 12, cy + 8), 2, col, -1, AA)
        _poly(f, [(cx - 19, cy + 3), (cx - 3, cy + 3), (cx - 11, cy + 14)], col)
    else:
        cv2.ellipse(f, (cx - 1, cy + 4), (11, 11), 0, 160, 360, col, 5, AA)
        cv2.circle(f, (cx - 12, cy + 8), 2, col, -1, AA)
        _poly(f, [(cx + 3, cy + 3), (cx + 19, cy + 3), (cx + 11, cy + 14)], col)


def _ic_grid(f, cx, cy, c):
    for dx, dy in ((-7, -7), (1, -7), (-7, 1), (1, 1)):
        rrect(f, (cx + dx, cy + dy), (cx + dx + 6, cy + dy + 6), c, 2)


def _ic_save(f, cx, cy, c):
    _limb(f, (cx, cy - 8), (cx, cy + 3), 5, c)
    _limb(f, (cx, cy + 4), (cx - 6, cy - 2), 5, c)
    _limb(f, (cx, cy + 4), (cx + 6, cy - 2), 5, c)
    _limb(f, (cx - 9, cy + 9), (cx + 9, cy + 9), 5, c)


def _ic_eye(f, cx, cy, c, off):
    cv2.ellipse(f, (cx, cy), (11, 7), 0, 0, 360, c, 3, AA)
    cv2.circle(f, (cx, cy), 3, c, -1, AA)
    if off:
        _limb(f, (cx - 10, cy + 9), (cx + 10, cy - 9), 4, c)


# ── barra: lista de elementos clicables ──────────────────────────────────────
def _toolbar_sk(self):
    scr = self._screen
    items = []
    cy, x = CY + 27, CX + 26                       # fila superior (dentro del panel)
    row = ["back", "undo", "redo"]
    if scr in ("guided", "color"):
        row.append("other")
    if scr == "guided":
        row.append("guide")
    row.append("save")
    for k in row:
        items.append((k, 0, (x - 19, cy - 19, x + 19, cy + 19), None))
        x += 46
    tools = (["size"] if scr in ("free", "guided") else []) + ["roller", "pencil", "eraser"]
    pitch = 400 / len(tools)                       # columna derecha
    for i, k in enumerate(tools):
        ty = int(72 + pitch * (i + .5))
        items.append((k, self._size_idx if k == "size" else 0, (710, ty - 42, 786, ty + 42), None))
    d, gap = 48, 30                                # bandeja de colores
    x = 14 + (676 - (5 * d + 4 * gap)) // 2
    for i in MAIN_IDX:
        items.append(("color", i, (x, 419, x + d, 467), None))
        x += d + gap
    items.append(("more", 0, (x, 419, x + d, 467), None))
    return items


def _draw_toolbar_sk(self, f):
    rrect(f, (14, 414), (690, H + 40), (226, 186, 90), 24)
    rrect(f, (14, 410), (690, H + 40), (255, 242, 208), 24, border=(255, 208, 70), t=5)
    fam = getattr(self, "_fam", 4)
    opened = getattr(self, "_more_open", False)
    undo_ok = bool(self.__dict__.get("_undo_st"))
    redo_ok = bool(self.__dict__.get("_redo_st"))
    on = getattr(self, "_show_guide", True)
    for kind, val, r, _ in self._toolbar():
        x1, y1, x2, y2 = r
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        if kind == "color":
            sel = (not self._eraser) and fam == val
            col = self._color if sel else self.__dict__.setdefault("_fam_shade", {}).get(val, PALETTE[val])
            _swatch_blob(f, r, col, sel)
        elif kind == "more":
            sel = (not self._eraser) and fam in EXTRA_IDX
            if sel or opened:
                cv2.circle(f, (cx, cy), 27, (190, 110, 24) if sel else SK_YEL, -1, AA)
                cv2.circle(f, (cx, cy), 25, WHITE, -1, AA)
            pm, a = _rainbow_pm(46)
            _blit_pm(f, pm, a, cx, cy)
            if sel:
                cv2.circle(f, (cx, cy), 10, WHITE, -1, AA)
                cv2.circle(f, (cx, cy), 8, self._color, -1, AA)
            else:
                _limb(f, (cx - 7, cy), (cx + 7, cy), 4, WHITE)
                _limb(f, (cx, cy - 7), (cx, cy + 7), 4, WHITE)
        elif kind in ("undo", "redo"):
            ok = undo_ok if kind == "undo" else redo_ok
            _ic_arrow(f, cx, cy, SK_YEL if ok else SK_YEL_LT, kind == "redo")
        elif kind == "back":
            _sk_circle(f, cx, cy, SK_YEL)
            _ic_back(f, cx, cy, SK_PANEL)
        elif kind == "other":
            _sk_circle(f, cx, cy, SK_YEL)
            _ic_grid(f, cx, cy, SK_PANEL)
        elif kind == "guide":
            _sk_circle(f, cx, cy, SK_YEL if on else (226, 206, 150))
            _ic_eye(f, cx, cy, SK_PANEL, not on)
        elif kind == "save":
            _sk_circle(f, cx, cy, (92, 200, 70))
            _ic_save(f, cx, cy, WHITE)
        else:                                       # size / roller / pencil / eraser
            selected = (kind == "pencil" and not self._eraser) or (kind == "eraser" and self._eraser)
            if selected:
                cv2.circle(f, (cx, cy), 40, (255, 214, 64), -1, AA)
                cv2.circle(f, (cx, cy), 37, (255, 246, 210), -1, AA)
            pm, a = _tool_icon(kind)
            _blit_pm(f, pm, a, cx, cy)
            if kind == "size":
                for i in range(3):
                    c = WHITE if i <= self._size_idx else (112, 196, 44)
                    cv2.circle(f, (cx - 14 + 14 * i, cy + 44), 3 + i, c, -1, AA)


def _sk_msg(self, f):
    x1, y1, x2, y2 = 14, 5, 690, 43
    rrect(f, (x1, y1 + 4), (x2, y2 + 4), SK_SHADOW, 18)
    rrect(f, (x1, y1), (x2, y2), SK_PANEL, 18)
    msg = self._msg or self.tr("¡Hola! Soy Yaren. ¿Qué quieres hacer hoy?",
                               "Hi! I'm Yaren. What do you want to do today?")
    size = next((sc(s) for s in (30, 26, 22, 18, 15)
                 if tsize(msg, sc(s))[0] <= (x2 - x1) - 40), sc(15))
    put_c(f, msg, (x1 + x2) // 2, (y1 + y2) // 2, size, SK_INK)


def _sk_panel(f):
    x1, y1, x2 = CX - 12, CY - 12, CX + CW + 12
    rrect(f, (x1, y1 + 6), (x2, 440), SK_SHADOW, 30)
    rrect(f, (x1, y1), (x2, 436), SK_PANEL, 30)


_prev_render_sk = PaintNode._render


def _render_sk(self):
    scr = self._screen
    if scr not in DRAW_SCREENS:
        return _prev_render_sk(self)
    f = _BG_SKETCH.copy()
    _sk_msg(self, f)
    _sk_panel(f)
    cv = self._compose_color() if scr == "color" else self._canvas
    f[CY:CY + CH, CX:CX + CW] = _paper(cv)
    if scr == "guided":
        self._draw_guide(f)
    self._draw_toolbar(f)
    if getattr(self, "_more_open", False):
        self._draw_more(f)
    elif getattr(self, "_shade_open", None) is not None:
        self._draw_shades(f)
    if getattr(self, "_sv", None):
        self._draw_save_overlay(f)
    return f


# ── colorear: lineas del dibujo en marron oscuro ─────────────────────────────
def _compose_color_sk(self):
    did = DRAWINGS[self._drawing]["id"]
    if _la_mask(did, VP_LOCAL[0]) is None:
        return _orig_compose(self)
    img = np.full((CH, CW, 3), 255, np.uint8)
    for sh, f in zip(self._shapes(), self._fills):
        if is_fillable(sh) and f is not None:
            draw_shape(img, sh, VP_LOCAL, f, -1)
    put_lineart(img, did, VP_LOCAL, SK_LINE)
    return img


# ── deshacer / rehacer ───────────────────────────────────────────────────────
def _snap(self):
    d = self.__dict__
    return {"canvas": self._canvas.copy(), "fills": list(self._fills),
            "grid": {k: list(v) for k, v in self._grid.items()},
            "all": {k: list(v) for k, v in d.get("_all_grid", {}).items()},
            "step": self._step, "pts": list(self._step_pts), "cov": list(self._cov),
            "prog": self._progress, "fin": self._finished,
            "left": list(d.get("_left", [])), "msg": self._msg}


def _restore(self, s):
    self._canvas, self._fills, self._grid = s["canvas"], s["fills"], s["grid"]
    self._all_grid, self._step, self._step_pts = s["all"], s["step"], s["pts"]
    self._cov, self._progress, self._finished = s["cov"], s["prog"], s["fin"]
    self._left, self._msg = s["left"], s["msg"]
    self._stroking = False


def _push_undo(self):
    st = self.__dict__.setdefault("_undo_st", [])
    st.append(_snap(self))
    del st[:-30]
    self._redo_st = []


def _do_undo(self):
    st = self.__dict__.get("_undo_st")
    if not st:
        return
    self.__dict__.setdefault("_redo_st", []).append(_snap(self))
    _restore(self, st.pop())


def _do_redo(self):
    st = self.__dict__.get("_redo_st")
    if not st:
        return
    self.__dict__.setdefault("_undo_st", []).append(_snap(self))
    _restore(self, st.pop())


def _do_clear(self):
    _push_undo(self)
    keep = list(self._undo_st)
    self._click_tool("clear" if self._screen == "free" else "restart", 0)
    self._undo_st, self._redo_st = keep, []


_prev_stamp_sk = PaintNode._stamp
_prev_click_color_sk = PaintNode._click_color


def _stamp_sk(self, pos):
    _push_undo(self)
    return _prev_stamp_sk(self, pos)


def _click_color_sk(self, pos):
    _push_undo(self)
    return _prev_click_color_sk(self, pos)


def _wrap_reset_undo(name):
    prev = getattr(PaintNode, name)

    def w(self, *a, **k):
        r = prev(self, *a, **k)
        self._undo_st, self._redo_st = [], []
        return r
    setattr(PaintNode, name, w)


for _n in ("_reset_all", "_start_mode"):
    _wrap_reset_undo(_n)


# ── acciones de los botones ──────────────────────────────────────────────────
_prev_click_tool_sk = PaintNode._click_tool


def _click_tool_sk(self, kind, val):
    if kind in ("back", "other", "undo", "redo", "roller", "pencil", "eraser"):
        self._more_open = False
        self._shade_open = None
        self._erase_open = False
    if kind == "back":
        self._go_menu()
    elif kind == "other":
        self._open_library(self._screen)
    elif kind == "undo":
        _do_undo(self)
    elif kind == "redo":
        _do_redo(self)
    elif kind == "roller":                 # brocha roja = limpiar / reiniciar (se puede deshacer)
        _do_clear(self)
    elif kind == "pencil":
        self._eraser = False
    elif kind == "eraser":
        self._eraser = not self._eraser
    else:
        return _prev_click_tool_sk(self, kind, val)


_prev_on_down_sk = PaintNode._on_down


def _on_down_sk(self, pos):
    """En las pantallas de dibujo se anulan los rectangulos viejos de Menu/Otro."""
    global MENU_R, OTHER_R
    if self._screen not in DRAW_SCREENS:
        return _prev_on_down_sk(self, pos)
    saved = (MENU_R, OTHER_R)
    MENU_R = OTHER_R = (-9, -9, -8, -8)
    try:
        return _prev_on_down_sk(self, pos)
    finally:
        MENU_R, OTHER_R = saved


PaintNode._toolbar = _toolbar_sk
PaintNode._draw_toolbar = _draw_toolbar_sk
PaintNode._render = _render_sk
PaintNode._compose_color = _compose_color_sk
PaintNode._stamp = _stamp_sk
PaintNode._click_color = _click_color_sk
PaintNode._click_tool = _click_tool_sk
PaintNode._on_down = _on_down_sk
# ==== SKETCH_BLOCK_END


# ==== SKETCH3_BLOCK_START
# =============================================================================
#  SKETCH3_BLOCK: pantalla de dibujo igual a la referencia
#  - fondo lima con solcito, panel crema con el lienzo
#  - atras / deshacer / rehacer arriba a la izquierda
#  - herramientas (pincel, brocha, lapiz, borrador) fuera del panel, a la derecha
#  - bandeja amarilla abajo con manchas grandes + boton arcoiris "+"
# =============================================================================
BOT_H = 100                                 # los paneles emergentes quedan sobre la bandeja
CX, CY, CW, CH = 26, 64, 652, 316
CANVAS = (CX, CY, CW, CH)
VP = viewport((CX, CY + 30, CW, CH - 30))
VP_LOCAL = (VP[0], VP[1] - CX, VP[2] - CY)


def _build_sketch_bg3():
    bg = np.empty((H, W, 3), np.uint8)
    bg[:] = (146, 228, 48)
    for cx, cy, r, c in ((14, 470, 120, (44, 196, 98)), (790, 470, 150, (160, 232, 64)),
                         (760, 110, 80, (124, 216, 42)), (330, 540, 150, (132, 222, 44))):
        cv2.circle(bg, (cx, cy), r, c, -1, AA)
    rrect(bg, (-40, 330), (56, 540), (40, 190, 92), 30)            # hojas verdes abajo-izq
    rnd = random.Random(21)
    for _ in range(12):
        cv2.circle(bg, (rnd.randint(8, W - 8), rnd.randint(8, H - 8)), rnd.randint(3, 6),
                   (176, 238, 96), -1, AA)
    cv2.circle(bg, (40, 28), 26, (255, 214, 30), -1, AA)           # solcito
    for a in range(0, 360, 30):
        cv2.ellipse(bg, (40, 28), (36, 36), 0, a, a + 16, (255, 160, 30), 3, AA)
    rrect(bg, (716, 4), (792, 42), (46, 200, 104), 14)             # tarjetita decorativa
    for i, c in enumerate(((255, 214, 30), (244, 67, 74), (30, 140, 236))):
        cv2.circle(bg, (736 + 20 * i, 23), 6, c, -1, AA)
    return bg


_BG_SKETCH = _build_sketch_bg3()


def _sk_msg3(self, f):
    x1, y1, x2, y2 = 84, 6, 700, 44
    rrect(f, (x1, y1 + 4), (x2, y2 + 4), SK_SHADOW, 19)
    rrect(f, (x1, y1), (x2, y2), SK_PANEL, 19)
    msg = self._msg or self.tr("¡Hola! Soy Yaren. ¿Qué quieres hacer hoy?",
                               "Hi! I'm Yaren. What do you want to do today?")
    size = next((sc(s) for s in (30, 26, 22, 18, 15)
                 if tsize(msg, sc(s))[0] <= (x2 - x1) - 40), sc(15))
    put_c(f, msg, (x1 + x2) // 2, (y1 + y2) // 2, size, SK_INK)


def _sk_panel3(f):
    rrect(f, (14, 58), (690, 426), SK_SHADOW, 36)
    rrect(f, (14, 52), (690, 420), SK_PANEL, 36)


_sk_msg, _sk_panel = _sk_msg3, _sk_panel3

_TOOL_CACHE.clear()


def _tool_icon(kind):
    if kind not in _TOOL_CACHE:
        _TOOL_CACHE[kind] = _build_tool(kind, 76)
    return _TOOL_CACHE[kind]


_RB3 = {}


def _rainbow3(sz):
    if sz not in _RB3:
        r = sz / 2.0
        yy, xx = np.mgrid[0:sz, 0:sz]
        dx, dy = xx + .5 - r, yy + .5 - r
        hsv = np.zeros((sz, sz, 3), np.uint8)
        hsv[..., 0] = ((np.arctan2(dy, dx) + np.pi) / (2 * np.pi) * 179).astype(np.uint8)
        hsv[..., 1], hsv[..., 2] = 150, 255
        rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB).astype(np.float32)
        rgb = cv2.GaussianBlur(rgb, (0, 0), 3.0)                    # arcoiris difuminado
        a = np.clip((r - 2.5 - np.hypot(dx, dy)) / 4.0, 0, 1).astype(np.float32)
        a = cv2.GaussianBlur(a, (0, 0), 2.0)
        _RB3[sz] = (rgb * a[..., None], a[..., None])
    return _RB3[sz]


def _swatch_blob3(f, r, col, sel):
    x1, y1, x2, y2 = r
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    rad = int(min(x2 - x1, y2 - y1) / 2 * .84)
    if sel:
        cy -= 5
        _blob_shape(f, cx, cy, rad + 7, (255, 208, 70))
        _blob_shape(f, cx, cy, rad + 4, WHITE)
    _blob_shape(f, cx, cy + 5, rad, _dark(col, .78))
    _blob_shape(f, cx, cy, rad, col)
    cv2.ellipse(f, (cx - int(rad * .38), cy - int(rad * .36)),
                (max(2, int(rad * .28)), max(2, int(rad * .14))), -25, 0, 360,
                _tint(col, .5), -1, AA)


_swatch_w = _swatch_blob3


def _sk_circle3(f, cx, cy, col, r=22):
    cv2.circle(f, (cx, cy + 3), r, _dark(col, .8), -1, AA)
    cv2.circle(f, (cx, cy), r, col, -1, AA)


def _ic_back3(f, cx, cy, c):
    _limb(f, (cx + 8, cy), (cx - 8, cy), 6, c)
    _limb(f, (cx - 8, cy), (cx, cy - 9), 6, c)
    _limb(f, (cx - 8, cy), (cx, cy + 9), 6, c)


def _ic_arrow3(f, cx, cy, col, flip):
    k = 1.35
    rad, th = int(11 * k), int(6 * k)
    c0 = (int(cx + (-1 if flip else 1) * k), int(cy + 5 * k))
    w9, h13 = int(9 * k), int(13 * k)
    if not flip:
        cv2.ellipse(f, c0, (rad, rad), 0, 180, 380, col, th, AA)
        cv2.circle(f, (c0[0] + int(rad * .94), c0[1] + int(rad * .34)), th // 2, col, -1, AA)
        hx, hy = c0[0] - rad, c0[1]
    else:
        cv2.ellipse(f, c0, (rad, rad), 0, 160, 360, col, th, AA)
        cv2.circle(f, (c0[0] - int(rad * .94), c0[1] + int(rad * .34)), th // 2, col, -1, AA)
        hx, hy = c0[0] + rad, c0[1]
    _poly(f, [(hx - w9, hy - 1), (hx + w9, hy - 1), (hx, hy + h13)], col)


def _toolbar_sk3(self):
    scr = self._screen
    items = []
    cy = CY + 34
    for i, k in enumerate(("back", "undo", "redo")):              # arriba a la izquierda
        x = CX + 38 + 58 * i
        items.append((k, 0, (x - 26, cy - 26, x + 26, cy + 26), None))
    acts = ((["other"] if scr in ("guided", "color") else [])
            + (["guide"] if scr == "guided" else []) + ["save"])
    for i, k in enumerate(reversed(acts)):                        # arriba a la derecha del lienzo
        x = CX + CW - 36 - 56 * i
        items.append((k, 0, (x - 25, cy - 25, x + 25, cy + 25), None))
    tools = (["size"] if scr in ("free", "guided") else []) + ["roller", "pencil", "eraser"]
    for i, k in enumerate(tools):                                 # columna fuera del panel
        ty = 104 + 88 * i
        items.append((k, self._size_idx if k == "size" else 0, (708, ty - 42, 788, ty + 42), None))
    w, gap = 76, 28                                               # bandeja de colores
    x = 62 + (640 - (5 * w + 4 * gap)) // 2
    for i in MAIN_IDX:
        items.append(("color", i, (x, 398, x + w, 466), None))
        x += w + gap
    items.append(("more", 0, (x, 398, x + w, 466), None))
    return items


def _draw_toolbar_sk3(self, f):
    rrect(f, (62, 392), (702, H + 60), (226, 186, 90), 38)
    rrect(f, (62, 384), (702, H + 60), SK_PANEL, 38, border=(255, 208, 70), t=7)
    fam = getattr(self, "_fam", 4)
    opened = getattr(self, "_more_open", False)
    undo_ok = bool(self.__dict__.get("_undo_st"))
    redo_ok = bool(self.__dict__.get("_redo_st"))
    on = getattr(self, "_show_guide", True)
    for kind, val, r, _ in self._toolbar():
        x1, y1, x2, y2 = r
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        if kind == "color":
            sel = (not self._eraser) and fam == val
            col = self._color if sel else self.__dict__.setdefault("_fam_shade", {}).get(val, PALETTE[val])
            _swatch_blob3(f, r, col, sel)
        elif kind == "more":
            sel = (not self._eraser) and fam in EXTRA_IDX
            if sel or opened:
                cv2.circle(f, (cx, cy), 36, (255, 208, 70), -1, AA)
                cv2.circle(f, (cx, cy), 33, WHITE, -1, AA)
            pm, a = _rainbow3(62)
            _blit_pm(f, pm, a, cx, cy)
            if sel:
                cv2.circle(f, (cx, cy), 13, WHITE, -1, AA)
                cv2.circle(f, (cx, cy), 11, self._color, -1, AA)
            else:
                _limb(f, (cx - 10, cy), (cx + 10, cy), 6, WHITE)
                _limb(f, (cx, cy - 10), (cx, cy + 10), 6, WHITE)
        elif kind in ("undo", "redo"):
            ok = undo_ok if kind == "undo" else redo_ok
            _ic_arrow3(f, cx, cy, SK_YEL if ok else SK_YEL_LT, kind == "redo")
        elif kind == "back":
            _sk_circle3(f, cx, cy, SK_YEL)
            _ic_back3(f, cx, cy, SK_PANEL)
        elif kind == "other":
            _sk_circle3(f, cx, cy, SK_YEL)
            _ic_grid(f, cx, cy, SK_PANEL)
        elif kind == "guide":
            _sk_circle3(f, cx, cy, SK_YEL if on else (226, 206, 150))
            _ic_eye(f, cx, cy, SK_PANEL, not on)
        elif kind == "save":
            _sk_circle3(f, cx, cy, (92, 200, 70))
            _ic_save(f, cx, cy, WHITE)
        else:                                       # size / roller / pencil / eraser
            selected = (kind == "pencil" and not self._eraser) or (kind == "eraser" and self._eraser)
            if selected:
                cv2.circle(f, (cx, cy), 42, (255, 214, 64), -1, AA)
                cv2.circle(f, (cx, cy), 39, (255, 246, 210), -1, AA)
            pm, a = _tool_icon(kind)
            _blit_pm(f, pm, a, cx, cy)
            if kind == "size":
                for i in range(3):
                    c = WHITE if i <= self._size_idx else (112, 196, 44)
                    cv2.circle(f, (cx - 14 + 14 * i, cy + 46), 3 + i, c, -1, AA)


PaintNode._toolbar = _toolbar_sk3
PaintNode._draw_toolbar = _draw_toolbar_sk3
# ==== SKETCH3_BLOCK_END


# ==== SKETCH4_BLOCK_START
# =============================================================================
#  SKETCH4_BLOCK: lienzo mas grande, solo boton "atras", menu de 3 puntos,
#  y los colores extra reemplazan a uno de los 4 principales (con sus tonos)
# =============================================================================
BOT_H = 62
CX, CY, CW, CH = 14, 56, 692, 354
CANVAS = (CX, CY, CW, CH)
VP = viewport((CX, CY, CW, CH))
VP_LOCAL = (VP[0], VP[1] - CX, VP[2] - CY)


def _build_sketch_bg4():
    bg = np.empty((H, W, 3), np.uint8)
    bg[:] = (146, 228, 48)
    for cx, cy, r, c in ((14, 470, 120, (44, 196, 98)), (790, 470, 150, (160, 232, 64)),
                         (760, 110, 80, (124, 216, 42)), (330, 540, 150, (132, 222, 44))):
        cv2.circle(bg, (cx, cy), r, c, -1, AA)
    rnd = random.Random(21)
    for _ in range(12):
        cv2.circle(bg, (rnd.randint(8, W - 8), rnd.randint(8, H - 8)), rnd.randint(3, 6),
                   (176, 238, 96), -1, AA)
    return bg


_BG_SKETCH = _build_sketch_bg4()


def _sk_msg4(self, f):
    x1, y1, x2, y2 = 70, 6, 712, 44
    rrect(f, (x1, y1 + 4), (x2, y2 + 4), SK_SHADOW, 19)
    rrect(f, (x1, y1), (x2, y2), SK_PANEL, 19)
    msg = self._msg or self.tr("¡Hola! Soy Yaren. ¿Qué quieres hacer hoy?",
                               "Hi! I'm Yaren. What do you want to do today?")
    size = next((sc(s) for s in (30, 26, 22, 18, 15)
                 if tsize(msg, sc(s))[0] <= (x2 - x1) - 40), sc(15))
    put_c(f, msg, (x1 + x2) // 2, (y1 + y2) // 2, size, SK_INK)


def _sk_panel4(f):
    rrect(f, (4, 54), (716, 424), SK_SHADOW, 26)
    rrect(f, (4, 48), (716, 418), SK_PANEL, 26)


_sk_msg, _sk_panel = _sk_msg4, _sk_panel4

DOTS_X, DOTS_Y = 757, 26


def _dots_rect(self):
    n = {"free": 1, "guided": 3, "color": 2}.get(self._screen, 1)
    if n < 2:
        return None
    return (722, 54, 792, 82 + 56 * (n - 1) + 34)


def _toolbar_n(self):
    scr = self._screen
    items = [("back", 0, (11, 1, 61, 51), None)]
    acts = ((["other"] if scr in ("guided", "color") else [])
            + (["guide"] if scr == "guided" else []) + ["save"])
    drect = (DOTS_X - 25, DOTS_Y - 25, DOTS_X + 25, DOTS_Y + 25)
    if len(acts) == 1:
        items.append((acts[0], 0, drect, None))
    else:
        if getattr(self, "_dots_open", False):
            for i, k in enumerate(acts):
                cy = 82 + 56 * i
                items.append((k, 0, (DOTS_X - 27, cy - 26, DOTS_X + 27, cy + 26), None))
        items.append(("dots", 0, drect, None))
    tools = (["size"] if scr in ("free", "guided") else []) + ["roller", "pencil", "eraser"]
    n = len(tools)
    for i, k in enumerate(tools):
        ty = int(252 + 100 * (i - (n - 1) / 2))
        items.append((k, self._size_idx if k == "size" else 0, (721, ty - 44, 795, ty + 44), None))
    w, gap = 62, 24
    x = 360 - (5 * w + 4 * gap) // 2
    for i in MAIN_IDX:
        items.append(("color", i, (x, 430, x + w, 478), None))
        x += w + gap
    items.append(("more", 0, (x, 430, x + w, 478), None))
    return items


def _draw_toolbar_n(self, f):
    rrect(f, (127, 420), (593, H + 60), (226, 186, 90), 34)
    rrect(f, (127, 414), (593, H + 60), SK_PANEL, 34, border=(255, 208, 70), t=6)
    fam = getattr(self, "_fam", 4)
    opened = getattr(self, "_more_open", False)
    on = getattr(self, "_show_guide", True)
    dg = _dots_rect(self) if getattr(self, "_dots_open", False) else None

    def one(kind, val, r):
        x1, y1, x2, y2 = r
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        if kind == "color":
            sel = (not self._eraser) and fam == val
            col = self._color if sel else self.__dict__.setdefault("_fam_shade", {}).get(val, PALETTE[val])
            _swatch_blob3(f, r, col, sel)
        elif kind == "more":
            sel = (not self._eraser) and fam in EXTRA_IDX
            if sel or opened:
                cv2.circle(f, (cx, cy), 36, (255, 208, 70), -1, AA)
                cv2.circle(f, (cx, cy), 33, WHITE, -1, AA)
            pm, a = _rainbow3(62)
            _blit_pm(f, pm, a, cx, cy)
            if sel:
                cv2.circle(f, (cx, cy), 13, WHITE, -1, AA)
                cv2.circle(f, (cx, cy), 11, self._color, -1, AA)
            else:
                _limb(f, (cx - 10, cy), (cx + 10, cy), 6, WHITE)
                _limb(f, (cx, cy - 10), (cx, cy + 10), 6, WHITE)
        elif kind == "back":
            _sk_circle3(f, cx, cy, SK_YEL)
            _ic_back3(f, cx, cy, SK_PANEL)
        elif kind == "dots":
            if getattr(self, "_dots_open", False):
                cv2.circle(f, (cx, cy), 27, WHITE, -1, AA)
            _sk_circle3(f, cx, cy, SK_YEL)
            for dy in (-9, 0, 9):
                cv2.circle(f, (cx, cy + dy), 4, SK_PANEL, -1, AA)
        elif kind == "other":
            _sk_circle3(f, cx, cy, SK_YEL)
            _ic_grid(f, cx, cy, SK_PANEL)
        elif kind == "guide":
            _sk_circle3(f, cx, cy, SK_YEL if on else (226, 206, 150))
            _ic_eye(f, cx, cy, SK_PANEL, not on)
        elif kind == "save":
            _sk_circle3(f, cx, cy, (92, 200, 70))
            _ic_save(f, cx, cy, WHITE)
        else:
            selected = (kind == "pencil" and not self._eraser) or (kind == "eraser" and self._eraser)
            if selected:
                cv2.circle(f, (cx, cy), 42, (255, 214, 64), -1, AA)
                cv2.circle(f, (cx, cy), 39, (255, 246, 210), -1, AA)
            pm, a = _tool_icon(kind)
            _blit_pm(f, pm, a, cx, cy)
            if kind == "size":
                for i in range(3):
                    c = WHITE if i <= self._size_idx else (112, 196, 44)
                    cv2.circle(f, (cx - 14 + 14 * i, cy + 46), 3 + i, c, -1, AA)

    items = self._toolbar()
    pop = [it for it in items if dg and it[0] in ("other", "guide", "save")]
    for kind, val, r, _l in items:
        if (kind, val, r, _l) not in pop:
            one(kind, val, r)
    if dg:
        rrect(f, (dg[0], dg[1] + 5), (dg[2], dg[3] + 5), (226, 186, 90), 20)
        rrect(f, (dg[0], dg[1]), (dg[2], dg[3]), SK_PANEL, 20, border=(255, 208, 70), t=4)
        for kind, val, r, _l in pop:
            one(kind, val, r)


# ── colores extra: reemplazan a un color principal (el menos usado) ──────────
def _swap_main(self, idx):
    lru = self.__dict__.setdefault("_lru", list(MAIN_IDX))
    lru[:] = [i for i in lru if i in MAIN_IDX] + [i for i in MAIN_IDX if i not in lru]
    fs = self.__dict__.setdefault("_fam_shade", {})
    if idx not in MAIN_IDX:
        out = lru.pop(0)
        MAIN_IDX[MAIN_IDX.index(out)] = idx
        EXTRA_IDX[:] = [i for i in range(len(PALETTE)) if i not in MAIN_IDX]
        fs.pop(out, None)
    if idx in lru:
        lru.remove(idx)
    lru.append(idx)
    self._fam, self._eraser = idx, False
    self._color = fs.get(idx, PALETTE[idx])
    self._more_open = False
    self._shade_open = idx          # abre sus tonalidades de una vez


_prev_click_tool_n = PaintNode._click_tool
_prev_on_down_n = PaintNode._on_down
_prev_reset_all_n = PaintNode._reset_all


def _click_tool_n(self, kind, val):
    if kind == "dots":
        self._more_open = False
        self._shade_open = None
        self._erase_open = False
        self._dots_open = not getattr(self, "_dots_open", False)
        return
    self._dots_open = False
    if kind == "color":
        lru = self.__dict__.setdefault("_lru", list(MAIN_IDX))
        if val in lru:
            lru.remove(val)
        lru.append(val)
    return _prev_click_tool_n(self, kind, val)


def _on_down_n(self, pos):
    if getattr(self, "_sv", None) or self._screen not in DRAW_SCREENS:
        return _prev_on_down_n(self, pos)
    x, y = pos
    if getattr(self, "_more_open", False):
        for idx, r in _more_geom(self)[1]:
            if _in(r, x, y):
                _swap_main(self, idx)
                return
    if getattr(self, "_dots_open", False):
        if any(_in(r, x, y) for k, v, r, l in self._toolbar()):
            return _prev_on_down_n(self, pos)
        self._dots_open = False
        g = _dots_rect(self)
        if (g and _in(g, x, y)) or _in((CX, CY, CX + CW, CY + CH), x, y):
            return
    return _prev_on_down_n(self, pos)


def _reset_all_n(self, *a, **k):
    r = _prev_reset_all_n(self, *a, **k)
    MAIN_IDX[:] = [0, 2, 3, 4]
    EXTRA_IDX[:] = [i for i in range(len(PALETTE)) if i not in MAIN_IDX]
    self._lru = list(MAIN_IDX)
    self._fam, self._fam_shade = 4, {}
    self._color = PALETTE[4]
    self._dots_open = False
    return r


def _wrap_close_dots(name):
    prev = getattr(PaintNode, name)

    def w(self, *a, **k):
        self._dots_open = False
        return prev(self, *a, **k)
    setattr(PaintNode, name, w)


for _n in ("_go_menu", "_start_mode", "_open_library"):
    _wrap_close_dots(_n)

PaintNode._reset_all = _reset_all_n
PaintNode._toolbar = _toolbar_n
PaintNode._draw_toolbar = _draw_toolbar_n
PaintNode._click_tool = _click_tool_n
PaintNode._on_down = _on_down_n
# ==== SKETCH4_BLOCK_END


# ==== SKETCH5_BLOCK_START
# =============================================================================
#  SKETCH5_BLOCK: sin boton de grosor (lapiz y borrador ciclan 3 tamanos),
#  boton de guia abajo-izquierda dentro del lienzo, flecha -> menu de dibujos,
#  solo boton de descarga arriba a la derecha, colores con buen contraste
# =============================================================================
ERASE_SIZES = [14, 30, 56]
PaintNode._erase_idx = 0


def _toolbar_5(self):
    scr = self._screen
    items = [("back", 0, (11, 1, 61, 51), None),
             ("save", 0, (732, 1, 782, 51), None)]
    if scr == "guided":
        items.append(("guide", 0, (24, 360, 160, 402), None))
    tools = ["roller", "pencil", "eraser"]
    for i, k in enumerate(tools):
        ty = 152 + 100 * i
        items.append((k, 0, (717, ty - 44, 797, ty + 44), None))
    w, gap = 62, 24
    x = 360 - (5 * w + 4 * gap) // 2
    for i in MAIN_IDX:
        items.append(("color", i, (x, 430, x + w, 478), None))
        x += w + gap
    items.append(("more", 0, (x, 430, x + w, 478), None))
    return items


def _ring_btn(f, cx, cy, col, r=22):
    cv2.circle(f, (cx, cy + 5), r + 4, (40, 100, 30), -1, AA)    # sombra
    cv2.circle(f, (cx, cy), r + 4, WHITE, -1, AA)                # aro blanco
    _sk_circle3(f, cx, cy, col, r)


def _draw_toolbar_5(self, f):
    rrect(f, (127, 420), (593, H + 60), (226, 186, 90), 34)
    rrect(f, (127, 414), (593, H + 60), SK_PANEL, 34, border=(255, 208, 70), t=6)
    fam = getattr(self, "_fam", 4)
    opened = getattr(self, "_more_open", False)
    on = getattr(self, "_show_guide", True)
    en = self.is_english
    scr = self._screen
    for kind, val, r, _l in self._toolbar():
        x1, y1, x2, y2 = r
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        if kind == "color":
            sel = (not self._eraser) and fam == val
            col = self._color if sel else self.__dict__.setdefault("_fam_shade", {}).get(val, PALETTE[val])
            _swatch_blob3(f, r, col, sel)
        elif kind == "more":
            sel = (not self._eraser) and fam in EXTRA_IDX
            if sel or opened:
                cv2.circle(f, (cx, cy), 36, (255, 208, 70), -1, AA)
                cv2.circle(f, (cx, cy), 33, WHITE, -1, AA)
            pm, a = _rainbow3(62)
            _blit_pm(f, pm, a, cx, cy)
            if sel:
                cv2.circle(f, (cx, cy), 13, WHITE, -1, AA)
                cv2.circle(f, (cx, cy), 11, self._color, -1, AA)
            else:
                _limb(f, (cx - 10, cy), (cx + 10, cy), 6, WHITE)
                _limb(f, (cx, cy - 10), (cx, cy + 10), 6, WHITE)
        elif kind == "back":
            _ring_btn(f, cx, cy, (240, 140, 30))
            _ic_back3(f, cx, cy, WHITE)
        elif kind == "save":
            _ring_btn(f, cx, cy, (30, 130, 230))
            _ic_save(f, cx, cy, WHITE)
        elif kind == "guide":
            lab = ("Guide" if on else "No guide") if en else ("Con guía" if on else "Sin guía")
            _wood_button(f, r, lab, (214, 104, 24) if on else (120, 100, 86), size=18, dx=14)
            _ic_eye(f, x1 + 24, cy, WHITE, not on)
        else:                                       # roller / pencil / eraser
            selected = (kind == "pencil" and not self._eraser) or (kind == "eraser" and self._eraser)
            cv2.circle(f, (cx, cy + 4), 40, (40, 100, 30), -1, AA)
            if selected:
                cv2.circle(f, (cx, cy), 41, (255, 196, 40), -1, AA)
            cv2.circle(f, (cx, cy), 39 if not selected else 38, WHITE, -1, AA)
            cv2.circle(f, (cx, cy), 36, (255, 246, 226), -1, AA)
            pm, a = _tool_icon(kind)
            _blit_pm(f, pm, a, cx, cy)
            if kind in ("pencil", "eraser") and scr in ("free", "guided"):
                idx = self._size_idx if kind == "pencil" else self._erase_idx
                for i in range(3):
                    c = WHITE if i <= idx else (40, 110, 30)
                    cv2.circle(f, (cx - 14 + 14 * i, cy + 49), 4 + i, (30, 90, 20), -1, AA)
                    cv2.circle(f, (cx - 14 + 14 * i, cy + 49), 3 + i, c, -1, AA)


def _brush_5(self):
    if self._eraser:
        return (WHITE, ERASE_SIZES[self._erase_idx])
    return (self._color, SIZES[self._size_idx])


_prev_click_tool_5 = PaintNode._click_tool


def _click_tool_5(self, kind, val):
    if kind in ("back", "pencil", "eraser"):
        self._more_open = False
        self._shade_open = None
        self._erase_open = False
        self._dots_open = False
    if kind == "back":
        if self._screen in ("guided", "color"):
            self._open_library(self._screen)       # menu de dibujos
        else:
            self._go_menu()
        return
    if kind == "pencil":
        if self._eraser:
            self._eraser = False
        elif self._screen in ("free", "guided"):
            self._size_idx = (self._size_idx + 1) % len(SIZES)
        return
    if kind == "eraser":
        if not self._eraser:
            self._eraser = True
        elif self._screen in ("free", "guided"):
            self._erase_idx = (self._erase_idx + 1) % len(ERASE_SIZES)
        return
    return _prev_click_tool_5(self, kind, val)


PaintNode._toolbar = _toolbar_5
PaintNode._draw_toolbar = _draw_toolbar_5
PaintNode._brush = _brush_5
PaintNode._click_tool = _click_tool_5
# ==== SKETCH5_BLOCK_END


# ==== SKETCH6_BLOCK_START
# =============================================================================
#  SKETCH6_BLOCK: herramientas sin circulos (solo resalta la seleccionada),
#  mas espacio entre el ojo y el texto de la guia, descarga del color de atras
# =============================================================================
BACK_C = (240, 140, 30)


def _toolbar_6(self):
    items = _toolbar_5(self)
    out = []
    for kind, val, r, lab in items:
        if kind == "guide":
            r = (24, 360, 190, 402)
        out.append((kind, val, r, lab))
    return out


def _draw_toolbar_6(self, f):
    rrect(f, (127, 420), (593, H + 60), (226, 186, 90), 34)
    rrect(f, (127, 414), (593, H + 60), SK_PANEL, 34, border=(255, 208, 70), t=6)
    fam = getattr(self, "_fam", 4)
    opened = getattr(self, "_more_open", False)
    on = getattr(self, "_show_guide", True)
    en = self.is_english
    scr = self._screen
    for kind, val, r, _l in self._toolbar():
        x1, y1, x2, y2 = r
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        if kind == "color":
            sel = (not self._eraser) and fam == val
            col = self._color if sel else self.__dict__.setdefault("_fam_shade", {}).get(val, PALETTE[val])
            _swatch_blob3(f, r, col, sel)
        elif kind == "more":
            sel = (not self._eraser) and fam in EXTRA_IDX
            if sel or opened:
                cv2.circle(f, (cx, cy), 36, (255, 208, 70), -1, AA)
                cv2.circle(f, (cx, cy), 33, WHITE, -1, AA)
            pm, a = _rainbow3(62)
            _blit_pm(f, pm, a, cx, cy)
            if sel:
                cv2.circle(f, (cx, cy), 13, WHITE, -1, AA)
                cv2.circle(f, (cx, cy), 11, self._color, -1, AA)
            else:
                _limb(f, (cx - 10, cy), (cx + 10, cy), 6, WHITE)
                _limb(f, (cx, cy - 10), (cx, cy + 10), 6, WHITE)
        elif kind == "back":
            _ring_btn(f, cx, cy, BACK_C)
            _ic_back3(f, cx, cy, WHITE)
        elif kind == "save":
            _ring_btn(f, cx, cy, BACK_C)          # mismo color que la flecha
            _ic_save(f, cx, cy, WHITE)
        elif kind == "guide":
            lab = ("Guide" if on else "No guide") if en else ("Con guía" if on else "Sin guía")
            _wood_button(f, r, lab, (214, 104, 24) if on else (120, 100, 86), size=18, dx=24)
            _ic_eye(f, x1 + 28, cy, WHITE, not on)
        else:                                       # roller / pencil / eraser
            selected = (kind == "pencil" and not self._eraser) or (kind == "eraser" and self._eraser)
            if selected:
                cv2.circle(f, (cx, cy), 42, (255, 214, 64), -1, AA)
                cv2.circle(f, (cx, cy), 39, (255, 246, 210), -1, AA)
            pm, a = _tool_icon(kind)
            _blit_pm(f, pm, a, cx, cy)
            if kind in ("pencil", "eraser") and scr in ("free", "guided"):
                idx = self._size_idx if kind == "pencil" else self._erase_idx
                for i in range(3):
                    c = WHITE if i <= idx else (40, 110, 30)
                    cv2.circle(f, (cx - 14 + 14 * i, cy + 49), 4 + i, (30, 90, 20), -1, AA)
                    cv2.circle(f, (cx - 14 + 14 * i, cy + 49), 3 + i, c, -1, AA)


PaintNode._toolbar = _toolbar_6
PaintNode._draw_toolbar = _draw_toolbar_6
# ==== SKETCH6_BLOCK_END




# ==== SKETCH8_BLOCK_START
# =============================================================================
#  SKETCH8_BLOCK: menu principal igual a la referencia "Sketch Concept":
#  pasto con borde de nubes arriba, marco grande de madera naranja con huecos,
#  letrero de madera que sobresale, tablones inclinados, matas, camino y
#  circulo inferior (boton Salir). Los 3 botones van dentro del papel crema.
# =============================================================================
MODE_RECTS = [(k, R(170, 129 + i * 76, 450, 64)) for i, k in enumerate(("free", "guided", "color"))]
QUIT_R = R(364, 408, 72, 72)

_G8 = (146, 230, 44)


def _rot_plank8(bg, cx, cy, w, h, ang):
    a = math.radians(ang)
    ca, sa = math.cos(a), math.sin(a)
    pts = [(cx + dx * ca - dy * sa, cy + dx * sa + dy * ca)
           for dx, dy in ((-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2))]
    _poly(bg, [(x, y + 6) for x, y in pts], (176, 96, 16))
    _poly(bg, pts, (242, 166, 52))
    hl = [(cx + dx * ca - dy * sa, cy + dx * sa + dy * ca)
          for dx, dy in ((-w / 2 + 4, -h / 2 + 4), (w / 2 - 4, -h / 2 + 4),
                         (w / 2 - 4, -h / 2 + 10), (-w / 2 + 4, -h / 2 + 10))]
    _poly(bg, hl, (255, 205, 110))
    cv2.line(bg, (int(cx - w * .25 * ca), int(cy - w * .25 * sa + 3)),
             (int(cx + w * .2 * ca), int(cy + w * .2 * sa + 3)), (214, 136, 36), 2, AA)
    nx, ny = cx - (w / 2 - 12) * ca, cy - (w / 2 - 12) * sa
    cv2.circle(bg, (int(nx), int(ny)), 4, (176, 96, 16), -1, AA)


def _build_menu8():
    bg = np.empty((H, W, 3), np.uint8)
    bg[:] = (238, 244, 250)
    bg[26:] = _G8
    for x in range(-10, W + 70, 64):                      # borde de nubes arriba
        cv2.circle(bg, (x, -6), 32, _G8, -1, AA)
    bg[26:] = _G8
    rnd = random.Random(12)
    for _ in range(9):                                     # manchas de pasto
        cv2.ellipse(bg, (rnd.randint(30, W - 30), rnd.randint(60, H - 30)),
                    (rnd.randint(26, 52), rnd.randint(8, 13)), 0, 0, 360, (128, 214, 38), -1, AA)
    cv2.ellipse(bg, (395, 408), (330, 26), 0, 0, 360, (112, 202, 34), -1, AA)    # sombra del marco
    cv2.rectangle(bg, (360, 410), (440, H), (204, 238, 110), -1)                  # camino
    cv2.circle(bg, (-6, 282), 32, (36, 170, 96), -1, AA)                          # arbusto izq
    rrect(bg, (4, 266), (11, 296), (255, 214, 40), 3)
    cv2.circle(bg, (804, 240), 30, (36, 170, 96), -1, AA)                         # arbusto der
    cv2.rectangle(bg, (222, 396), (238, 410), (255, 226, 120), -1)                # cubito amarillo
    _rot_plank8(bg, 22, 72, 100, 40, 30)                                          # tablones
    _rot_plank8(bg, 772, 292, 100, 40, -26)
    _rot_plank8(bg, 24, 380, 96, 38, -14)
    ORN, GRN8 = (250, 118, 28), (84, 176, 24)
    for x, y, s, c in ((772, 100, 1.2, ORN), (746, 104, 1.0, ORN), (700, 52, .9, GRN8),
                       (27, 140, .8, GRN8), (14, 146, .6, GRN8), (36, 148, .7, ORN),
                       (104, 440, 1.1, ORN), (86, 444, .8, GRN8),
                       (676, 436, 1.0, GRN8), (696, 440, .8, GRN8)):
        _tuft(bg, x, y, s, c)
    return bg


_BG_MENU8 = _build_menu8()


def _draw_menu_8(self, f):
    # marco de madera naranja
    rrect(f, (85, 82), (705, 404), (150, 76, 12), 48)
    rrect(f, (85, 75), (705, 397), (222, 122, 26), 48)
    rrect(f, (93, 83), (697, 385), (240, 148, 40), 42)
    rrect(f, (118, 108), (672, 366), (176, 92, 16), 30)
    rrect(f, (126, 116), (664, 358), (255, 250, 238), 16)
    for hx, hy in ((104, 94), (686, 94), (104, 380), (686, 380)):
        cv2.ellipse(f, (hx, hy + 2), (12, 8), 0, 0, 360, (240, 160, 60), -1, AA)
        cv2.ellipse(f, (hx, hy), (12, 8), 0, 0, 360, (150, 76, 12), -1, AA)
    # letrero de madera
    x1, y1, x2, y2 = 262, 40, 534, 108
    rrect(f, (x1, y1 + 7), (x2, y2 + 7), (170, 90, 14), 20)
    rrect(f, (x1, y1), (x2, y2), (242, 164, 52), 20, border=(206, 120, 24), t=4)
    rrect(f, (x1 + 14, y1 + 8), (x2 - 14, y1 + 18), (255, 206, 120), 5)
    for nx in (x1 + 16, x2 - 16):
        cv2.circle(f, (nx, (y1 + y2) // 2 + 4), 4, (176, 96, 16), -1, AA)
    sz = fit_size("YAREN PAINT", (x2 - x1) - 70, 34)
    put_c(f, "YAREN PAINT", 398, 79, sz, (170, 90, 14))
    put_c(f, "YAREN PAINT", 398, 76, sz, WHITE)
    # botones dentro del papel
    specs = {
        "free": (self.tr("Dibujo libre", "Free draw"), GRN, icon_easel),
        "guided": (self.tr("Dibujar con Yaren", "Draw with Yaren"), BLUE, icon_pencil),
        "color": (self.tr("Colorear", "Color in"), ORNG, icon_palette),
    }
    for key, r in MODE_RECTS:
        label, col, icon = specs[key]
        bx1, by1, bx2, by2 = r
        cy = (by1 + by2) // 2
        _wood_button(f, r, "", col)
        icon(f, bx1 + 52, cy, col)
        s2 = fit_size(label, (bx2 - bx1) - 120, 30)
        put_ml(f, label, bx1 + 100, cy + 2, s2, _dark(col, .6))
        put_ml(f, label, bx1 + 100, cy, s2, WHITE)
    # circulo inferior = Salir
    cx, cy = 400, 444
    cv2.circle(f, (cx, cy + 4), 36, (120, 150, 60), -1, AA)
    cv2.circle(f, (cx, cy), 36, WHITE, -1, AA)
    cv2.circle(f, (cx, cy), 30, (150, 214, 246), -1, AA)
    cv2.circle(f, (cx, cy), 25, WHITE, -1, AA)
    put_c(f, self.tr("Salir", "Exit"), cx, cy - 6, fit_size(self.tr("Salir", "Exit"), 40, 15), (40, 120, 220))


_prev_render_8 = PaintNode._render


def _render_8(self):
    if self._screen != "menu":
        return _prev_render_8(self)
    f = _BG_MENU8.copy()
    self._draw_menu(f)
    if getattr(self, "_sv", None):
        self._draw_save_overlay(f)
    return f


PaintNode._draw_menu = _draw_menu_8
PaintNode._render = _render_8
# ==== SKETCH8_BLOCK_END




# ==== SKETCH_FINAL_BLOCK_START
# Menu identico a la referencia "Sketch Concept" (solo cambia el interior)
MODE_RECTS = [(k, R(154 + i * 168, 140, 156, 210)) for i, k in enumerate(("free", "guided", "color"))]
QUIT_R = R(690, 436, 92, 34)


def _build_menu_final():
    G = (146, 230, 44)
    bg = np.empty((H, W, 3), np.uint8)
    bg[:] = G
    # borde superior claro con nubes de pasto colgando
    bg[:30] = (236, 244, 250)
    for x in range(-10, W + 50, 40):
        cv2.circle(bg, (x, 10), 22, G, -1, AA)
    bg[:10] = (236, 244, 250)
    for x in range(-10, W + 50, 40):
        cv2.circle(bg, (x, 10), 22, G, -1, AA)
    # manchas de pasto
    rnd = random.Random(12)
    for _ in range(9):
        cv2.ellipse(bg, (rnd.randint(30, W - 30), rnd.randint(70, H - 30)),
                    (rnd.randint(26, 52), rnd.randint(8, 13)), 0, 0, 360, (128, 214, 38), -1, AA)
    # sombra de pasto bajo el marco y camino
    cv2.ellipse(bg, (400, 424), (330, 24), 0, 0, 360, (112, 202, 34), -1, AA)
    # arbusto izquierdo + palito amarillo
    cv2.circle(bg, (-8, 300), 34, (36, 170, 96), -1, AA)
    rrect(bg, (4, 284), (12, 318), (255, 214, 40), 3)
    # matas
    ORN, GRN8 = (250, 118, 28), (84, 176, 24)
    for x, y, s, c in ((40, 160, .9, ORN), (22, 166, .7, GRN8), (56, 168, .6, ORN),
                       (780, 96, 1.2, ORN), (754, 100, 1.0, ORN), (690, 50, .9, GRN8),
                       (122, 470, 1.1, ORN), (100, 474, .8, GRN8),
                       (700, 470, 1.0, GRN8), (720, 474, .8, ORN)):
        _tuft(bg, x, y, s, c)
    # marco de madera naranja
    rrect(bg, (92, 74), (708, 432), (120, 64, 10), 56)
    rrect(bg, (92, 66), (708, 424), (176, 92, 14), 56)
    rrect(bg, (96, 70), (704, 418), (232, 124, 24), 52)
    rrect(bg, (104, 78), (696, 410), (246, 152, 40), 46)
    rrect(bg, (136, 114), (664, 394), (186, 96, 12), 24)
    # papel crema (aqui va el contenido)
    rrect(bg, (142, 120), (658, 388), (255, 250, 238), 14)
    # tornillos
    for hx, hy in ((116, 94), (684, 94), (116, 396), (684, 396)):
        cv2.ellipse(bg, (hx, hy + 2), (12, 8), 0, 0, 360, (240, 160, 60), -1, AA)
        cv2.ellipse(bg, (hx, hy), (12, 8), 0, 0, 360, (150, 76, 12), -1, AA)
    # letrero de madera que sobresale
    x1, y1, x2, y2 = 267, 30, 538, 102
    rrect(bg, (x1, y1 + 7), (x2, y2 + 7), (170, 90, 14), 20)
    rrect(bg, (x1, y1), (x2, y2), (242, 164, 52), 20, border=(206, 120, 24), t=4)
    rrect(bg, (x1 + 14, y1 + 8), (x2 - 14, y1 + 18), (255, 206, 120), 5)
    for nx in (x1 + 16, x2 - 16):
        cv2.circle(bg, (nx, (y1 + y2) // 2 + 4), 4, (176, 96, 16), -1, AA)
    sz = fit_size("YAREN PAINT", (x2 - x1) - 70, 34)
    put_c(bg, "YAREN PAINT", 403, 69, sz, (170, 90, 14))
    put_c(bg, "YAREN PAINT", 403, 66, sz, WHITE)
    return bg


_BG_MENU_FINAL = _build_menu_final()


def _draw_menu_final(self, f):
    specs = {
        "free": (self.tr("Dibujo libre", "Free draw"), GRN, icon_easel),
        "guided": (self.tr("Dibujar con Yaren", "Draw with Yaren"), BLUE, icon_pencil),
        "color": (self.tr("Colorear", "Color in"), ORNG, icon_palette),
    }
    for key, r in MODE_RECTS:
        label, col, icon = specs[key]
        bx1, by1, bx2, by2 = r
        cx = (bx1 + bx2) // 2
        cy = (by1 + by2) // 2
        
        _wood_button(f, r, "", col)
        
        icon(f, cx, cy - 26, col)
        
        lines = _wrap(label, sc(22), (bx2 - bx1) - 16)
        y_text = cy + 40 if len(lines) == 1 else cy + 28
        
        for line in lines:
            s2 = fit_size(line, (bx2 - bx1) - 16, 22)
            put_c(f, line, cx, y_text + 2, s2, _dark(col, .6))
            put_c(f, line, cx, y_text, s2, WHITE)
            y_text += 26

    _wood_button(f, QUIT_R, self.tr("Salir", "Exit"), (214, 84, 64), size=16)

_prev_render_final = PaintNode._render


def _render_final(self):
    if self._screen != "menu":
        return _prev_render_final(self)
    f = _BG_MENU_FINAL.copy()
    self._draw_menu(f)
    if getattr(self, "_sv", None):
        self._draw_save_overlay(f)
    return f


PaintNode._draw_menu = _draw_menu_final
PaintNode._render = _render_final
# ==== SKETCH_FINAL_BLOCK_END

# =============================================================================
#  MAIN
# =============================================================================
def _autostart():
    """Para probar sin manager: configure + activate via CLI."""
    time.sleep(1.0)
    for t in ("configure", "activate"):
        subprocess.run(["ros2", "lifecycle", "set", "/paint_node", t],
                       check=False, capture_output=True)
        time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    node = PaintNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    if node.get_parameter('autostart').value:
        threading.Thread(target=_autostart, daemon=True).start()
    try:
        node.run_display_main_thread()
    except KeyboardInterrupt:
        print("Interrupcion recibida, cerrando...")
    finally:
        node._active = False
        cv2.destroyAllWindows()
        spin_thread.join(timeout=2.0)
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()