#!/usr/bin/env python3
"""
memoria_node.py  –  Juego de Memoria tipo Simon Says para YAREN2
Paquete: yaren_juegos

Visual PIXEL-ART (estilo arcade):
  · Todo se dibuja en un lienzo de baja resolución (200x120) y se escala x4
    con INTER_NEAREST -> pixeles grandes y nítidos.
  · Fuente bitmap 5x7 propia.
  · Sin assets externos: todo es código (numpy + cv2).

Cambios de esta versión:
  · Márgenes aumentados para evitar que los textos se vean pegados.
  · Lógica condicional de colores en corazones en el Game Over (Rojo si gana, gris si pierde).
  · Tamaño de fuente reducido para texto secundario en Game Over.
"""

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_msgs.msg import String, Bool
from rclpy.qos import QoSProfile, DurabilityPolicy

import cv2
import numpy as np
import threading
import time
import random
import os

# ─── Resolución ───────────────────────────────────────────────────────────────
W, H = 800, 480          # ventana real (los clics usan estas coordenadas)
S = 4                    # factor de escala pixel
LW, LH = W // S, H // S  # lienzo lógico 200 x 120
GROUND_Y = 112

# ─── Paleta BGR ───────────────────────────────────────────────────────────────
COLORS = {
    "ROJO":     (60,  60,  230),
    "AZUL":     (230, 130,  40),
    "VERDE":    (70,  200,  70),
    "AMARILLO": (30,  210, 250),
}
COLOR_NAMES = list(COLORS.keys())

QUADRANT_CORNERS = {
    "ROJO":     ((0,   0),   (400, 240)),
    "AZUL":     ((400, 0),   (800, 240)),
    "VERDE":    ((0,   240), (400, 480)),
    "AMARILLO": ((400, 240), (800, 480)),
}

# posiciones en el lienzo LÓGICO (200x120)
BALL_POSITIONS = {
    "ROJO":     (50,  30),
    "AZUL":     (150, 30),
    "VERDE":    (50,  77),
    "AMARILLO": (150, 77),
}
BALL_R = 11

SPEED_TABLE = [
    (1,  0.85, 0.30),
    (2,  0.75, 0.25),
    (3,  0.62, 0.22),
    (4,  0.48, 0.18),
    (5,  0.36, 0.14),
    (6,  0.27, 0.11),
    (7,  0.20, 0.09),
    (8,  0.15, 0.07),
    (9,  0.11, 0.06),
    (10, 0.08, 0.05),
]
MAX_ROUNDS    = 10
INPUT_TIMEOUT = 6.0

# ─── Paleta UI ────────────────────────────────────────────────────────────────
BG_DARK   = (40, 22, 14)
BG_NIGHT  = (62, 38, 28)
WHITE     = (255, 255, 255)
GOLD_T    = (90, 215, 255)     # dorado (parte alta de la letra)
GOLD_B    = (30, 140, 245)     # naranja (parte baja)
OUT       = (85, 40, 25)       # contorno azul oscuro
SHADOW    = (55, 25, 12)
GREEN_T   = (130, 240, 160)
GREEN_B   = (50, 170, 50)
RED_T     = (120, 120, 255)
RED_B     = (40, 40, 200)
EASY_C    = (110, 235, 130)
HARD_C    = (50, 185, 250)
INSANE_C  = (90, 90, 255)
HEART_RED = (50, 50, 225)
HEART_GRY = (100, 100, 100)

# ─── Audio ────────────────────────────────────────────────────────────────────
HOME_DIR   = os.path.expanduser("~")
BASE_WS    = os.path.join(HOME_DIR, "robotis_ws", "src", "YAREN2")
AUDIO_BASE = os.path.join(BASE_WS, "yaren_juegos", "sounds")


def _find_sound(name):
    for ext in ("wav", "mp3"):
        p = os.path.join(AUDIO_BASE, f"{name}.{ext}")
        if os.path.isfile(p):
            return p
    return None


SOUNDS = {
    "acierto":  _find_sound("acierto"),
    "error":    _find_sound("error"),
    "gameover": _find_sound("gameover"),
}


class _NullSound:
    def play(self, key: str): pass
    def quit(self): pass


class SoundManager:
    """WAV vía pygame.mixer.Sound (solapable); MP3 vía pygame.mixer.music."""
    def __init__(self):
        self._ok    = False
        self._cache = {}
        self._mp3   = {}
        self._pygame = None
        try:
            import pygame
            pygame.mixer.pre_init(frequency=44100, size=-16, channels=2, buffer=512)
            pygame.mixer.init()
            self._pygame = pygame
            self._ok = True
            self._load_sounds()
        except Exception as e:
            print(f"[SoundManager] No se pudo inicializar pygame.mixer: {e}")

    def _load_sounds(self):
        # Todos los efectos (WAV o MP3) se cargan como Sound, asi nunca usan
        # pygame.mixer.music, que queda reservado para la musica de fondo.
        for key, path in SOUNDS.items():
            if not path:
                print(f"[SoundManager] Archivo no encontrado para: {key}")
                continue
            try:
                self._cache[key] = self._pygame.mixer.Sound(path)
            except Exception as e:
                print(f"[SoundManager] Error cargando {key} ({path}): {e}")

    def play(self, key: str):
        if not self._ok:
            return
        snd = self._cache.get(key)
        if snd:
            try:
                snd.play()
            except Exception:
                pass

    def quit(self):
        if self._ok:
            try:
                self._pygame.mixer.quit()
            except Exception:
                pass


def _make_sound_manager():
    try:
        return SoundManager()
    except Exception:
        return _NullSound()


# =============================================================================
#  PIXEL ENGINE  (todo en coordenadas del lienzo lógico 200x120)
# =============================================================================

# ── Fuente bitmap 5x7 ─────────────────────────────────────────────────────────
_FONT_SRC = {
    "A": "01110 10001 10001 11111 10001 10001 10001",
    "B": "11110 10001 10001 11110 10001 10001 11110",
    "C": "01110 10001 10000 10000 10000 10001 01110",
    "D": "11110 10001 10001 10001 10001 10001 11110",
    "E": "11111 10000 10000 11110 10000 10000 11111",
    "F": "11111 10000 10000 11110 10000 10000 10000",
    "G": "01110 10001 10000 10111 10001 10001 01111",
    "H": "10001 10001 10001 11111 10001 10001 10001",
    "I": "01110 00100 00100 00100 00100 00100 01110",
    "J": "00111 00010 00010 00010 00010 10010 01100",
    "K": "10001 10010 10100 11000 10100 10010 10001",
    "L": "10000 10000 10000 10000 10000 10000 11111",
    "M": "10001 11011 10101 10101 10001 10001 10001",
    "N": "10001 11001 10101 10011 10001 10001 10001",
    "O": "01110 10001 10001 10001 10001 10001 01110",
    "P": "11110 10001 10001 11110 10000 10000 10000",
    "Q": "01110 10001 10001 10001 10101 10010 01101",
    "R": "11110 10001 10001 11110 10100 10010 10001",
    "S": "01111 10000 10000 01110 00001 00001 11110",
    "T": "11111 00100 00100 00100 00100 00100 00100",
    "U": "10001 10001 10001 10001 10001 10001 01110",
    "V": "10001 10001 10001 10001 10001 01010 00100",
    "W": "10001 10001 10001 10101 10101 11011 10001",
    "X": "10001 10001 01010 00100 01010 10001 10001",
    "Y": "10001 10001 01010 00100 00100 00100 00100",
    "Z": "11111 00001 00010 00100 01000 10000 11111",
    "0": "01110 10001 10011 10101 11001 10001 01110",
    "1": "00100 01100 00100 00100 00100 00100 01110",
    "2": "01110 10001 00001 00010 00100 01000 11111",
    "3": "11110 00001 00001 01110 00001 00001 11110",
    "4": "00010 00110 01010 10010 11111 00010 00010",
    "5": "11111 10000 11110 00001 00001 10001 01110",
    "6": "00110 01000 10000 11110 10001 10001 01110",
    "7": "11111 00001 00010 00100 01000 01000 01000",
    "8": "01110 10001 10001 01110 10001 10001 01110",
    "9": "01110 10001 10001 01111 00001 00010 01100",
    "!": "00100 00100 00100 00100 00100 00000 00100",
    "?": "01110 10001 00001 00010 00100 00000 00100",
    ":": "00000 00100 00000 00000 00000 00100 00000",
    "-": "00000 00000 00000 11111 00000 00000 00000",
    ".": "00000 00000 00000 00000 00000 01100 01100",
    "/": "00001 00001 00010 00100 01000 10000 10000",
    "|": "00100 00100 00100 00100 00100 00100 00100",
    " ": "00000 00000 00000 00000 00000 00000 00000",
}
_GLYPHS = {k: np.array([[int(c) for c in row] for row in v.split()], np.uint8)
           for k, v in _FONT_SRC.items()}


from functools import lru_cache

def _mk_glyph(s):
    return np.array([[int(ch) for ch in r] for r in s.split()], np.uint8)

_GLYPHS["¿"] = _mk_glyph("00100 00000 00100 00100 01000 10001 01110")
_GLYPHS[","] = _mk_glyph("00000 00000 00000 00000 00110 00100 01000")

_TEXT_Q = {}   # id(frame) -> textos pendientes (se dibujan en _push, a resolucion real)


def text_width(text, scale=1):
    """Ancho REAL (px, ventana 800x480) del texto."""
    c = scale * S
    t = max(2, c // 2)
    return max(0, len(text) * (6 * c + t) - c + t)


@lru_cache(maxsize=512)
def _chunky_mask(text, scale):
    c = scale * S
    t = max(2, c // 2)
    adv = 6 * c + t
    w = len(text) * adv - c
    h = 7 * c
    if w <= 0:
        return np.zeros((0, 0), bool)
    base = np.zeros((h + t, w + t), bool)
    one = np.ones((c, c), np.uint8)
    for i, ch in enumerate(text.upper()):
        g = _GLYPHS.get(ch)
        if g is None:
            continue
        x0 = i * adv
        base[:h, x0:x0 + 5 * c] |= np.kron(g, one).astype(bool)
    b = base.copy()
    b[:, t:] |= base[:, :-t]          # trazo mas grueso a la derecha
    out = b.copy()
    out[t:, :] |= b[:-t, :]           # y hacia abajo
    return out


def _paint(img, mask, ox, oy, color):
    h, w = mask.shape
    ih, iw = img.shape[:2]
    x0, y0 = max(ox, 0), max(oy, 0)
    x1, y1 = min(ox + w, iw), min(oy + h, ih)
    if x1 <= x0 or y1 <= y0:
        return
    m = mask[y0 - oy:y1 - oy, x0 - ox:x1 - ox]
    img[y0:y1, x0:x1][m] = np.asarray(color, np.uint8)


def _draw_text_real(img, text, x, y, scale, top, bot, outline, center):
    mask = _chunky_mask(text, scale)
    if mask.size == 0:
        return
    h, w = mask.shape
    c = scale * S
    e = max(2, c // 2)                 # grosor de la base 3D
    x *= S
    y *= S
    if center:
        x -= w // 2
    ext = np.zeros((h + e, w), bool)
    ext[:h] |= mask
    ext[e:] |= mask
    if outline is not None:
        ow = 2 if scale < 3 else 4
        grown = cv2.dilate(np.pad(ext, ow).astype(np.uint8),
                           np.ones((2 * ow + 1, 2 * ow + 1), np.uint8)).astype(bool)
        _paint(img, grown, x - ow, y - ow, outline)
    shade = bot if (bot is not None and tuple(bot) != tuple(top)) else _mul(top, 0.55)
    _paint(img, ext, x, y, shade)      # base naranja / sombra
    _paint(img, mask, x, y, top)       # cara de la letra
    if scale >= 3:                     # pixel blanco de brillo (solo titulos grandes)
        adv = 6 * c + max(2, c // 2)
        hs = max(3, c // 3)
        for i, ch in enumerate(text.upper()):
            g = _GLYPHS.get(ch)
            if g is None or not g[0].any():
                continue
            hx = x + i * adv + int(np.argmax(g[0])) * c + 2
            cv2.rectangle(img, (hx, y + 2), (hx + hs - 1, y + hs + 1), WHITE, -1)


def draw_text(img, text, x, y, scale=1, top=WHITE, bot=None, outline=OUT,
              shadow=True, center=False):
    """Misma firma de siempre. x, y en coordenadas del lienzo logico."""
    if not text:
        return
    _TEXT_Q.setdefault(id(img), []).append((text, x, y, scale, top, bot, outline, center))


def _mul(c, k):
    return tuple(int(min(255, v * k)) for v in c)

def _lighten(c, a):
    return tuple(int(min(255, v + a)) for v in c)

# ── Fondo: cielo + nubes + suelo ──────────────────────────────────────────────
def _build_sky():
    img = np.zeros((LH, LW, 3), np.uint8)
    top = np.array((235, 165, 60), np.float32)
    bot = np.array((250, 215, 135), np.float32)
    bands = 10
    for r in range(LH):
        b = min(bands - 1, int(r / GROUND_Y * bands))
        t = b / (bands - 1)
        img[r] = (top * (1 - t) + bot * t).astype(np.uint8)
    img[GROUND_Y:] = (140, 150, 190)                       # tierra
    for y in range(GROUND_Y + 3, LH):
        for x in range(LW):
            if (x * 7 + y * 13) % 11 == 0:
                img[y, x] = (95, 105, 150)
    img[GROUND_Y:GROUND_Y + 1] = (110, 235, 150)           # pasto claro
    img[GROUND_Y + 1:GROUND_Y + 3] = (55, 175, 70)         # pasto
    return img


_SKY_BASE = _build_sky()
_CLOUDS = [(20, 16, 1.0, 3.0), (110, 30, 1.4, 2.0),
           (165, 8, 0.8, 4.0), (60, 52, 0.9, 2.5)]   # x0, y, escala, velocidad


def _draw_cloud(img, x, y, s):
    parts = [(0, 0, 6), (8, -3, 8), (17, 0, 6), (8, 3, 6)]
    for dy, col in ((1, (235, 215, 190)), (0, (255, 255, 255))):
        for px, py, pr in parts:
            cv2.circle(img, (int(x + px * s), int(y + py * s + dy)),
                       max(2, int(pr * s)), col, -1)


def _sky(clouds=True):
    img = _SKY_BASE.copy()
    if clouds:
        t = time.time()
        for x0, y, s, sp in _CLOUDS:
            _draw_cloud(img, (x0 + t * sp) % (LW + 60) - 30, y, s)
    return img


def _dark_bg():
    """Fondo plano oscuro para las pantallas donde importan los colores."""
    img = np.zeros((LH, LW, 3), np.uint8)
    img[:] = BG_NIGHT
    return img


# ── Paneles y botones ─────────────────────────────────────────────────────────
def _pixel_panel(img, x1, y1, x2, y2, fill, border, hi=None, shadow=False):
    if shadow:
        _pixel_panel(img, x1, y1 + 2, x2, y2 + 2, (40, 20, 10), (40, 20, 10))
    cv2.rectangle(img, (x1 + 1, y1), (x2 - 1, y2), border, -1)
    cv2.rectangle(img, (x1, y1 + 1), (x2, y2 - 1), border, -1)
    cv2.rectangle(img, (x1 + 2, y1 + 1), (x2 - 2, y2 - 1), fill, -1)
    cv2.rectangle(img, (x1 + 1, y1 + 2), (x2 - 1, y2 - 2), fill, -1)
    if hi:
        cv2.line(img, (x1 + 2, y1 + 1), (x2 - 2, y1 + 1), hi, 1)


def _draw_button(img, p1, p2, label, fill, hi, border):
    (x1, y1), (x2, y2) = p1, p2
    _pixel_panel(img, x1, y1, x2, y2, fill, border, hi=hi, shadow=True)
    draw_text(img, label, (x1 + x2) // 2, y1 + (y2 - y1 - 8) // 2,
              scale=1, center=True)


BTN_START = dict(fill=(40, 150, 245), hi=(110, 210, 255), border=(20, 60, 130))
BTN_BACK  = dict(fill=(130, 95, 60),  hi=(170, 140, 100), border=(70, 40, 25))

# ── Sprites ───────────────────────────────────────────────────────────────────
_HEART = [
    ".ooo...ooo.",
    "orrrooorrro",
    "orhrrrrrrro",
    "orhrrrrrrro",
    "orrrrrrrrro",
    ".orrrrrrro.",
    "..orrrrro..",
    "...orrro...",
    "....oro....",
    ".....o.....",
]
_HEART_ARR = np.array([list(r) for r in _HEART])
_HEART_M = {c: (_HEART_ARR == c) for c in "orh"}


def _draw_heart(img, x, y, custom_color=None, scale=1):
    """ Dibuja un corazón en pixel art. 
        Si custom_color está definido, lo pinta de ese color. 
        De lo contrario, usa el predeterminado rojo (lleno) o gris oscuro (vacío).
    """
    if custom_color is not None:
        pal = {'o': _mul(custom_color, 0.4), 'r': custom_color, 'h': _lighten(custom_color, 80)}
    else:
        pal = {'o': (30, 20, 110), 'r': HEART_RED, 'h': (235, 235, 255)}

    one = np.ones((scale, scale), np.uint8)
    for c, m in _HEART_M.items():
        _paint(img, np.kron(m.astype(np.uint8), one).astype(bool), x, y, pal[c])


def _draw_ball(img, c, r, color, lit=True, glow=False):
    if glow:
        for dr, a in ((6, 0.30), (3, 0.40)):
            ov = img.copy()
            cv2.circle(ov, c, r + dr, color, -1)
            cv2.addWeighted(ov, a, img, 1 - a, 0, img)
    base = color if lit else _mul(color, 0.35)
    cx, cy = c
    cv2.circle(img, c, r + 1, _mul(base, 0.35), -1)
    cv2.circle(img, c, r, _mul(base, 0.72), -1)
    cv2.circle(img, (cx - 1, cy - 1), r - 2, base, -1)
    hl = WHITE if lit else _mul(base, 2.2)
    hx, hy = cx - r // 2 - 1, cy - r // 2 - 1
    cv2.rectangle(img, (hx, hy), (hx + 2, hy + 2), hl, -1)
    cv2.rectangle(img, (hx + 4, hy + 2), (hx + 4, hy + 2), hl, -1)


# =============================================================================
#  ESTADOS Y BOTONES (coordenadas REALES 800x480 para los clics)
# =============================================================================
class State:
    INTRO         = "intro"
    IDLE          = "idle"
    COUNTDOWN     = "countdown"
    SHOW_SEQUENCE = "show_sequence"
    PLAYER_INPUT  = "player_input"
    FEEDBACK      = "feedback"
    GAME_OVER     = "game_over"


def _real(p1, p2):
    return ((p1[0] * S, p1[1] * S), (p2[0] * S, p2[1] * S))


# Botones en coordenadas del lienzo lógico
_L_START = ((40, 90), (100, 110))     # EMPEZAR / SIGUIENTE / JUGAR
_L_BACK  = ((110, 90), (170, 110))    # VOLVER / ATRAS
_L_RETRY = ((20, 90), (100, 110))     # REINTENTAR (game over)
_L_EXIT  = ((110, 90), (170, 110))    # SALIR (game over)
# Botones en coordenadas reales (para los clics)
_INTRO_BTN_START = _real(*_L_START)
_INTRO_BTN_BACK  = _real(*_L_BACK)
_GO_BTN_RETRY    = _real(*_L_RETRY)
_GO_BTN_EXIT     = _real(*_L_EXIT)

# =============================================================================
#  MUSICA DE FONDO (pygame.mixer.music) + ICONO DE BOCINA
# =============================================================================
MUSIC_FILE = os.path.expanduser('~/robotis_ws/src/YAREN2/yaren_juegos/sounds/SONIC.mp3')
MUSIC_DEFAULT_VOLUME = 0.5


class MusicPlayer:
    """Musica de fondo en bucle con pygame.mixer.music.

    - Solo maneja la musica: los efectos usan pygame.mixer.Sound (SoundManager).
    - Mute y volumen afectan unicamente a la musica.
    - Si falta pygame o el archivo, el juego sigue funcionando sin musica.
    """

    def __init__(self, path, logger=None, volume=MUSIC_DEFAULT_VOLUME):
        self.path = path
        self.logger = logger
        self.volume = volume
        self.muted = False
        self.playing = False
        self._own_mixer = False      # True si fue este reproductor quien inicio el mixer
        self._lock = threading.RLock()

    def _warn(self, msg):
        if self.logger is not None:
            self.logger.warning(msg)

    def _level(self):
        return 0.0 if self.muted else self.volume

    def play(self):
        with self._lock:
            if self.playing:
                return
            if not os.path.isfile(self.path):
                self._warn(f'Musica no encontrada: {self.path}')
                return
            try:
                os.environ.setdefault('PYGAME_HIDE_SUPPORT_PROMPT', '1')
                import pygame
                if not pygame.mixer.get_init():
                    pygame.mixer.init()
                    self._own_mixer = True
                pygame.mixer.music.load(self.path)
                pygame.mixer.music.set_volume(self._level())
                pygame.mixer.music.play(loops=-1, fade_ms=800)
                self.playing = True
            except Exception as e:
                self._warn(f'No se pudo reproducir la musica: {e}')

    def stop(self, fade_ms=0):
        """Detiene la musica. Con fade_ms baja el volumen poco a poco (en un hilo aparte)."""
        with self._lock:
            if not self.playing:
                return
            self.playing = False

        def _halt():
            with self._lock:
                if self.playing:         # la volvieron a iniciar mientras tanto
                    return
                try:
                    import pygame
                    if fade_ms > 0:
                        pygame.mixer.music.fadeout(fade_ms)
                    else:
                        pygame.mixer.music.stop()
                    if self._own_mixer:  # libera el audio solo si lo abrimos nosotros
                        pygame.mixer.quit()
                        self._own_mixer = False
                except Exception:
                    pass

        if fade_ms > 0:
            threading.Thread(target=_halt, daemon=True).start()
        else:
            _halt()

    def _apply(self):
        with self._lock:
            if not self.playing:
                return
            try:
                import pygame
                pygame.mixer.music.set_volume(self._level())
            except Exception:
                pass

    def toggle_mute(self):
        self.muted = not self.muted
        self._apply()

    def change_volume(self, delta):
        self.volume = round(max(0.0, min(1.0, self.volume + delta)), 2)
        if delta > 0:
            self.muted = False           # subir el volumen quita el mute
        self._apply()


def draw_speaker(img, cx, cy, color, muted, level=1.0):
    """Bocina con ondas segun el volumen, o con una X si esta muteada."""
    cone = np.array([(cx - 9, cy - 4), (cx - 4, cy - 4), (cx + 3, cy - 10),
                     (cx + 3, cy + 10), (cx - 4, cy + 4), (cx - 9, cy + 4)], np.int32)
    cv2.fillPoly(img, [cone], color, cv2.LINE_AA)
    if muted:
        cv2.line(img, (cx + 8, cy - 5), (cx + 16, cy + 5), color, 3, cv2.LINE_AA)
        cv2.line(img, (cx + 16, cy - 5), (cx + 8, cy + 5), color, 3, cv2.LINE_AA)
    else:
        waves = 1 if level < 0.34 else (2 if level < 0.67 else 3)
        for i in range(waves):
            r = 7 + 5 * i
            cv2.ellipse(img, (cx + 3, cy), (r, r), 0, -50, 50, color, 2, cv2.LINE_AA)


# =============================================================================
#  NODO PRINCIPAL
# =============================================================================
class MemoriaNode(LifecycleNode):

    MAX_LIVES = 3
    AUD_C = (750, 36)
    AUD_PANEL = (580, 62, 780, 124)
    AUD_MUTE = (608, 93, 17)
    AUD_MINUS = (650, 93, 15)
    AUD_PLUS = (752, 93, 15)

    def __init__(self):
        super().__init__('memoria_node')
        self._window          = "YAREN2 - Juego de Memoria"
        self._active          = False
        self._lock            = threading.Lock()
        self._game_thread     = None
        self.is_english       = False
        self._music = MusicPlayer(MUSIC_FILE, self.get_logger())
        self.audio_open = False
        self._audio_t = 0.0

        self._sound = _NullSound()

        self._frame      = None
        self._frame_lock = threading.Lock()

        self._sequence: list = []
        self._round    = 0
        self._lives    = self.MAX_LIVES
        self._misses   = 0
        self._score    = 0
        self._state    = State.INTRO

        self._completed_this_round: list = []
        self._flash_color = None
        self._flash_until = 0.0
        self._game_over_sound_played = False
        self._winner = False

        self._user_choice    = None
        self._choice_event   = threading.Event()

        self._intro_choice   = None
        self._intro_event    = threading.Event()
        self._intro_page     = 0        # 0 = título, 1 = cómo jugar, 2 = dificultad

        self._go_choice      = None     # None | "retry" | "exit"

        self._mode_pub = None
        self._lang_sub = None

    # ─────────────────────────────────────────────────────────────────────────
    #  LIFECYCLE
    # ─────────────────────────────────────────────────────────────────────────

    def on_configure(self, state):
        self.get_logger().info('MemoriaNode: configurando...')
        try:
            self._mode_pub = self.create_publisher(String, '/yaren_mode', 1)
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self._lang_sub = self.create_subscription(
                Bool, '/yaren/is_english', self._lang_cb, qos)

            self._sound = _make_sound_manager()
            if isinstance(self._sound, _NullSound):
                self.get_logger().warn('Audio no disponible. Sigo sin sonido.')
            else:
                self.get_logger().info('Audio inicializado correctamente.')

            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'on_configure error: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        self.get_logger().info('MemoriaNode: ACTIVO')
        self._active = True
        self._choice_event.clear()
        self._intro_event.clear()
        self._intro_choice = None
        self._intro_page   = 0
        self._go_choice    = None
        self._user_choice  = None
        self._frame        = None
        self._set_state(State.INTRO)

        self._game_thread = threading.Thread(target=self._game_loop, daemon=True)
        self._game_thread.start()

        return super().on_activate(state)

    def on_deactivate(self, state):
        self.get_logger().info('MemoriaNode: INACTIVO')
        self._active = False
        self._music.stop()
        self._choice_event.set()
        self._intro_event.set()
        self._game_thread = None
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        if self._mode_pub:   self.destroy_publisher(self._mode_pub)
        if self._lang_sub:   self.destroy_subscription(self._lang_sub)
        self._music.stop()
        self._sound.quit()
        cv2.destroyAllWindows()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self._active = False
        self._choice_event.set()
        self._intro_event.set()
        self._music.stop()
        self._sound.quit()
        cv2.destroyAllWindows()
        return TransitionCallbackReturn.SUCCESS

    # ─────────────────────────────────────────────────────────────────────────

    def _lang_cb(self, msg):
        self.is_english = msg.data

    def _on_click(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        with self._lock:
            state = self._state

        if self._handle_audio_click(x, y, state):
            return

        # Intro (los 3 slides usan los mismos dos botones)
        if state == State.INTRO:
            (sx1, sy1), (sx2, sy2) = _INTRO_BTN_START
            (bx1, by1), (bx2, by2) = _INTRO_BTN_BACK
            if sx1 <= x <= sx2 and sy1 <= y <= sy2:
                self._intro_choice = "start"
                self._intro_event.set()
            elif bx1 <= x <= bx2 and by1 <= y <= by2:
                self._intro_choice = "back"
                self._intro_event.set()
            return

        # Game over: REINTENTAR o SALIR
        if state == State.GAME_OVER:
            (rx1, ry1), (rx2, ry2) = _GO_BTN_RETRY
            (ex1, ey1), (ex2, ey2) = _GO_BTN_EXIT
            if rx1 <= x <= rx2 and ry1 <= y <= ry2:
                self._go_choice = "retry"
            elif ex1 <= x <= ex2 and ey1 <= y <= ey2:
                self._go_choice = "exit"
                self._publish_idle()
                self._active = False
                self._choice_event.set()
            return

        # Botón X (salir de la partida)
        if x < 60 and y < 60:
            if state not in (State.GAME_OVER, State.INTRO):
                self._sound.play("error")
                self._set_state(State.GAME_OVER)
                self._choice_event.set()
            return

        if state == State.PLAYER_INPUT:
            for color, ((x1, y1), (x2, y2)) in QUADRANT_CORNERS.items():
                if x1 <= x < x2 and y1 <= y < y2:
                    self._user_choice = color
                    self._choice_event.set()
                    return

    def run_display_main_thread(self):
        win_name = 'YAREN2 - Juego de Memoria'
        import subprocess

        while rclpy.ok():
            while not self._active and rclpy.ok():
                time.sleep(0.05)

            if not rclpy.ok():
                break

            cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(win_name, W, H)
            cv2.setWindowProperty(win_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            cv2.setWindowProperty(win_name, cv2.WND_PROP_TOPMOST, 1)
            cv2.setMouseCallback(win_name, self._on_click)

            def _force_focus():
                time.sleep(0.5)
                try:
                    subprocess.run(
                        ["xdotool", "search", "--name", win_name, "windowactivate", "--sync", "windowfocus"],
                        check=False, capture_output=True
                    )
                except FileNotFoundError:
                    pass

            threading.Thread(target=_force_focus, daemon=True).start()

            self._reset_game()
            self._set_state(State.INTRO)
            self._choice_event.clear()
            self._intro_event.clear()

            frame = np.zeros((H, W, 3), dtype=np.uint8)
            self._music.play()

            while self._active and rclpy.ok():
                try:
                    if cv2.getWindowProperty(win_name, cv2.WND_PROP_VISIBLE) < 1:
                        self.get_logger().info("Ventana cerrada con X. Publicando idle...")
                        self._publish_idle()
                        self._active = False
                        break
                except Exception:
                    self.get_logger().info("Ventana destruida. Publicando idle...")
                    self._publish_idle()
                    self._active = False
                    break

                with self._frame_lock:
                    if self._frame is not None:
                        frame = self._frame.copy()
                    else:
                        frame[:] = BG_DARK

                self._draw_audio(frame)
                cv2.imshow(win_name, frame)
                key = cv2.waitKey(16) & 0xFF
                if key == 27:
                    self._active = False
                    break

            self._music.stop()
            try:
                cv2.destroyAllWindows()
                cv2.waitKey(1)
            except Exception:
                pass

            print("MemoriaNode desactivado, esperando reactivacion...")
            time.sleep(0.2)

    # ------------- Audio (musica de fondo) -------------
    @staticmethod
    def _in_circle(x, y, c, extra=6):
        return (x - c[0]) ** 2 + (y - c[1]) ** 2 <= (c[2] + extra) ** 2

    def _handle_audio_click(self, x, y, state):
        """True si el toque fue del control de audio (entonces el juego lo ignora)."""
        cx, cy = self.AUD_C
        if self._in_circle(x, y, (cx, cy, 18), 8):
            if state != State.SHOW_SEQUENCE:     # no taparle los colores al jugador
                self.audio_open = not self.audio_open
                self._audio_t = time.time()
            return True
        if not self.audio_open:
            return False
        x1, y1, x2, y2 = self.AUD_PANEL
        if x1 <= x <= x2 and y1 <= y <= y2:
            self._audio_t = time.time()
            if self._in_circle(x, y, self.AUD_MUTE):
                self._music.toggle_mute()
            elif self._in_circle(x, y, self.AUD_MINUS):
                self._music.change_volume(-0.1)
            elif self._in_circle(x, y, self.AUD_PLUS):
                self._music.change_volume(0.1)
            return True
        self.audio_open = False      # toque fuera del panel: se cierra y el toque sigue su curso
        return False

    def _draw_audio(self, frame):
        m = self._music
        if self._state == State.SHOW_SEQUENCE:
            self.audio_open = False
        elif self.audio_open and time.time() - self._audio_t > 4.0:
            self.audio_open = False  # se esconde solo tras 4 s sin tocarlo

        WHITE_ = (255, 255, 255)
        cx, cy = self.AUD_C
        col = (150, 160, 165) if m.muted else (225, 80, 25)
        cv2.circle(frame, (cx, cy + 3), 18, _mul(col, 0.55), -1, cv2.LINE_AA)
        cv2.circle(frame, (cx, cy), 18, col, -1, cv2.LINE_AA)
        cv2.circle(frame, (cx, cy), 18, WHITE_, 2, cv2.LINE_AA)
        draw_speaker(frame, cx - 3, cy, WHITE_, m.muted, m.volume)
        if not self.audio_open:
            return

        x1, y1, x2, y2 = self.AUD_PANEL
        cv2.rectangle(frame, (x1 + 4, y1 + 6), (x2 + 4, y2 + 6), (40, 25, 15), -1)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (250, 253, 255), -1)
        cv2.rectangle(frame, (x1, y1), (x2, y2), col, 3)

        mx, my, mr = self.AUD_MUTE
        mcol = (150, 160, 165) if m.muted else (50, 100, 240)
        cv2.circle(frame, (mx, my + 2), mr, _mul(mcol, 0.55), -1, cv2.LINE_AA)
        cv2.circle(frame, (mx, my), mr, mcol, -1, cv2.LINE_AA)
        draw_speaker(frame, mx - 3, my, WHITE_, m.muted, m.volume)

        for (bx, by, br), plus, ok in ((self.AUD_MINUS, False, m.volume > 0.0),
                                       (self.AUD_PLUS, True, m.volume < 1.0)):
            bcol = (150, 52, 112) if ok else (190, 200, 205)
            cv2.circle(frame, (bx, by + 2), br, _mul(bcol, 0.55), -1, cv2.LINE_AA)
            cv2.circle(frame, (bx, by), br, bcol, -1, cv2.LINE_AA)
            cv2.line(frame, (bx - 6, by), (bx + 6, by), WHITE_, 3, cv2.LINE_AA)
            if plus:
                cv2.line(frame, (bx, by - 6), (bx, by + 6), WHITE_, 3, cv2.LINE_AA)

        # Barra de volumen entre el "-" y el "+"
        bar_x = self.AUD_MINUS[0] + 24
        base_y = self.AUD_MUTE[1] + 8
        filled = 0 if m.muted else int(round(m.volume * 10))
        for i in range(10):
            x = bar_x + i * 5
            cv2.rectangle(frame, (x, base_y - (6 + i)), (x + 3, base_y),
                          (85, 185, 60) if i < filled else (190, 200, 205), -1)
        label = 'MUTE' if m.muted else f'{int(round(m.volume * 100))}%'
        (tw, _), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_DUPLEX, 0.5, 1)
        cv2.putText(frame, label, (bar_x + 24 - tw // 2, self.AUD_MUTE[1] + 26),
                    cv2.FONT_HERSHEY_DUPLEX, 0.5, (125, 105, 95), 1, cv2.LINE_AA)

    # ─────────────────────────────────────────────────────────────────────────
    #  GAME LOOP
    # ─────────────────────────────────────────────────────────────────────────

    def _game_loop(self):
        self._set_state(State.INTRO)
        self._show_intro_screen()

        if not self._active:
            return

        if self._intro_choice == "back":
            self._publish_idle()
            self._active = False
            return

        # Partida + reintentos
        while self._active:
            self._play_one_game()
            if self._go_choice == "retry" and self._active:
                continue
            break

        if not self._active:
            self._publish_idle()

    def _play_one_game(self):
        self._reset_game()
        self._set_state(State.COUNTDOWN)
        self._show_countdown()

        while self._active and self._state != State.GAME_OVER and self._round < MAX_ROUNDS and rclpy.ok():
            self._sequence.append(random.choice(COLOR_NAMES))
            self._round += 1

            _speed = SPEED_TABLE[min(self._round - 1, MAX_ROUNDS - 1)]
            on_t, off_t = _speed[1], _speed[2]

            self._set_state(State.SHOW_SEQUENCE)
            self._play_sequence(on_t, off_t)
            if not self._active or self._state == State.GAME_OVER:
                break

            self._set_state(State.PLAYER_INPUT)
            self._completed_this_round = []
            self._flash_color = None
            round_passed = True
            total_steps  = len(self._sequence)

            for i, expected in enumerate(self._sequence):
                choice = self._wait_for_input(step_idx=i + 1, total_steps=total_steps)
                if not self._active or self._state == State.GAME_OVER:
                    break

                if choice != expected:
                    round_passed = False
                    self._lives -= 1
                    self._misses += 1
                    self._flash_color = choice
                    self._flash_until = time.time() + 0.25

                    self._sound.play("error")

                    self._set_state(State.FEEDBACK)
                    self._show_feedback(False, expected)
                    break

                self._completed_this_round.append(choice)
                self._flash_color = choice
                self._flash_until = time.time() + 0.30
                time.sleep(0.15)

            if not self._active or self._state == State.GAME_OVER:
                break

            if round_passed:
                self._score += 1
                self._sound.play("acierto")
                self._set_state(State.FEEDBACK)
                self._show_feedback(True, "")

            if self._lives <= 0:
                self._set_state(State.GAME_OVER)
                break

        if self._state != State.GAME_OVER and self._round >= MAX_ROUNDS and self._active:
            self._set_state(State.GAME_OVER)
            self._winner = True

        if self._state == State.GAME_OVER:
            self._show_game_over()

    # ─────────────────────────────────────────────────────────────────────────
    #  PANTALLA INTRO (3 slides)
    # ─────────────────────────────────────────────────────────────────────────

    def _show_intro_screen(self):
        self._intro_page = 0
        last_page = 2
        t0 = time.time()

        while self._active and rclpy.ok():
            self._intro_choice = None
            self._intro_event.clear()

            while self._active and not self._intro_event.is_set() and rclpy.ok():
                self._push(self._make_intro_frame(time.time() - t0, self._intro_page))
                time.sleep(0.033)

            if not self._active:
                return

            if self._intro_choice == "start":
                if self._intro_page < last_page:
                    self._intro_page += 1     # siguiente slide
                else:
                    return                    # último slide -> a jugar
            else:  # "back"
                if self._intro_page == 0:
                    return                    # primer slide -> salir al menú
                self._intro_page -= 1         # slide anterior

    def _make_intro_frame(self, t: float, page: int = 0) -> np.ndarray:
        en = self.is_english
        frame = _sky()

        # ── Slide 0: título ──
        if page == 0:
            bob = int(round(1.5 * np.sin(t * 2.5)))
            l1, l2 = ("MEMORY", "GAME") if en else ("JUEGO DE", "MEMORIA")
            draw_text(frame, l1, 100, 16 + bob, scale=2, top=GOLD_T, bot=GOLD_B, center=True)
            draw_text(frame, l2, 100, 38 + bob, scale=2, top=GOLD_T, bot=GOLD_B, center=True)

            active = int(t * 3) % 4
            for i, col in enumerate(COLORS.values()):
                _draw_ball(frame, (int(100 + (i - 1.5) * 24), 72), 6, col,
                           lit=True, glow=(i == active))

            _draw_button(frame, *_L_START, "START" if en else "EMPEZAR", **BTN_START)
            _draw_button(frame, *_L_BACK,  "BACK" if en else "VOLVER",   **BTN_BACK)
            return frame

        # ── Slide 1: cómo jugar ──
        if page == 1:
            draw_text(frame, "HOW TO PLAY?" if en else "¿COMO JUGAR?", 100, 8,
                      scale=2, top=GOLD_T, bot=GOLD_B, center=True)

            lines = (("MEMORIZE THE COLORS", "AND REPEAT THEM IN ORDER", "10 ROUNDS - 3 LIVES")
                     if en else
                     ("MEMORIZA LOS COLORES", "Y REPITELOS EN ORDEN", "10 RONDAS - 3 VIDAS"))

            draw_text(frame, lines[0], 100, 27, top=WHITE, center=True)
            draw_text(frame, lines[1], 100, 38, top=WHITE, center=True)
            draw_text(frame, lines[2], 100, 52, top=GOLD_T, bot=GOLD_B, center=True)

            n, hw, gap = self.MAX_LIVES, 22, 6
            x0 = 100 - (n * hw + (n - 1) * gap) // 2
            for i in range(n):
                _draw_heart(frame, x0 + i * (hw + gap), 64, custom_color=HEART_RED, scale=2)

            _draw_button(frame, *_L_START, "NEXT" if en else "SIGUIENTE", **BTN_START)
            _draw_button(frame, *_L_BACK,  "BACK" if en else "ATRAS",     **BTN_BACK)
            return frame

        # ── Slide 2: dificultad ──
        draw_text(frame, "DIFFICULTY" if en else "DIFICULTAD", 100, 8,
                  scale=2, top=GOLD_T, bot=GOLD_B, center=True)

        diffs = [("1-3",  "EASY" if en else "FACIL",       EASY_C),
                 ("4-6",  "HARD" if en else "DIFICIL",     HARD_C),
                 ("7-10", "INSANE" if en else "IMPOSIBLE", INSANE_C)]
        for i, (rng, lbl, col) in enumerate(diffs):
            y = 36 + i * 16
            draw_text(frame, rng, 55, y, top=col)
            draw_text(frame, lbl, 90, y, top=_mul(col, 0.9))

        _draw_button(frame, *_L_START, "PLAY" if en else "JUGAR", **BTN_START)
        _draw_button(frame, *_L_BACK,  "BACK" if en else "ATRAS", **BTN_BACK)
        return frame

    # ─────────────────────────────────────────────────────────────────────────
    #  HELPERS DE JUEGO
    # ─────────────────────────────────────────────────────────────────────────

    def _reset_game(self):
        self._sequence               = []
        self._round                  = 0
        self._lives                  = self.MAX_LIVES
        self._misses                 = 0
        self._score                  = 0
        self._user_choice            = None
        self._completed_this_round   = []
        self._flash_color            = None
        self._flash_until            = 0.0
        self._game_over_sound_played = False
        self._winner                 = False
        self._go_choice              = None
        self._choice_event.clear()

    def _set_state(self, s):
        with self._lock:
            self._state = s
        # Musica de fondo: solo en la intro y (despues del sonido) en game over.
        # Durante cuenta regresiva, secuencia, turno y feedback NO suena.
        if s not in (State.INTRO, State.GAME_OVER):
            self._music.stop(fade_ms=500)

    def _publish_idle(self):
        if self._mode_pub:
            try:
                self._mode_pub.publish(String(data='idle'))
            except Exception:
                pass

    def _show_countdown(self):
        for n in ["3", "2", "1", "GO!" if self.is_english else "YA!"]:
            if not self._active or self._state == State.GAME_OVER:
                return
            frame = _sky()
            draw_text(frame, n, 100, 24, scale=6, top=GOLD_T, bot=GOLD_B, center=True)
            sub = "GET READY" if self.is_english else "PREPARATE"
            draw_text(frame, sub, 100, 82, scale=2, top=WHITE, center=True)
            self._draw_hud(frame)
            self._push(frame)
            time.sleep(0.8)

    def _play_sequence(self, on_time: float, off_time: float):
        if not self._active or self._state == State.GAME_OVER:
            return
        self._push(self._make_ball_frame(lit=None))
        time.sleep(0.4)
        for color in self._sequence:
            if not self._active or self._state == State.GAME_OVER:
                return
            self._push(self._make_ball_frame(lit=color))
            time.sleep(on_time)
            self._push(self._make_ball_frame(lit=None))
            time.sleep(off_time)
        time.sleep(0.2)

    def _wait_for_input(self, step_idx: int, total_steps: int) -> str:
        self._user_choice = None
        self._choice_event.clear()
        deadline = time.time() + INPUT_TIMEOUT

        while self._active and self._state != State.GAME_OVER and rclpy.ok():
            remaining = max(0.0, deadline - time.time())
            if self._flash_color and time.time() > self._flash_until:
                self._flash_color = None
            self._push(self._make_input_frame(
                remaining, step_idx, total_steps,
                completed=self._completed_this_round,
                flash_color=self._flash_color,
            ))
            if self._choice_event.wait(timeout=0.04):
                return self._user_choice or ""
            if time.time() >= deadline:
                return ""
        return ""

    def _show_feedback(self, correct: bool, expected: str):
        en = self.is_english
        frame = _sky()
        if correct:
            symbol, msg = "OK", ("CORRECT!" if en else "CORRECTO!")
            top, bot = GREEN_T, GREEN_B
        else:
            symbol, msg = "ERROR", (f"WAS: {expected}" if en else f"ERA: {expected}")
            top, bot = RED_T, RED_B
        draw_text(frame, symbol, 100, 24, scale=6, top=top, bot=bot, center=True)
        draw_text(frame, msg, 100, 76, scale=2, top=WHITE, center=True)
        self._draw_hud(frame)
        self._push(frame)
        time.sleep(1.4)

    def _sfx_length(self, key):
        """Duracion (s) de un efecto de sonido; 0 si no esta cargado."""
        try:
            return float(self._sound._cache[key].get_length())
        except Exception:
            return 0.0

    def _show_game_over(self):
        en = self.is_english
        while (self._active and self._state == State.GAME_OVER
               and self._go_choice is None and rclpy.ok()):
            if not self._game_over_sound_played:
                self._sound.play("gameover")
                self._game_over_sound_played = True
                # Sonic entra cuando termina el sonido de game over
                self._music_resume_at = time.time() + self._sfx_length("gameover") + 0.4
            elif getattr(self, '_music_resume_at', 0.0) and time.time() >= self._music_resume_at:
                self._music_resume_at = 0.0
                if self._active and self._state == State.GAME_OVER:
                    self._music.play()

            frame = _sky()
            is_winner = getattr(self, '_winner', False)

            # Lógica de color de los corazones: Rojos si gana, gris si pierde
            heart_color = HEART_RED if is_winner else HEART_GRY

            n, hw, gap = self.MAX_LIVES, 22, 6
            x0 = 100 - (n * hw + (n - 1) * gap) // 2
            for i in range(n):
                _draw_heart(frame, x0 + i * (hw + gap), 8, custom_color=heart_color, scale=2)

            if is_winner:
                go_txt = "YOU WIN!" if en else "GANASTE!"
                top, bot = GREEN_T, GREEN_B
            else:
                go_txt = "GAME OVER"
                top, bot = GOLD_T, GOLD_B
            
            draw_text(frame, go_txt, 100, 36, scale=3, top=top, bot=bot, center=True)

            sc = (f"ROUND: {self._round}   HITS: {self._score}" if en else
                  f"RONDA: {self._round}   ACIERTOS: {self._score}")
            ms = (f"MISSES: {self._misses}" if en else
                  f"INTENTOS FALLIDOS: {self._misses}")
            
            # Se aumentó la separación (Y) y se ajustó el tamaño de "Intentos fallidos"
            draw_text(frame, sc, 100, 62, scale=1, top=WHITE, center=True)
            draw_text(frame, ms, 100, 75, scale=1, top=(210, 200, 230), center=True) 

            _draw_button(frame, *_L_RETRY, "TRY AGAIN" if en else "REINTENTAR", **BTN_START)
            _draw_button(frame, *_L_EXIT,  "EXIT" if en else "SALIR",           **BTN_BACK)

            self._push(frame)
            time.sleep(0.05)

    # ─────────────────────────────────────────────────────────────────────────
    #  RENDER (lienzo lógico 200x120)
    # ─────────────────────────────────────────────────────────────────────────

    def _make_ball_frame(self, lit) -> np.ndarray:
        """Pantalla de 'observa': fondo plano oscuro, sin nubes ni pasto ni
        texto extra, para que lo único que llame la atención sean los colores."""
        en = self.is_english
        frame = _dark_bg()

        if self._round >= 4:
            idx = min(self._round - 1, MAX_ROUNDS - 1)
            if idx >= 6:
                lbl, col = ("INSANE" if en else "IMPOSIBLE"), INSANE_C
            else:
                lbl, col = ("HARD" if en else "DIFICIL"), HARD_C
            draw_text(frame, lbl, 100, 5, top=_mul(col, 0.8), outline=None,
                      shadow=False, center=True)

        for name, c in BALL_POSITIONS.items():
            is_lit = (name == lit)
            _draw_ball(frame, c, BALL_R, COLORS[name], lit=is_lit, glow=is_lit)
            draw_text(frame, name, c[0], c[1] + BALL_R + 4,
                      top=WHITE if is_lit else (130, 110, 100),
                      outline=None, shadow=False, center=True)

        self._draw_hud(frame)
        return frame

    def _make_input_frame(self, remaining: float, step_idx: int, total_steps: int,
                          completed=None, flash_color=None) -> np.ndarray:
        if completed is None:
            completed = []

        frame = np.zeros((LH, LW, 3), np.uint8)
        frame[:] = BG_NIGHT

        for name, ((x1, y1), (x2, y2)) in QUADRANT_CORNERS.items():
            col = COLORS[name]
            lx1, ly1, lx2, ly2 = x1 // S, y1 // S, x2 // S, y2 // S
            fill = tuple(int(b * 0.72 + v * 0.28) for b, v in zip(BG_NIGHT, col))
            cv2.rectangle(frame, (lx1, ly1), (lx2 - 1, ly2 - 1), fill, -1)
            cv2.rectangle(frame, (lx1 + 2, ly1 + 2), (lx2 - 3, ly2 - 3), _mul(col, 0.5), 1)

            c = BALL_POSITIONS[name]
            _draw_ball(frame, c, BALL_R, col, lit=True, glow=(name == flash_color))
            if name == flash_color and name in completed:
                cx, cy = c
                cv2.line(frame, (cx - 6, cy), (cx - 2, cy + 4), WHITE, 2)
                cv2.line(frame, (cx - 2, cy + 4), (cx + 6, cy - 5), WHITE, 2)
            draw_text(frame, name, c[0], c[1] + BALL_R + 4, top=WHITE, center=True)

        cv2.line(frame, (LW // 2, 0), (LW // 2, LH), (20, 12, 10), 2)
        cv2.line(frame, (0, LH // 2), (LW, LH // 2), (20, 12, 10), 2)

        # Contador de paso
        instr = f"COLOR {step_idx}/{total_steps}"
        iw = text_width(instr, 1) // S
        _pixel_panel(frame, 100 - iw // 2 - 5, 2, 100 + iw // 2 + 5, 15,
                     (70, 45, 30), (150, 110, 80))
        draw_text(frame, instr, 100, 5, top=WHITE, outline=None, shadow=False, center=True)

        # Barra de tiempo
        ratio = remaining / INPUT_TIMEOUT
        bar_w = int(ratio * LW)
        bar_c = ((80, 210, 90) if ratio > 0.5 else
                 (50, 185, 230) if ratio > 0.25 else (70, 70, 220))
        cv2.rectangle(frame, (0, 100), (LW, 102), (30, 18, 12), -1)
        if bar_w > 0:
            cv2.rectangle(frame, (0, 100), (bar_w, 102), bar_c, -1)
            cv2.line(frame, (0, 100), (bar_w, 100), _mul(bar_c, 1.4), 1)

        # Slots de secuencia
        _pixel_panel(frame, 20, 105, 180, 119, (50, 30, 22), (140, 100, 80))
        n = total_steps
        sz = min(9, max(5, 150 // max(n, 1) - 2))
        gap = 2
        sx = 100 - (n * sz + (n - 1) * gap) // 2
        sy = 112 - sz // 2
        for i in range(n):
            x = sx + i * (sz + gap)
            if i < len(completed):
                col = COLORS.get(completed[i], (150, 150, 150))
                cv2.rectangle(frame, (x, sy), (x + sz - 1, sy + sz - 1), col, -1)
                cv2.rectangle(frame, (x, sy), (x + sz - 1, sy + sz - 1), _mul(col, 1.4), 1)
                cv2.rectangle(frame, (x + sz // 2, sy + sz // 2),
                              (x + sz // 2, sy + sz // 2), WHITE, -1)
            elif i == len(completed):
                cv2.rectangle(frame, (x, sy), (x + sz - 1, sy + sz - 1), (55, 38, 30), -1)
                cv2.rectangle(frame, (x, sy), (x + sz - 1, sy + sz - 1), GOLD_B, 1)
                if flash_color:
                    fc = _mul(COLORS.get(flash_color, GOLD_B), 0.5)
                    cv2.rectangle(frame, (x + 2, sy + 2),
                                  (x + sz - 3, sy + sz - 3), fc, -1)
            else:
                cv2.rectangle(frame, (x, sy), (x + sz - 1, sy + sz - 1), (45, 30, 24), -1)
                cv2.rectangle(frame, (x, sy), (x + sz - 1, sy + sz - 1), (95, 70, 55), 1)

        self._draw_hud(frame)
        return frame

    def _draw_hud(self, frame: np.ndarray):
        if self._state in (State.GAME_OVER, State.INTRO):
            return
        # Botón salir (zona de clic real: x<60, y<60)
        _pixel_panel(frame, 1, 1, 14, 14, (50, 50, 200), (30, 20, 90), hi=(120, 120, 255))
        draw_text(frame, "X", 8, 4, top=WHITE, outline=None, shadow=False, center=True)

        # Vidas
        for i in range(self.MAX_LIVES):
            # custom_color None para que use color predeterminado rojo (o gris si no tiene la vida)
            _draw_heart(frame, 18 + i * 13, 3, custom_color=HEART_RED if i < self._lives else HEART_GRY)

        # Ronda
        r_txt = f"R{self._round}"
        w = text_width(r_txt, 2) // S
        draw_text(frame, r_txt, LW - 24 - w, 3, scale=2, top=GOLD_T, bot=GOLD_B)

    def _push(self, frame: np.ndarray):
        up = cv2.resize(frame, (W, H), interpolation=cv2.INTER_NEAREST)
        for it in _TEXT_Q.pop(id(frame), []):
            _draw_text_real(up, *it)
        if len(_TEXT_Q) > 40:
            _TEXT_Q.clear()
        with self._frame_lock:
            self._frame = up


# =============================================================================
#  MAIN
# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = MemoriaNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.run_display_main_thread()
    except KeyboardInterrupt:
        print("Interrupcion recibida, cerrando...")
    finally:
        node._active = False
        node._music.stop()
        node._sound.quit()
        cv2.destroyAllWindows()

        print("MemoriaNode en estado INACTIVE, listo para reactivar")

        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()