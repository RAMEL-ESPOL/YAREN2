#!/usr/bin/env python3
"""
memoria_node.py  –  Juego de Memoria tipo Simon Says para YAREN2
Paquete: yaren_juegos

Fixes aplicados:
  · SoundManager usa pygame.mixer.music para MP3 (o Sound para WAV si existen)
  · on_deactivate sin join() bloqueante → segunda activación no falla
  · on_activate resetea eventos y estado antes de arrancar el thread
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

# ─── Paleta BGR ───────────────────────────────────────────────────────────────
COLORS = {
    "ROJO":     (50,  50,  220),
    "AZUL":     (220, 100,  50),
    "VERDE":    (50,  200,  80),
    "AMARILLO": (0,   220, 220),
}
COLOR_NAMES = list(COLORS.keys())
COLOR_DIM   = {k: tuple(int(c * 0.25) for c in v) for k, v in COLORS.items()}

QUADRANT_CORNERS = {
    "ROJO":     ((0,   0),   (400, 240)),
    "AZUL":     ((400, 0),   (800, 240)),
    "VERDE":    ((0,   240), (400, 480)),
    "AMARILLO": ((400, 240), (800, 480)),
}

BALL_POSITIONS = {
    "ROJO":     (200, 140),
    "AZUL":     (600, 140),
    "VERDE":    (200, 340),
    "AMARILLO": (600, 340),
}

W, H = 800, 480
BALL_RADIUS = 60

# Velocidad por ronda
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

# ─── Paleta UI YAREN ──────────────────────────────────────────────────────────
BG_DARK    = (18, 12, 28)
BG_PANEL   = (30, 22, 46)
ACCENT     = (230, 140, 60)
ACCENT2    = (200, 90,  40)
TEXT_LIGHT = (240, 235, 255)
TEXT_DIM   = (160, 145, 190)
DIVIDER    = (60,  45,  85)
HEART_COLOR= (90,  70,  220)
CORRECT_CLR= (90,  210, 80)
WRONG_CLR  = (70,  70,  220)

# ─── Audio ────────────────────────────────────────────────────────────────────
HOME_DIR   = os.path.expanduser("~")
BASE_WS    = os.path.join(HOME_DIR, "robotis_ws", "src", "YAREN2")
AUDIO_BASE = os.path.join(BASE_WS, "yaren_juegos", "sounds")
# Prioridad: WAV > MP3 (pygame.mixer.Sound solo soporta WAV de forma fiable)
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

# =============================================================================
#  SISTEMA DE SONIDO — pygame con soporte WAV (Sound) y MP3 (music channel)
# =============================================================================
class _NullSound:
    def play(self, key: str): pass
    def quit(self): pass


class SoundManager:
    """
    Usa pygame.mixer.Sound para WAV (baja latencia, solapable).
    Si solo hay MP3, cae a pygame.mixer.music (no solapable pero funciona).
    """
    def __init__(self):
        self._ok    = False
        self._cache = {}          # key → pygame.mixer.Sound  (WAV)
        self._mp3   = {}          # key → path str            (MP3 fallback)
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
        for key, path in SOUNDS.items():
            if not path:
                print(f"[SoundManager] Archivo no encontrado para: {key}")
                continue
            if path.lower().endswith(".wav"):
                try:
                    self._cache[key] = self._pygame.mixer.Sound(path)
                    print(f"[SoundManager] WAV cargado: {key}")
                except Exception as e:
                    print(f"[SoundManager] Error cargando WAV {key}: {e}")
            else:
                # MP3 → guardar ruta para reproducir con music
                self._mp3[key] = path
                print(f"[SoundManager] MP3 registrado: {key} → {path}")

    def play(self, key: str):
        if not self._ok:
            return
        # Preferir WAV (Sound, solapable, baja latencia)
        snd = self._cache.get(key)
        if snd:
            try:
                snd.play()
                return
            except Exception:
                pass
        # Fallback MP3 vía music channel
        mp3_path = self._mp3.get(key)
        if mp3_path:
            try:
                self._pygame.mixer.music.load(mp3_path)
                self._pygame.mixer.music.play()
            except Exception as e:
                print(f"[SoundManager] Error reproduciendo MP3 {key}: {e}")

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
#  HELPERS DE DIBUJO
# =============================================================================

def _draw_rounded_rect(img, pt1, pt2, color, radius=12, thickness=-1, alpha=1.0):
    x1, y1 = pt1
    x2, y2 = pt2
    if thickness == -1:
        overlay = img.copy()
        cv2.rectangle(overlay, (x1 + radius, y1), (x2 - radius, y2), color, -1)
        cv2.rectangle(overlay, (x1, y1 + radius), (x2, y2 - radius), color, -1)
        for cx, cy in [(x1+radius, y1+radius), (x2-radius, y1+radius),
                       (x1+radius, y2-radius), (x2-radius, y2-radius)]:
            cv2.circle(overlay, (cx, cy), radius, color, -1)
        if alpha < 1.0:
            cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)
        else:
            np.copyto(img, overlay)
    else:
        cv2.rectangle(img, (x1 + radius, y1), (x2 - radius, y1), color, thickness)
        cv2.rectangle(img, (x1 + radius, y2), (x2 - radius, y2), color, thickness)
        cv2.rectangle(img, (x1, y1 + radius), (x1, y2 - radius), color, thickness)
        cv2.rectangle(img, (x2, y1 + radius), (x2, y2 - radius), color, thickness)
        cv2.ellipse(img, (x1+radius, y1+radius), (radius, radius), 180, 0, 90, color, thickness)
        cv2.ellipse(img, (x2-radius, y1+radius), (radius, radius), 270, 0, 90, color, thickness)
        cv2.ellipse(img, (x1+radius, y2-radius), (radius, radius),  90, 0, 90, color, thickness)
        cv2.ellipse(img, (x2-radius, y2-radius), (radius, radius),   0, 0, 90, color, thickness)

def _draw_button(frame, pt1, pt2, label, font_scale=1.0, thickness=2,
                 bg_color=None, border_color=None, text_color=None,
                 radius=12, alpha=0.95):
    if bg_color is None:
        bg_color = ACCENT2
    if border_color is None:
        border_color = ACCENT
    if text_color is None:
        text_color = TEXT_LIGHT

    x1, y1 = pt1
    x2, y2 = pt2

    _draw_rounded_rect(frame, pt1, pt2, bg_color, radius=radius,
                       thickness=-1, alpha=alpha)
    _draw_rounded_rect(frame, pt1, pt2, border_color, radius=radius,
                       thickness=2)

    font = cv2.FONT_HERSHEY_DUPLEX
    (tw, th), baseline = cv2.getTextSize(label, font, font_scale, thickness)
    tx = (x1 + x2) // 2 - tw // 2
    ty = (y1 + y2) // 2 + th // 2 - baseline // 2
    cv2.putText(frame, label, (tx, ty), font, font_scale,
                text_color, thickness, cv2.LINE_AA)

def _draw_glow_circle(frame, center, radius, color, intensity=0.45):
    overlay = frame.copy()
    for r_off, a in [(radius + 22, 0.08), (radius + 14, 0.15), (radius + 7, 0.25)]:
        cv2.circle(overlay, center, r_off, color, -1)
    cv2.addWeighted(overlay, intensity, frame, 1 - intensity, 0, frame)
    cv2.circle(frame, center, radius, color, -1)

def _gradient_bg(h, w, top_color, bot_color):
    frame = np.zeros((h, w, 3), dtype=np.float32)
    for i in range(h):
        t = i / h
        frame[i] = [top_color[c] * (1 - t) + bot_color[c] * t for c in range(3)]
    return frame.astype(np.uint8)

def _draw_checkmark(frame, center, size, color, thickness=4):
    cx, cy = center
    p1 = (cx - int(size * 0.5), cy)
    p2 = (cx - int(size * 0.12), cy + int(size * 0.45))
    p3 = (cx + int(size * 0.55), cy - int(size * 0.45))
    cv2.line(frame, p1, p2, color, thickness, cv2.LINE_AA)
    cv2.line(frame, p2, p3, color, thickness, cv2.LINE_AA)

def _draw_sequence_slots(frame, sequence_len: int, completed: list,
                         current_flash=None):
    if sequence_len == 0:
        return

    slot_size = min(28, max(14, int(560 / sequence_len) - 6))
    gap       = 5
    total_w   = sequence_len * slot_size + (sequence_len - 1) * gap
    start_x   = W // 2 - total_w // 2
    row_y     = H - 36
    radius    = slot_size // 2

    for i in range(sequence_len):
        cx = start_x + i * (slot_size + gap) + slot_size // 2
        cy = row_y

        if i < len(completed):
            color = COLORS.get(completed[i], (150, 150, 150))
            bright = tuple(min(255, int(c * 1.35)) for c in color)
            cv2.circle(frame, (cx, cy), radius, color, -1)
            cv2.circle(frame, (cx, cy), radius, bright, 2)
            _draw_checkmark(frame, (cx, cy), radius, (240, 235, 255), thickness=2)
        elif i == len(completed):
            cv2.circle(frame, (cx, cy), radius, (35, 28, 55), -1)
            cv2.circle(frame, (cx, cy), radius, ACCENT, 2)
            if current_flash:
                fc = COLORS.get(current_flash, ACCENT)
                cv2.circle(frame, (cx, cy), radius - 4,
                           tuple(int(c * 0.4) for c in fc), -1)
        else:
            cv2.circle(frame, (cx, cy), radius, (35, 28, 52), -1)
            cv2.circle(frame, (cx, cy), radius, DIVIDER, 1)

def _draw_color_dots_row(frame, cx_center, cy, radius=10, spacing=32):
    colors_list = list(COLORS.values())
    total = len(colors_list) * (radius * 2 + spacing) - spacing
    sx = cx_center - total // 2
    for i, col in enumerate(colors_list):
        x = sx + i * (radius * 2 + spacing) + radius
        bright = tuple(min(255, int(c * 1.2)) for c in col)
        _draw_glow_circle(frame, (x, cy), radius, col, intensity=0.4)
        cv2.circle(frame, (x, cy), radius, bright, 2)

# =============================================================================
#  ESTADOS
# =============================================================================
class State:
    INTRO         = "intro"
    IDLE          = "idle"
    COUNTDOWN     = "countdown"
    SHOW_SEQUENCE = "show_sequence"
    PLAYER_INPUT  = "player_input"
    FEEDBACK      = "feedback"
    GAME_OVER     = "game_over"

_BTN_W = 200
_BTN_Y1, _BTN_Y2 = H - 90, H - 32
_INTRO_BTN_START = ((W // 2 - _BTN_W - 12, _BTN_Y1), (W // 2 - 12, _BTN_Y2))
_INTRO_BTN_BACK  = ((W // 2 + 12, _BTN_Y1), (W // 2 + _BTN_W + 12, _BTN_Y2))

_GO_BTN = ((W // 2 - 120, H - 110), (W // 2 + 120, H - 42))

# =============================================================================
#  NODO PRINCIPAL
# =============================================================================
class MemoriaNode(LifecycleNode):

    MAX_LIVES = 3

    def __init__(self):
        super().__init__('memoria_node')
        self._window          = "YAREN2 - Juego de Memoria"
        self._active          = False
        self._lock            = threading.Lock()
        self._game_thread     = None
        self.is_english       = False

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

        # FIX: resetear TODOS los eventos y estado antes de arrancar el thread
        # para que la segunda activación empiece limpia
        self._active = True
        self._choice_event.clear()
        self._intro_event.clear()
        self._intro_choice = None
        self._user_choice  = None
        self._frame        = None
        self._set_state(State.INTRO)

        self._game_thread = threading.Thread(target=self._game_loop, daemon=True)
        self._game_thread.start()

        return super().on_activate(state)

    def on_deactivate(self, state):
        self.get_logger().info('MemoriaNode: INACTIVO')
        self._active = False
        self._choice_event.set()
        self._intro_event.set()
        self._game_thread = None
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        if self._mode_pub:   self.destroy_publisher(self._mode_pub)
        if self._lang_sub:   self.destroy_subscription(self._lang_sub)
        self._sound.quit()
        cv2.destroyAllWindows()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self._active = False
        self._choice_event.set()
        self._intro_event.set()
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

        if state == State.INTRO:
            (sx1, sy1), (sx2, sy2) = _INTRO_BTN_START
            (bx1, by1), (bx2, by2) = _INTRO_BTN_BACK
            if sx1 <= x <= sx2 and sy1 <= y <= sy2:
                self._sound.play("acierto")
                self._intro_choice = "start"
                self._intro_event.set()
            elif bx1 <= x <= bx2 and by1 <= y <= by2:
                self._sound.play("acierto")
                self._intro_choice = "back"
                self._intro_event.set()
            return

        if state == State.GAME_OVER:
            (bx1, by1), (bx2, by2) = _GO_BTN
            if bx1 <= x <= bx2 and by1 <= y <= by2:
                self._sound.play("acierto")
                self._publish_idle()
                self._active = False
                self._choice_event.set()
            return

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
        
        while rclpy.ok():
            # Esperar activación
            while not self._active and rclpy.ok():
                time.sleep(0.05)
            
            if not rclpy.ok():
                break
            
            # Crear ventana
            cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(win_name, W, H)
            cv2.setWindowProperty(win_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            cv2.setWindowProperty(win_name, cv2.WND_PROP_TOPMOST, 1)
            cv2.setMouseCallback(win_name, self._on_click)
            
            # Resetear estado del juego
            self._reset_game()
            self._set_state(State.INTRO)
            self._choice_event.clear()
            self._intro_event.clear()
            
            # ✅ Crear frame dentro del bucle
            frame = np.zeros((H, W, 3), dtype=np.uint8)
            
            # Bucle del juego (se ejecuta mientras esté activo)
            while self._active and rclpy.ok():
                # ✅ Verificar si la ventana sigue abierta
                try:
                    if cv2.getWindowProperty(win_name, cv2.WND_PROP_AUTOSIZE) == -1:
                        self._active = False
                        break
                except Exception:
                    self._active = False
                    break
                
                # ✅ Aquí va la lógica del juego (tu código existente)
                # Por ejemplo, obtener el frame actual
                with self._frame_lock:
                    if self._frame is not None:
                        frame = self._frame.copy()
                    else:
                        frame[:] = BG_DARK  # Fondo por defecto
                
                cv2.imshow(win_name, frame)
                key = cv2.waitKey(16) & 0xFF
                if key == 27:  # ESC
                    self._active = False
                    break
            
            # Limpiar cuando se desactiva
            try:
                cv2.destroyAllWindows()
                cv2.waitKey(1)
            except Exception:
                pass
            
            # ✅ NO salir - volver al bucle externo para esperar reactivación
            print("🔄 MemoriaNode desactivado, esperando reactivación...")
            time.sleep(0.2)
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

                self._sound.play("acierto")
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

        if self._state != State.GAME_OVER and self._round >= MAX_ROUNDS:
            self._set_state(State.GAME_OVER)
            self._winner = True

        if self._state == State.GAME_OVER:
            self._show_game_over()

        if not self._active:
            self._publish_idle()

    # ─────────────────────────────────────────────────────────────────────────
    #  PANTALLA INTRO
    # ─────────────────────────────────────────────────────────────────────────

    def _show_intro_screen(self):
        self._intro_choice = None
        self._intro_event.clear()

        anim_t = 0.0
        while self._active and not self._intro_event.is_set() and rclpy.ok():
            frame = self._make_intro_frame(anim_t)
            self._push(frame)
            time.sleep(0.033)
            anim_t += 0.033

    def _make_intro_frame(self, t: float) -> np.ndarray:
        frame = _gradient_bg(H, W, BG_DARK, (28, 18, 45))

        font_d = cv2.FONT_HERSHEY_DUPLEX
        font_s = cv2.FONT_HERSHEY_SIMPLEX

        px1, py1 = 30, 18
        px2, py2 = W - 30, H - 18
        _draw_rounded_rect(frame, (px1, py1), (px2, py2), BG_PANEL,
                           radius=20, alpha=0.97)
        _draw_rounded_rect(frame, (px1, py1), (px2, py2), DIVIDER,
                           radius=20, thickness=2)

        title = "MEMORY GAME" if self.is_english else "JUEGO DE MEMORIA"
        (tw, th), _ = cv2.getTextSize(title, font_d, 1.55, 3)
        tx = W // 2 - tw // 2
        cv2.putText(frame, title, (tx + 2, 66), font_d, 1.55,
                    (10, 6, 20), 3, cv2.LINE_AA)
        cv2.putText(frame, title, (tx, 64), font_d, 1.55,
                    ACCENT, 3, cv2.LINE_AA)

        cv2.line(frame, (px1 + 40, 84), (px2 - 40, 84), DIVIDER, 1)

        _draw_color_dots_row(frame, W // 2, 118, radius=11, spacing=36)

        if self.is_english:
            lines = [
                ("Memorize the sequence of colored balls", False),
                ("and repeat it in the same order.", False),
                ("", False),
                (f"10 rounds  |  3 lives  |  Speed increases each round", True),
            ]
        else:
            lines = [
                ("Memoriza la secuencia de colores", False),
                ("y repitela en el mismo orden.", False),
                ("", False),
                (f"10 rondas  |  3 vidas  |  La velocidad aumenta cada ronda", True),
            ]

        y_text = 168
        for line, highlight in lines:
            if line == "":
                y_text += 14
                continue
            scale = 0.60
            (lw, lh), _ = cv2.getTextSize(line, font_s, scale, 1)
            lx = W // 2 - lw // 2
            col = ACCENT if highlight else TEXT_DIM
            thick = 2 if highlight else 1
            cv2.putText(frame, line, (lx, y_text), font_s, scale,
                        col, thick, cv2.LINE_AA)
            y_text += lh + 16

        diff_y = 278
        cv2.line(frame, (px1 + 40, diff_y - 14), (px2 - 40, diff_y - 14), DIVIDER, 1)

        diffs = [
            ("1-3", "FACIL"    if not self.is_english else "EASY",   CORRECT_CLR),
            ("4-6", "DIFICIL"  if not self.is_english else "HARD",   (50, 185, 230)),
            ("7-10","IMPOSIBLE"if not self.is_english else "INSANE",  WRONG_CLR),
        ]
        col_w = (px2 - px1 - 60) // 3
        for i, (rng, lbl, col) in enumerate(diffs):
            cx = px1 + 30 + col_w * i + col_w // 2
            (rw, _), _ = cv2.getTextSize(rng, font_d, 0.85, 2)
            cv2.putText(frame, rng, (cx - rw // 2, diff_y + 14),
                        font_d, 0.85, col, 2, cv2.LINE_AA)
            (lw, _), _ = cv2.getTextSize(lbl, font_s, 0.52, 1)
            cv2.putText(frame, lbl, (cx - lw // 2, diff_y + 34),
                        font_s, 0.52, tuple(int(c * 0.7) for c in col), 1, cv2.LINE_AA)
            if i < 2:
                cv2.line(frame, (px1 + 30 + col_w * (i + 1), diff_y - 6),
                         (px1 + 30 + col_w * (i + 1), diff_y + 40), DIVIDER, 1)

        btn_y1 = H - 90
        btn_y2 = H - 32
        btn_w  = 200

        sx1 = W // 2 - btn_w - 12
        sx2 = W // 2 - 12
        bx1 = W // 2 + 12
        bx2 = W // 2 + btn_w + 12

        start_lbl = "START"  if self.is_english else "EMPEZAR"
        back_lbl  = "BACK"   if self.is_english else "VOLVER"

        _draw_button(frame, (sx1, btn_y1), (sx2, btn_y2), start_lbl,
                     font_scale=1.05, thickness=2,
                     bg_color=ACCENT2, border_color=ACCENT,
                     text_color=TEXT_LIGHT, radius=14)

        _draw_button(frame, (bx1, btn_y1), (bx2, btn_y2), back_lbl,
                     font_scale=1.05, thickness=2,
                     bg_color=(40, 30, 60), border_color=DIVIDER,
                     text_color=TEXT_DIM, radius=14)

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
        self._choice_event.clear()

    def _set_state(self, s):
        with self._lock:
            self._state = s

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
            frame = self._dark_bg()
            cx, cy = W // 2, H // 2
            cv2.circle(frame, (cx, cy), 90, DIVIDER, -1)
            cv2.circle(frame, (cx, cy), 86, BG_PANEL, -1)
            self._draw_centered(frame, n, scale=4.5, thickness=7,
                                color=ACCENT, shadow_color=(20, 10, 40))
            sub = "GET READY" if self.is_english else "PREPARATE"
            self._draw_centered(frame, sub, scale=0.85, thickness=2,
                                color=TEXT_DIM, y_offset=120)
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
        frame = self._dark_bg()
        if correct:
            symbol     = "OK"
            msg        = "CORRECT!" if self.is_english else "CORRECTO!"
            main_color = CORRECT_CLR
            cv2.circle(frame, (W//2, H//2 - 30), 110,
                       tuple(int(c * 0.18) for c in CORRECT_CLR), -1)
        else:
            symbol     = "ERROR"
            msg        = f"Was: {expected}" if self.is_english else f"Era: {expected}"
            main_color = WRONG_CLR
            cv2.circle(frame, (W//2, H//2 - 30), 110,
                       tuple(int(c * 0.18) for c in WRONG_CLR), -1)
        self._draw_centered(frame, symbol, scale=5.5, thickness=8,
                            color=main_color, shadow_color=(10, 5, 20), y_offset=-55)
        self._draw_centered(frame, msg, scale=1.5, thickness=2,
                            color=TEXT_LIGHT, y_offset=65)
        self._draw_hud(frame)
        self._push(frame)
        time.sleep(1.4)

    def _show_game_over(self):
        while self._active and self._state == State.GAME_OVER and rclpy.ok():
            if not self._game_over_sound_played:
                self._sound.play("gameover")
                self._game_over_sound_played = True

            frame = self._dark_bg()
            px1, py1 = W//2 - 260, H//2 - 155
            px2, py2 = W//2 + 260, H//2 + 110
            _draw_rounded_rect(frame, (px1, py1), (px2, py2), BG_PANEL,
                               radius=18, alpha=0.92)

            if getattr(self, '_winner', False):
                go_txt = "YOU WIN!" if self.is_english else "GANASTE!"
                go_col = CORRECT_CLR
            else:
                go_txt = "GAME OVER"
                go_col = WRONG_CLR
            sc_txt   = (f"Round: {self._round}    Hits: {self._score}"
                        if self.is_english else
                        f"Ronda: {self._round}    Aciertos: {self._score}")
            miss_txt = (f"Misses: {self._misses}"
                        if self.is_english else
                        f"Intentos fallidos: {self._misses}")

            self._draw_centered(frame, go_txt, scale=2.8, thickness=5,
                                color=go_col, shadow_color=(10, 5, 20),
                                y_offset=-95)

            lx1, lx2 = W//2 - 180, W//2 + 180
            cv2.line(frame, (lx1, H//2 - 25), (lx2, H//2 - 25), (45, 35, 65), 1)

            self._draw_centered(frame, sc_txt,   scale=1.0, thickness=2,
                                color=TEXT_LIGHT, y_offset=10)
            self._draw_centered(frame, miss_txt, scale=0.95, thickness=2,
                                color=TEXT_DIM, y_offset=52)

            v_txt = "BACK" if self.is_english else "VOLVER"
            (bx1, by1), (bx2, by2) = _GO_BTN
            _draw_button(frame, (bx1, by1), (bx2, by2), v_txt,
                         font_scale=1.1, thickness=2,
                         bg_color=ACCENT2, border_color=ACCENT,
                         text_color=TEXT_LIGHT, radius=10)

            self._push(frame)
            time.sleep(0.05)

    # ─────────────────────────────────────────────────────────────────────────
    #  RENDER HELPERS
    # ─────────────────────────────────────────────────────────────────────────

    def _dark_bg(self) -> np.ndarray:
        return _gradient_bg(H, W, BG_DARK, (28, 18, 45))

    def _make_ball_frame(self, lit) -> np.ndarray:
        frame = self._dark_bg()
        r_txt = f"Round {self._round}" if self.is_english else f"Ronda {self._round}"
        (rw, rh), _ = cv2.getTextSize(r_txt, cv2.FONT_HERSHEY_DUPLEX, 0.85, 2)
        rx = W//2 - rw//2
        _draw_rounded_rect(frame, (rx - 12, 8), (rx + rw + 12, 38), BG_PANEL, radius=10)
        cv2.putText(frame, r_txt, (rx, 30),
                    cv2.FONT_HERSHEY_DUPLEX, 0.85, TEXT_DIM, 2, cv2.LINE_AA)

        if self._round >= 1:
            diff_labels = ["","","",
                           "DIFICIL","DIFICIL","DIFICIL",
                           "IMPOSIBLE","IMPOSIBLE","IMPOSIBLE","IMPOSIBLE"]
            diff_colors = [(90,210,80),(90,210,80),(90,210,80),
                           (50,185,230),(50,185,230),(50,185,230),
                           (70,70,220),(70,70,220),(70,70,220),(70,70,220)]
            idx = min(self._round - 1, MAX_ROUNDS - 1)
            dlbl = diff_labels[idx]
            dcol = diff_colors[idx]
            if dlbl:
                (dw, _), _ = cv2.getTextSize(dlbl, cv2.FONT_HERSHEY_SIMPLEX, 0.58, 2)
                cv2.putText(frame, dlbl, (W - dw - 8, 34),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.58, dcol, 2, cv2.LINE_AA)

        watch = "Watch carefully..." if self.is_english else "Observa con atencion..."
        (ww, _), _ = cv2.getTextSize(watch, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.putText(frame, watch, (W//2 - ww//2, H - 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, TEXT_DIM, 1, cv2.LINE_AA)

        for name, center in BALL_POSITIONS.items():
            if name == lit:
                color = COLORS[name]
                _draw_glow_circle(frame, center, BALL_RADIUS, color, intensity=0.5)
                bright = tuple(min(255, int(c * 1.4)) for c in color)
                cv2.circle(frame, center, BALL_RADIUS, bright, 3)
                rx2 = center[0] - BALL_RADIUS//3
                ry2 = center[1] - BALL_RADIUS//3
                cv2.circle(frame, (rx2, ry2), BALL_RADIUS//5, (255, 255, 255), -1)
                cv2.circle(frame, (rx2 + 10, ry2 + 8), BALL_RADIUS//10, (200, 200, 200), -1)
            else:
                color = COLOR_DIM[name]
                cv2.circle(frame, center, BALL_RADIUS, color, -1)
                dim_border = tuple(min(255, int(c * 1.8)) for c in color)
                cv2.circle(frame, center, BALL_RADIUS, dim_border, 1)

            (tw, _), _ = cv2.getTextSize(name, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
            label_color = TEXT_LIGHT if name == lit else TEXT_DIM
            cv2.putText(frame, name,
                        (center[0] - tw//2, center[1] + BALL_RADIUS + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, label_color, 2, cv2.LINE_AA)

        self._draw_hud(frame)
        return frame

    def _make_input_frame(self, remaining: float, step_idx: int, total_steps: int,
                          completed=None, flash_color=None) -> np.ndarray:
        if completed is None:
            completed = []

        frame = np.zeros((H, W, 3), dtype=np.uint8)
        frame[:] = BG_DARK

        for name, ((x1, y1), (x2, y2)) in QUADRANT_CORNERS.items():
            color      = COLORS[name]
            quad_color = tuple(int(c * 0.22) for c in color)
            overlay    = frame.copy()
            cv2.rectangle(overlay, (x1, y1), (x2 - 1, y2 - 1), quad_color, -1)
            cv2.addWeighted(overlay, 0.85, frame, 0.15, 0, frame)

            cx_q, cy_q = (x1 + x2) // 2, (y1 + y2) // 2

            if name == flash_color:
                _draw_glow_circle(frame, (cx_q, cy_q), 65, color, intensity=0.55)
                bright = tuple(min(255, int(c * 1.5)) for c in color)
                cv2.circle(frame, (cx_q, cy_q), 65, bright, 4)
                if name in completed:
                    _draw_checkmark(frame, (cx_q, cy_q), 46, (255, 255, 255), thickness=6)
            else:
                _draw_glow_circle(frame, (cx_q, cy_q), 55, color, intensity=0.35)
                bright = tuple(min(255, int(c * 1.3)) for c in color)
                cv2.circle(frame, (cx_q, cy_q), 55, bright, 3)

            cv2.circle(frame, (cx_q - 17, cy_q - 17), 13, (255, 255, 255), -1)
            cv2.circle(frame, (cx_q - 6,  cy_q - 6),   6, (200, 200, 200), -1)

            (tw, _), _ = cv2.getTextSize(name, cv2.FONT_HERSHEY_DUPLEX, 1.0, 2)
            cv2.putText(frame, name, (cx_q - tw//2, cy_q + 82),
                        cv2.FONT_HERSHEY_DUPLEX, 1.0, TEXT_LIGHT, 2, cv2.LINE_AA)

        cv2.line(frame, (W//2, 0), (W//2, H), (8, 6, 14), 5)
        cv2.line(frame, (0, H//2), (W, H//2), (8, 6, 14), 5)

        instr = f"Color {step_idx}/{total_steps}"
        (iw, ih), _ = cv2.getTextSize(instr, cv2.FONT_HERSHEY_SIMPLEX, 0.78, 2)
        ix = W//2 - iw//2
        _draw_rounded_rect(frame, (ix - 14, 5), (ix + iw + 14, 40),
                           (35, 28, 55), radius=10, alpha=0.90)
        _draw_rounded_rect(frame, (ix - 14, 5), (ix + iw + 14, 40),
                           DIVIDER, radius=10, thickness=1)
        cv2.putText(frame, instr, (ix, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.78, TEXT_LIGHT, 2, cv2.LINE_AA)

        slot_panel_h = 52
        _draw_rounded_rect(frame,
                           (W//2 - 300, H - slot_panel_h - 2),
                           (W//2 + 300, H - 2),
                           (25, 18, 42), radius=10, alpha=0.80)
        _draw_sequence_slots(frame, total_steps, completed, current_flash=flash_color)

        bar_y = H - slot_panel_h - 9
        bar_w = int((remaining / INPUT_TIMEOUT) * W)
        t_ratio = remaining / INPUT_TIMEOUT
        bar_color = ((80, 210, 90)  if t_ratio > 0.5 else
                     (50, 185, 230) if t_ratio > 0.25 else
                     (70, 70, 220))
        cv2.rectangle(frame, (0, bar_y), (bar_w, bar_y + 7), bar_color, -1)
        if bar_w > 8:
            cv2.rectangle(frame, (bar_w - 6, bar_y), (bar_w, bar_y + 7),
                          tuple(min(255, c + 60) for c in bar_color), -1)

        self._draw_hud(frame)
        return frame

    def _draw_hud(self, frame: np.ndarray):
        if self._state in (State.GAME_OVER, State.INTRO):
            return
        _draw_rounded_rect(frame, (4, 4), (54, 54), (38, 28, 58), radius=10)
        _draw_rounded_rect(frame, (4, 4), (54, 54), DIVIDER, radius=10, thickness=1)
        cv2.putText(frame, "X", (16, 39),
                    cv2.FONT_HERSHEY_DUPLEX, 1.1, (130, 100, 200), 2, cv2.LINE_AA)

        _draw_rounded_rect(frame, (62, 6), (240, 52), (38, 28, 58), radius=10)
        _draw_rounded_rect(frame, (62, 6), (240, 52), DIVIDER, radius=10, thickness=1)
        vida_lbl = "Lives" if self.is_english else "Vidas"
        cv2.putText(frame, vida_lbl + ":", (72, 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, TEXT_DIM, 1, cv2.LINE_AA)
        heart_x = 142
        for i in range(self.MAX_LIVES):
            hx = heart_x + i * 32
            hy = 29
            color = HEART_COLOR if i < self._lives else (50, 40, 65)
            cv2.circle(frame, (hx - 5, hy - 4), 7, color, -1)
            cv2.circle(frame, (hx + 5, hy - 4), 7, color, -1)
            pts = np.array([[hx - 11, hy - 1], [hx + 11, hy - 1],
                            [hx, hy + 9]], np.int32)
            cv2.fillPoly(frame, [pts], color)

        r_txt = f"R{self._round}"
        (rw, _), _ = cv2.getTextSize(r_txt, cv2.FONT_HERSHEY_SIMPLEX, 0.85, 2)
        _draw_rounded_rect(frame, (W - rw - 26, 6), (W - 6, 52),
                           (38, 28, 58), radius=10)
        _draw_rounded_rect(frame, (W - rw - 26, 6), (W - 6, 52),
                           DIVIDER, radius=10, thickness=1)
        cv2.putText(frame, r_txt, (W - rw - 13, 37),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, ACCENT, 2, cv2.LINE_AA)

    def _draw_centered(self, frame, text, scale=2.0, thickness=3,
                       color=(255, 255, 255), shadow_color=(0, 0, 0),
                       y_offset=0):
        font = cv2.FONT_HERSHEY_DUPLEX
        (tw, th), _ = cv2.getTextSize(text, font, scale, thickness)
        x = (W - tw) // 2
        y = H // 2 + th // 2 + y_offset
        cv2.putText(frame, text, (x + 3, y + 3), font, scale,
                    shadow_color, thickness + 3, cv2.LINE_AA)
        cv2.putText(frame, text, (x, y), font, scale, color, thickness, cv2.LINE_AA)

    def _push(self, frame: np.ndarray):
        with self._frame_lock:
            self._frame = frame


# =============================================================================
#  MAIN
# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = MemoriaNode()
    node.trigger_configure()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.run_display_main_thread()
    except KeyboardInterrupt:
        print("Interrupción recibida, cerrando...")
    finally:
        node._active = False
        node._sound.quit()
        cv2.destroyAllWindows()
        
        print("MemoriaNode en estado INACTIVE, listo para reactivar")
        
        spin_thread.join(timeout=2.0)
if __name__ == "__main__":
    main()