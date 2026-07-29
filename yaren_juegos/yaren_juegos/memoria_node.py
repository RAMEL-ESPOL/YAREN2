#!/usr/bin/env python3
"""
memoria_node.py  –  Juego de Memoria tipo Simon Says para YAREN2
Paquete: yaren_juegos
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

W, H         = 800, 480
BALL_RADIUS  = 60
BALL_ON_TIME = 0.7
BALL_OFF_TIME= 0.25
INPUT_TIMEOUT= 6.0

# ─── Paleta UI YAREN ──────────────────────────────────────────────────────────
BG_DARK      = (18, 12, 28)          # Fondo principal, morado muy oscuro
BG_PANEL     = (30, 22, 46)          # Panel secundario
ACCENT       = (230, 140, 60)        # Azul claro YAREN (BGR)
ACCENT2      = (200, 90,  40)        # Azul más profundo
TEXT_LIGHT   = (240, 235, 255)       # Blanco cálido
TEXT_DIM     = (160, 145, 190)       # Gris lavanda
DIVIDER      = (60,  45,  85)        # Línea separadora
HEART_COLOR  = (90,  70,  220)       # Rojo vidas (BGR)
CORRECT_CLR  = (90,  210, 80)        # Verde acierto
WRONG_CLR    = (70,  70,  220)       # Rojo error


def _draw_rounded_rect(img, pt1, pt2, color, radius=12, thickness=-1, alpha=1.0):
    """Rectángulo con esquinas redondeadas, con soporte de alpha."""
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
        cv2.rectangle(img, (x1 + radius, y1), (x2 - radius, y2), color, thickness)
        cv2.rectangle(img, (x1, y1 + radius), (x2, y2 - radius), color, thickness)
        for cx, cy in [(x1+radius, y1+radius), (x2-radius, y1+radius),
                       (x1+radius, y2-radius), (x2-radius, y2-radius)]:
            cv2.circle(img, (cx, cy), radius, color, thickness)


def _draw_glow_circle(frame, center, radius, color, intensity=0.45):
    """Círculo con halo difuso alrededor."""
    overlay = frame.copy()
    for r_off, a in [(radius + 22, 0.08), (radius + 14, 0.15), (radius + 7, 0.25)]:
        cv2.circle(overlay, center, r_off, color, -1)
    cv2.addWeighted(overlay, intensity, frame, 1 - intensity, 0, frame)
    cv2.circle(frame, center, radius, color, -1)


def _gradient_bg(h, w, top_color, bot_color):
    """Fondo con gradiente vertical suave."""
    frame = np.zeros((h, w, 3), dtype=np.float32)
    for i in range(h):
        t = i / h
        frame[i] = [top_color[c] * (1 - t) + bot_color[c] * t for c in range(3)]
    return frame.astype(np.uint8)


class State:
    IDLE         = "idle"
    COUNTDOWN    = "countdown"
    SHOW_SEQUENCE= "show_sequence"
    PLAYER_INPUT = "player_input"
    FEEDBACK     = "feedback"
    GAME_OVER    = "game_over"


class MemoriaNode(LifecycleNode):

    MAX_LIVES = 3

    def __init__(self):
        super().__init__('memoria_node')
        self._window     = "YAREN2 - Juego de Memoria"
        self._active     = False
        self._lock       = threading.Lock()
        self._game_thread= None
        self.is_english  = False

        # Shared state between game thread and display
        self._frame      = None
        self._frame_lock = threading.Lock()

        # Game state
        self._sequence: list[str] = []
        self._round    = 0
        self._lives    = self.MAX_LIVES
        self._misses   = 0
        self._score    = 0
        self._state    = State.IDLE

        # User input event
        self._user_choice: str | None = None
        self._choice_event = threading.Event()

        self._mode_pub = None
        self._lang_sub = None

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def on_configure(self, state):
        self.get_logger().info('MemoriaNode: configurando...')
        try:
            self._mode_pub = self.create_publisher(String, '/yaren_mode', 1)
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self._lang_sub = self.create_subscription(
                Bool, '/yaren/is_english', self._lang_cb, qos)
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'on_configure error: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        self.get_logger().info('MemoriaNode: ACTIVO')

        cv2.namedWindow(self._window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self._window, W, H)
        cv2.setWindowProperty(self._window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.setWindowProperty(self._window, cv2.WND_PROP_TOPMOST, 1)
        cv2.setMouseCallback(self._window, self._on_click)

        def _focus():
            time.sleep(0.4)
            os.system(f"xdotool search --sync --name '{self._window}' "
                      "windowactivate --sync windowraise 2>/dev/null")
        threading.Thread(target=_focus, daemon=True).start()

        self._active = True
        self._reset_game()

        self._game_thread = threading.Thread(target=self._game_loop, daemon=True)
        self._game_thread.start()

        self._display_loop()

        return super().on_activate(state)

    def on_deactivate(self, state):
        self.get_logger().info('MemoriaNode: INACTIVO')
        self._active = False
        self._choice_event.set()
        if self._game_thread:
            self._game_thread.join(timeout=3.0)
            self._game_thread = None
        cv2.destroyWindow(self._window)
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        if self._mode_pub:   self.destroy_publisher(self._mode_pub)
        if self._lang_sub:   self.destroy_subscription(self._lang_sub)
        cv2.destroyAllWindows()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self._active = False
        self._choice_event.set()
        cv2.destroyAllWindows()
        return TransitionCallbackReturn.SUCCESS

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _lang_cb(self, msg):
        self.is_english = msg.data

    def _on_click(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        with self._lock:
            state = self._state

        if state == State.GAME_OVER:
            bx1, by1 = W//2 - 120, H - 110
            bx2, by2 = W//2 + 120, H - 42
            if bx1 <= x <= bx2 and by1 <= y <= by2:
                self._publish_idle()   # Primero avisa a face_screen
                self._active = False   # Luego cierra el loop local
                self._choice_event.set()
            return

        if x < 60 and y < 60:
            if state != State.GAME_OVER:
                self._set_state(State.GAME_OVER)
                self._choice_event.set()
            return

        if state == State.PLAYER_INPUT:
            for color, ((x1, y1), (x2, y2)) in QUADRANT_CORNERS.items():
                if x1 <= x < x2 and y1 <= y < y2:
                    self._user_choice = color
                    self._choice_event.set()
                    return

    # ── Display loop (MAIN THREAD) ────────────────────────────────────────────

    def _display_loop(self):
        while self._active and rclpy.ok():
            with self._frame_lock:
                frame = self._frame.copy() if self._frame is not None else None

            if frame is not None:
                cv2.imshow(self._window, frame)

            key = cv2.waitKey(16)
            if key == 27:
                self._active = False
                self._choice_event.set()
                self._publish_idle()
                break

        self._active = False

    # ── Game logic (BACKGROUND THREAD) ────────────────────────────────────────

    def _game_loop(self):
        self._reset_game()
        self._set_state(State.COUNTDOWN)
        self._show_countdown()

        while self._active and self._state != State.GAME_OVER and rclpy.ok():
            self._sequence.append(random.choice(COLOR_NAMES))
            self._round += 1

            self._set_state(State.SHOW_SEQUENCE)
            self._play_sequence()
            if not self._active or self._state == State.GAME_OVER: break

            self._set_state(State.PLAYER_INPUT)
            round_passed = True
            total_steps = len(self._sequence)

            for i, expected in enumerate(self._sequence):
                choice = self._wait_for_input(step_idx=i+1, total_steps=total_steps)
                if not self._active or self._state == State.GAME_OVER:
                    break

                if choice != expected:
                    round_passed = False
                    self._lives -= 1
                    self._misses += 1
                    self._set_state(State.FEEDBACK)
                    self._show_feedback(False, expected)
                    break

                time.sleep(0.15)

            if not self._active or self._state == State.GAME_OVER: break

            if round_passed:
                self._set_state(State.FEEDBACK)
                self._show_feedback(True, "")

            if self._lives <= 0:
                self._set_state(State.GAME_OVER)
                break

        if self._state == State.GAME_OVER:
            self._show_game_over()

        if not self._active:
            self._publish_idle()

    # ── Game helpers ──────────────────────────────────────────────────────────

    def _reset_game(self):
        self._sequence   = []
        self._round      = 0
        self._lives      = self.MAX_LIVES
        self._misses     = 0
        self._score      = 0
        self._user_choice= None
        self._choice_event.clear()

    def _set_state(self, s):
        with self._lock:
            self._state = s

    def _publish_idle(self):
        if self._mode_pub:
            self._mode_pub.publish(String(data='idle'))

    def _show_countdown(self):
        for n in ["3", "2", "1", "GO!" if self.is_english else "YA!"]:
            if not self._active or self._state == State.GAME_OVER: return
            frame = self._dark_bg()

            # Círculo decorativo de fondo
            cx, cy = W // 2, H // 2
            cv2.circle(frame, (cx, cy), 90, DIVIDER, -1)
            cv2.circle(frame, (cx, cy), 86, BG_PANEL, -1)

            self._draw_centered(frame, n, scale=4.5, thickness=7,
                                color=ACCENT, shadow_color=(20, 10, 40))

            # Subtítulo parpadeante
            sub = "GET READY" if self.is_english else "PREPARATE"
            self._draw_centered(frame, sub, scale=0.85, thickness=2,
                                color=TEXT_DIM, y_offset=120)

            self._draw_hud(frame)
            self._push(frame)
            time.sleep(0.8)

    def _play_sequence(self):
        if not self._active or self._state == State.GAME_OVER: return
        self._push(self._make_ball_frame(lit=None))
        time.sleep(0.4)
        for color in self._sequence:
            if not self._active or self._state == State.GAME_OVER: return
            self._push(self._make_ball_frame(lit=color))
            time.sleep(BALL_ON_TIME)
            self._push(self._make_ball_frame(lit=None))
            time.sleep(BALL_OFF_TIME)
        time.sleep(0.2)

    def _wait_for_input(self, step_idx: int, total_steps: int) -> str:
        self._user_choice = None
        self._choice_event.clear()
        deadline = time.time() + INPUT_TIMEOUT

        while self._active and self._state != State.GAME_OVER and rclpy.ok():
            remaining = max(0.0, deadline - time.time())
            self._push(self._make_input_frame(remaining, step_idx, total_steps))

            if self._choice_event.wait(timeout=0.05):
                return self._user_choice or ""
            if time.time() >= deadline:
                return ""
        return ""

    def _show_feedback(self, correct: bool, expected: str):
        frame = self._dark_bg()

        if correct:
            self._score += 1
            symbol = "OK"
            msg    = "CORRECT!" if self.is_english else "CORRECTO!"
            main_color = CORRECT_CLR
            # Halo verde de fondo
            cv2.circle(frame, (W//2, H//2 - 30), 110,
                       tuple(int(c * 0.18) for c in CORRECT_CLR), -1)
        else:
            symbol = "ERROR"
            msg    = f"Was: {expected}" if self.is_english else f"Era: {expected}"
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
            frame = self._dark_bg()

            # Panel central
            px1, py1 = W//2 - 260, H//2 - 155
            px2, py2 = W//2 + 260, H//2 + 110
            _draw_rounded_rect(frame, (px1, py1), (px2, py2), BG_PANEL,
                               radius=18, alpha=0.92)

            go_txt = "GAME OVER"
            sc_txt = (f"Round: {self._round}    Hits: {self._score}"
                      if self.is_english else
                      f"Ronda: {self._round}    Aciertos: {self._score}")
            miss_txt = (f"Misses: {self._misses}"
                        if self.is_english else
                        f"Intentos fallidos: {self._misses}")

            self._draw_centered(frame, go_txt, scale=2.8, thickness=5,
                                color=WRONG_CLR, shadow_color=(10, 5, 20),
                                y_offset=-95)

            # Línea separadora muy sutil dentro del panel
            lx1, lx2 = W//2 - 180, W//2 + 180
            ly = H//2 - 25
            cv2.line(frame, (lx1, ly), (lx2, ly), (45, 35, 65), 1)

            self._draw_centered(frame, sc_txt, scale=1.0, thickness=2,
                                color=TEXT_LIGHT, y_offset=10)
            self._draw_centered(frame, miss_txt, scale=0.95, thickness=2,
                                color=TEXT_DIM, y_offset=52)

            # Botón Volver con diseño mejorado
            bx1, by1 = W//2 - 120, H - 110
            bx2, by2 = W//2 + 120, H - 42
            _draw_rounded_rect(frame, (bx1, by1), (bx2, by2),
                               ACCENT2, radius=10, alpha=1.0)
            _draw_rounded_rect(frame, (bx1, by1), (bx2, by2),
                               ACCENT, radius=10, thickness=2)

            v_txt = "BACK" if self.is_english else "VOLVER"
            (vw, vh), _ = cv2.getTextSize(v_txt, cv2.FONT_HERSHEY_DUPLEX, 1.1, 2)
            vx = W//2 - vw // 2
            vy = (by1 + by2) // 2 + vh // 2
            cv2.putText(frame, v_txt, (vx, vy),
                        cv2.FONT_HERSHEY_DUPLEX, 1.1, TEXT_LIGHT, 2, cv2.LINE_AA)

            self._push(frame)
            time.sleep(0.05)

    # ── Frame factories ───────────────────────────────────────────────────────

    def _dark_bg(self) -> np.ndarray:
        return _gradient_bg(H, W, BG_DARK, (28, 18, 45))

    def _make_ball_frame(self, lit: str | None) -> np.ndarray:
        frame = self._dark_bg()

        # Título de ronda centrado con fondo pill
        r_txt = f"Round {self._round}" if self.is_english else f"Ronda {self._round}"
        (rw, rh), _ = cv2.getTextSize(r_txt, cv2.FONT_HERSHEY_DUPLEX, 0.85, 2)
        rx = W//2 - rw//2
        _draw_rounded_rect(frame, (rx - 12, 8), (rx + rw + 12, 38),
                           BG_PANEL, radius=10)
        cv2.putText(frame, r_txt, (rx, 30),
                    cv2.FONT_HERSHEY_DUPLEX, 0.85, TEXT_DIM, 2, cv2.LINE_AA)

        # Instrucción sutil
        watch = "Watch carefully..." if self.is_english else "Observa con atencion..."
        (ww, _), _ = cv2.getTextSize(watch, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.putText(frame, watch, (W//2 - ww//2, H - 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, TEXT_DIM, 1, cv2.LINE_AA)

        for name, center in BALL_POSITIONS.items():
            if name == lit:
                color = COLORS[name]
                _draw_glow_circle(frame, center, BALL_RADIUS, color, intensity=0.5)
                # Borde brillante
                bright = tuple(min(255, int(c * 1.4)) for c in color)
                cv2.circle(frame, center, BALL_RADIUS, bright, 3)
                # Reflejo especular
                rx2 = center[0] - BALL_RADIUS//3
                ry2 = center[1] - BALL_RADIUS//3
                cv2.circle(frame, (rx2, ry2), BALL_RADIUS//5,
                           (255, 255, 255), -1)
                cv2.circle(frame, (rx2 + 10, ry2 + 8), BALL_RADIUS//10,
                           (200, 200, 200), -1)
            else:
                color = COLOR_DIM[name]
                cv2.circle(frame, center, BALL_RADIUS, color, -1)
                # Borde muy sutil en apagado
                dim_border = tuple(min(255, int(c * 1.8)) for c in color)
                cv2.circle(frame, center, BALL_RADIUS, dim_border, 1)

            # Etiqueta debajo de cada bola
            (tw, _), _ = cv2.getTextSize(name, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
            label_color = TEXT_LIGHT if name == lit else TEXT_DIM
            cv2.putText(frame, name,
                        (center[0] - tw//2, center[1] + BALL_RADIUS + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, label_color, 2, cv2.LINE_AA)

        self._draw_hud(frame)
        return frame

    def _make_input_frame(self, remaining: float, step_idx: int,
                          total_steps: int) -> np.ndarray:
        frame = np.zeros((H, W, 3), dtype=np.uint8)
        frame[:] = BG_DARK

        for name, ((x1, y1), (x2, y2)) in QUADRANT_CORNERS.items():
            color = COLORS[name]

            # Fondo del cuadrante con color suavizado
            quad_color = tuple(int(c * 0.22) for c in color)
            overlay = frame.copy()
            cv2.rectangle(overlay, (x1, y1), (x2 - 1, y2 - 1), quad_color, -1)
            cv2.addWeighted(overlay, 0.85, frame, 0.15, 0, frame)

            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

            # Círculo principal con glow
            _draw_glow_circle(frame, (cx, cy), 55, color, intensity=0.35)

            # Borde del círculo
            bright = tuple(min(255, int(c * 1.3)) for c in color)
            cv2.circle(frame, (cx, cy), 55, bright, 3)

            # Reflejo especular
            cv2.circle(frame, (cx - 17, cy - 17), 13, (255, 255, 255), -1)
            cv2.circle(frame, (cx - 6, cy - 6), 6, (200, 200, 200), -1)

            # Etiqueta
            (tw, _), _ = cv2.getTextSize(name, cv2.FONT_HERSHEY_DUPLEX, 1.0, 2)
            cv2.putText(frame, name, (cx - tw//2, cy + 82),
                        cv2.FONT_HERSHEY_DUPLEX, 1.0, TEXT_LIGHT, 2, cv2.LINE_AA)

        # Divisores entre cuadrantes
        cv2.line(frame, (W//2, 0), (W//2, H), (8, 6, 14), 5)
        cv2.line(frame, (0, H//2), (W, H//2), (8, 6, 14), 5)

        # Píldora de instrucción centrada
        instr = f"Color {step_idx}/{total_steps}"
        (iw, ih), _ = cv2.getTextSize(instr, cv2.FONT_HERSHEY_SIMPLEX, 0.78, 2)
        ix = W//2 - iw//2
        _draw_rounded_rect(frame, (ix - 14, 5), (ix + iw + 14, 40),
                           (35, 28, 55), radius=10, alpha=0.90)
        _draw_rounded_rect(frame, (ix - 14, 5), (ix + iw + 14, 40),
                           DIVIDER, radius=10, thickness=1)
        cv2.putText(frame, instr, (ix, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.78, TEXT_LIGHT, 2, cv2.LINE_AA)

        # Barra de tiempo — más delgada y elegante
        bar_w = int((remaining / INPUT_TIMEOUT) * W)
        t_ratio = remaining / INPUT_TIMEOUT
        if t_ratio > 0.5:
            bar_color = (80, 210, 90)        # Verde
        elif t_ratio > 0.25:
            bar_color = (50, 185, 230)       # Amarillo cálido (BGR)
        else:
            bar_color = (70, 70, 220)        # Rojo urgencia
        cv2.rectangle(frame, (0, H - 7), (bar_w, H), bar_color, -1)
        # Brillo en el extremo derecho de la barra
        if bar_w > 8:
            cv2.rectangle(frame, (bar_w - 6, H - 7), (bar_w, H),
                          tuple(min(255, c + 60) for c in bar_color), -1)

        self._draw_hud(frame)
        return frame

    # ── HUD ───────────────────────────────────────────────────────────────────

    def _draw_hud(self, frame: np.ndarray):
        if self._state == State.GAME_OVER:
            return

        # ── Botón X ───────────────────────────────────────────────────────────
        _draw_rounded_rect(frame, (4, 4), (54, 54), (38, 28, 58), radius=10)
        _draw_rounded_rect(frame, (4, 4), (54, 54), DIVIDER, radius=10, thickness=1)
        cv2.putText(frame, "X", (16, 39),
                    cv2.FONT_HERSHEY_DUPLEX, 1.1, (130, 100, 200), 2, cv2.LINE_AA)

        # ── Corazones de vidas ────────────────────────────────────────────────
        # Fondo pill para las vidas
        _draw_rounded_rect(frame, (62, 6), (240, 52), (38, 28, 58), radius=10)
        _draw_rounded_rect(frame, (62, 6), (240, 52), DIVIDER, radius=10, thickness=1)

        vida_lbl = "Lives" if self.is_english else "Vidas"
        cv2.putText(frame, vida_lbl + ":", (72, 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, TEXT_DIM, 1, cv2.LINE_AA)

        # Corazones dibujados como "♥" (aproximado con círculos+triángulo)
        heart_x = 142
        for i in range(self.MAX_LIVES):
            hx = heart_x + i * 32
            hy = 29
            color = HEART_COLOR if i < self._lives else (50, 40, 65)
            # Aproximación de corazón: 2 círculos + triángulo
            cv2.circle(frame, (hx - 5, hy - 4), 7, color, -1)
            cv2.circle(frame, (hx + 5, hy - 4), 7, color, -1)
            pts = np.array([[hx - 11, hy - 1], [hx + 11, hy - 1],
                            [hx, hy + 9]], np.int32)
            cv2.fillPoly(frame, [pts], color)

        # ── Indicador de ronda (derecha) ──────────────────────────────────────
        r_txt = f"R{self._round}"
        (rw, _), _ = cv2.getTextSize(r_txt, cv2.FONT_HERSHEY_SIMPLEX, 0.85, 2)
        _draw_rounded_rect(frame, (W - rw - 26, 6), (W - 6, 52),
                           (38, 28, 58), radius=10)
        _draw_rounded_rect(frame, (W - rw - 26, 6), (W - 6, 52),
                           DIVIDER, radius=10, thickness=1)
        cv2.putText(frame, r_txt, (W - rw - 13, 37),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, ACCENT, 2, cv2.LINE_AA)

    # ── Utilities ─────────────────────────────────────────────────────────────

    def _draw_centered(self, frame, text, scale=2.0, thickness=3,
                       color=(255, 255, 255), shadow_color=(0, 0, 0),
                       y_offset=0):
        font = cv2.FONT_HERSHEY_DUPLEX
        (tw, th), _ = cv2.getTextSize(text, font, scale, thickness)
        x = (W - tw) // 2
        y = H // 2 + th // 2 + y_offset
        # Sombra suave desplazada
        cv2.putText(frame, text, (x + 3, y + 3), font, scale,
                    shadow_color, thickness + 3, cv2.LINE_AA)
        cv2.putText(frame, text, (x, y), font, scale, color, thickness, cv2.LINE_AA)

    def _push(self, frame: np.ndarray):
        with self._frame_lock:
            self._frame = frame


# ─── Entry point ──────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = MemoriaNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()