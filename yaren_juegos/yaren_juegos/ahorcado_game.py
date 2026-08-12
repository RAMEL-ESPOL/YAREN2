#!/usr/bin/env python3
"""
ahorcado_game.py — YAREN2 Hangman Game
LifecycleNode | OpenCV touch input | Bilingual | 3 difficulty levels
Estructura Thread-Safe adaptada al formato de YAREN.
Incluye seguro de salida (Overlay) y mecánica de reposición de letras.
"""

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from lifecycle_msgs.msg import Transition
from std_msgs.msg import String, Bool
from rclpy.qos import QoSProfile, DurabilityPolicy
import cv2
import numpy as np
import threading
import random
import time
import os

# ─────────────────────────────────────────────
#  WORD BANK
# ─────────────────────────────────────────────
WORD_BANK = {
    "es": {
        "facil": [
            "GATO", "PERRO", "CASA", "LUNA", "MESA", "SILLA", "AGUA", "FUEGO",
            "TIERRA", "VIENTO", "ARBOL", "FLOR", "PAJARO", "PECES", "NINO",
            "NINA", "LIBRO", "PUERTA", "VENTANA", "COCHE", "PLAYA", "MONTE",
            "CIUDAD", "CAMPO", "NUBE", "RIO", "MAR", "SOL", "ESTRELLA", "LLUVIA"
        ],
        "medio": [
            "MARIPOSA", "ELEFANTE", "DINOSAURIO", "COMPUTADORA", "TELESCOPIO",
            "BIBLIOTECA", "SUBMARINO", "HELICOPTERO", "MONTANA", "SERPIENTE",
            "COCODRILO", "PIRAMIDE", "LABERINTO", "UNIVERSO", "METEORITO"
        ],
        "dificil": [
            "FOTOSINTESIS", "ELECTROMAGNETICO", "BIODIVERSIDAD",
            "EXTRATERRESTRE", "PALEONTOLOGIA"
        ]
    },
    "en": {
        "facil": [
            "CAT", "DOG", "HOUSE", "MOON", "TABLE", "CHAIR", "WATER", "FIRE",
            "EARTH", "WIND", "TREE", "FLOWER", "BIRD", "FISH", "BOY",
            "GIRL", "BOOK", "DOOR", "WINDOW", "CAR", "BEACH", "HILL",
            "CITY", "FIELD", "CLOUD", "RIVER", "SEA", "SUN", "STAR", "RAIN"
        ],
        "medio": [
            "BUTTERFLY", "ELEPHANT", "DINOSAUR", "COMPUTER", "TELESCOPE",
            "LIBRARY", "SUBMARINE", "HELICOPTER", "MOUNTAIN", "SERPENT",
            "CROCODILE", "PYRAMID", "LABYRINTH", "UNIVERSE", "METEORITE"
        ],
        "dificil": [
            "PHOTOSYNTHESIS", "ELECTROMAGNETIC", "BIODIVERSITY",
            "EXTRATERRESTRIAL", "PALEONTOLOGY"
        ]
    }
}

LEVEL_CONFIG = {
    "facil":   {"lives": 6, "words": 10, "label_es": "FACIL",   "label_en": "EASY",   "color": (80, 200, 80)},
    "medio":   {"lives": 5, "words": 5,  "label_es": "MEDIO",   "label_en": "MEDIUM", "color": (80, 180, 220)},
    "dificil": {"lives": 3, "words": 1,  "label_es": "DIFICIL", "label_en": "HARD",   "color": (80, 80, 220)},
}

# ─── Paleta YAREN (BGR) ────────────────────────────────────────────────────────
W, H = 800, 480
FPS  = 30

BG_DARK        = (18, 12, 28)
BG_PANEL       = (30, 22, 46)
PANEL_COLOR    = (35, 28, 55)
DIVIDER        = (60, 45, 85)
WHITE          = (240, 240, 240)
TEXT_LIGHT     = (240, 235, 255)
TEXT_DIM       = (160, 145, 190)
YELLOW         = (0, 220, 255)
GREEN          = (80, 210, 90)
RED            = (60, 60, 220)
ORANGE         = (50, 165, 255)
GRAY           = (120, 120, 140)
DARK_GRAY      = (60, 60, 75)
ACCENT         = (230, 140, 60)
ACCENT2        = (200, 90, 40)
HEART_COLOR    = (90, 70, 220)

ALPHABET_ES = list("ABCDEFGHIJKLMNOPQRSTUVWXYZ")
ALPHABET_EN = list("ABCDEFGHIJKLMNOPQRSTUVWXYZ")

# ─────────────────────────────────────────────
#  HELPERS DE DIBUJO YAREN
# ─────────────────────────────────────────────
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
        cv2.ellipse(img, (x1+radius, y1+radius), (radius, radius), 180, 0, 90, color, thickness, cv2.LINE_AA)
        cv2.ellipse(img, (x2-radius, y1+radius), (radius, radius), 270, 0, 90, color, thickness, cv2.LINE_AA)
        cv2.ellipse(img, (x1+radius, y2-radius), (radius, radius),  90, 0, 90, color, thickness, cv2.LINE_AA)
        cv2.ellipse(img, (x2-radius, y2-radius), (radius, radius),   0, 0, 90, color, thickness, cv2.LINE_AA)

def _gradient_bg(h, w, top_color, bot_color):
    frame = np.zeros((h, w, 3), dtype=np.float32)
    for i in range(h):
        t = i / h
        frame[i] = [top_color[c] * (1 - t) + bot_color[c] * t for c in range(3)]
    return frame.astype(np.uint8)

def draw_hangman(frame, errors, ox=160, oy=140, scale=1.0):
    s = scale
    lw = max(2, int(4 * s))
    c = TEXT_LIGHT
    wood = (90, 110, 140)

    cv2.line(frame, (int(ox-80*s), int(oy+220*s)), (int(ox+80*s), int(oy+220*s)), wood, lw+2, cv2.LINE_AA)
    cv2.line(frame, (int(ox-30*s), int(oy+220*s)), (int(ox-30*s), int(oy+10*s)),   wood, lw+2, cv2.LINE_AA)
    cv2.line(frame, (int(ox-30*s), int(oy+10*s)),  (int(ox+70*s), int(oy+10*s)),   wood, lw+2, cv2.LINE_AA)
    cv2.line(frame, (int(ox+70*s), int(oy+10*s)),  (int(ox+70*s), int(oy+35*s)),   wood, lw, cv2.LINE_AA)

    if errors >= 1:
        cv2.circle(frame, (int(ox+70*s), int(oy+55*s)), int(20*s), c, lw, cv2.LINE_AA)
    if errors >= 2:
        cv2.line(frame, (int(ox+70*s), int(oy+75*s)), (int(ox+70*s), int(oy+140*s)), c, lw, cv2.LINE_AA)
    if errors >= 3:
        cv2.line(frame, (int(ox+70*s), int(oy+90*s)), (int(ox+40*s), int(oy+115*s)), c, lw, cv2.LINE_AA)
    if errors >= 4:
        cv2.line(frame, (int(ox+70*s), int(oy+90*s)), (int(ox+100*s), int(oy+115*s)), c, lw, cv2.LINE_AA)
    if errors >= 5:
        cv2.line(frame, (int(ox+70*s), int(oy+140*s)), (int(ox+40*s), int(oy+185*s)), c, lw, cv2.LINE_AA)
    if errors >= 6:
        cv2.line(frame, (int(ox+70*s), int(oy+140*s)), (int(ox+100*s), int(oy+185*s)), c, lw, cv2.LINE_AA)

# ─────────────────────────────────────────────
#  BUTTON HELPER
# ─────────────────────────────────────────────
class Button:
    def __init__(self, x, y, w, h, text, bg_color=PANEL_COLOR, text_color=TEXT_LIGHT,
                 border_color=DIVIDER, font_scale=0.8, radius=12):
        self.rect = (x, y, w, h)
        self.text = text
        self.bg_color = bg_color
        self.text_color = text_color
        self.border_color = border_color
        self.font_scale = font_scale
        self.radius = radius
        self.hovered = False

    def draw(self, frame):
        x, y, w, h = self.rect
        if self.hovered:
            b_color  = tuple(min(c + 40, 255) for c in self.bg_color)
            br_color = YELLOW
        else:
            b_color  = self.bg_color
            br_color = self.border_color

        _draw_rounded_rect(frame, (x, y), (x+w, y+h), b_color, radius=self.radius, thickness=-1)
        _draw_rounded_rect(frame, (x, y), (x+w, y+h), br_color, radius=self.radius, thickness=2)

        font = cv2.FONT_HERSHEY_DUPLEX
        (tw, th), baseline = cv2.getTextSize(self.text, font, self.font_scale, 2)
        tx = x + (w - tw) // 2
        ty = y + (h + th) // 2 - baseline // 2
        cv2.putText(frame, self.text, (tx, ty),
                    font, self.font_scale, self.text_color, 2 if self.hovered else 1, cv2.LINE_AA)

    def is_clicked(self, mx, my):
        x, y, w, h = self.rect
        return x <= mx <= x + w and y <= my <= y + h

# ─────────────────────────────────────────────
#  MAIN NODE
# ─────────────────────────────────────────────
class AhorcadoNode(LifecycleNode):

    def __init__(self):
        super().__init__('ahorcado_node')
        self.get_logger().info('AhorcadoNode created')
        self._active  = False
        self.is_english = False
        self._lang_sub = None
        self.show_exit_confirm = False

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring ahorcado_node...')
        self._game_active_pub = self.create_publisher(Bool, '/yaren/game_active', 10)

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._lang_sub = self.create_subscription(Bool, '/yaren/is_english', self._lang_cb, qos)

        self._reset_game_state()
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating ahorcado_node...')
        self._active = True
        msg = Bool()
        msg.data = True
        self._game_active_pub.publish(msg)
        self._reset_game_state()
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating ahorcado_node...')
        self._active = False
        msg = Bool()
        msg.data = False
        self._game_active_pub.publish(msg)
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        if self._lang_sub:
            self.destroy_subscription(self._lang_sub)
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._active = False
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        return TransitionCallbackReturn.SUCCESS

    # ── ROS Callbacks ───────────────────────
    def _lang_cb(self, msg):
        self.is_english = msg.data
        self.language = 'en' if self.is_english else 'es'
        if getattr(self, 'screen', None) == 'intro':          self._build_intro_buttons()
        elif getattr(self, 'screen', None) == 'level':        self._build_level_buttons()
        elif getattr(self, 'screen', None) == 'word_result':  self._build_word_result_buttons()
        elif getattr(self, 'screen', None) == 'level_complete': self._build_level_complete_buttons()
        elif getattr(self, 'screen', None) == 'game_over':    self._build_game_over_buttons()
        self._build_confirm_buttons()

    # ── Game State ───────────────────────
    def _reset_game_state(self):
        self.screen        = 'intro'
        self.language      = 'en' if self.is_english else 'es'
        self.level         = 'facil'
        self.word_list     = []
        self.word_index    = 0
        self.current_word  = ''
        self.guessed       = set()
        self.errors        = 0
        self.max_lives     = 6
        self.letters_shown = []
        self.remaining_pool = []
        self.result_msg    = ''
        self.result_win    = False
        self.buttons       = {}
        self.show_exit_confirm = False
        self._build_intro_buttons()
        self._build_confirm_buttons()

    def _start_level(self):
        cfg  = LEVEL_CONFIG[self.level]
        bank = WORD_BANK[self.language][self.level].copy()
        random.shuffle(bank)
        self.word_list  = bank[:cfg['words']]
        self.word_index = 0
        self._load_word()

    def _load_word(self):
        self.current_word = self.word_list[self.word_index]
        self.guessed      = set()
        self.errors       = 0
        self.max_lives    = LEVEL_CONFIG[self.level]['lives']
        self._generate_letters()
        self.screen = 'playing'
        self._build_letter_buttons()

    def _generate_letters(self):
        word_letters = list(set(self.current_word))
        alpha   = ALPHABET_EN if self.language == 'en' else ALPHABET_ES
        fillers = [l for l in alpha if l not in word_letters]

        random.shuffle(word_letters)
        random.shuffle(fillers)

        num_correct_initial = min(3, len(word_letters))
        initial_letters = word_letters[:num_correct_initial] + fillers[:10 - num_correct_initial]
        random.shuffle(initial_letters)

        self.letters_shown  = initial_letters
        self.remaining_pool = word_letters[num_correct_initial:] + fillers[10 - num_correct_initial:]
        random.shuffle(self.remaining_pool)

    def _normalize(self, word):
        return word

    def _check_win(self):
        norm = self._normalize(self.current_word)
        return all(ch in self.guessed or ch == ' ' for ch in norm)

    # ── Construcción de UI ──────────────────────────────────────────────
    def _build_intro_buttons(self):
        btn_y1 = H - 90
        btn_h  = 58
        btn_w  = 200
        sx1 = W // 2 - btn_w - 12
        bx1 = W // 2 + 12
        start_lbl = "START"  if self.is_english else "EMPEZAR"
        back_lbl  = "BACK"   if self.is_english else "VOLVER"
        self.buttons = {
            'start': Button(sx1, btn_y1, btn_w, btn_h, start_lbl, ACCENT2, TEXT_LIGHT, ACCENT, 1.05, 14),
            'back':  Button(bx1, btn_y1, btn_w, btn_h, back_lbl, (40, 30, 60), TEXT_DIM, DIVIDER, 1.05, 14),
        }

    def _build_level_buttons(self):
        self.buttons = {
            'facil':   Button(160-85, 215, 170, 55, self._t('FACIL','EASY'),   (40,120,40), WHITE, GREEN, 0.8),
            'medio':   Button(400-85, 215, 170, 55, self._t('MEDIO','MEDIUM'), (40,110,140), WHITE, YELLOW, 0.8),
            'dificil': Button(640-85, 215, 170, 55, self._t('DIFICIL','HARD'), (40,40,140), WHITE, RED, 0.8),
            'back':    Button(W//2-100, H-80, 200, 55, self._t('VOLVER','BACK'), (40, 30, 60), TEXT_DIM, DIVIDER, 0.9, 14),
        }

    def _build_letter_buttons(self):
        self.buttons = {}
        cols  = 5
        btn_w, btn_h = 75, 55
        gap_x, gap_y = 12, 12
        total_w = cols * btn_w + (cols - 1) * gap_x
        start_x = 310 + (460 - total_w) // 2
        start_y = 310

        for i, letter in enumerate(self.letters_shown):
            if letter == '':
                continue
            row = i // cols
            col = i % cols
            x = start_x + col * (btn_w + gap_x)
            y = start_y + row * (btn_h + gap_y)
            self.buttons[f'letter_{letter}'] = Button(x, y, btn_w, btn_h, letter,
                                                      PANEL_COLOR, TEXT_LIGHT, DIVIDER, 0.85, 12)

    def _build_word_result_buttons(self):
        if self.result_win:
            self.buttons = {
                'next': Button(W//2-100, 310, 200, 55, self._t('Siguiente ->','Next ->'), (40,120,40), WHITE, GREEN, 0.8),
            }
        else:
            self.buttons = {
                'restart': Button(W//2-220, 310, 200, 55, self._t('Reiniciar nivel','Restart level'), ACCENT2, WHITE, ACCENT, 0.7),
                'levels':  Button(W//2+20,  310, 200, 55, self._t('Elegir nivel','Change level'), BG_PANEL, TEXT_LIGHT, DIVIDER, 0.7),
            }

    def _build_level_complete_buttons(self):
        self.buttons = {
            'next_level': Button(W//2-220, 310, 200, 55, self._t('Siguiente nivel','Next level'), (40,120,40), WHITE, GREEN, 0.7),
            'levels':     Button(W//2+20,  310, 200, 55, self._t('Elegir nivel','Choose level'), BG_PANEL, TEXT_LIGHT, DIVIDER, 0.7),
        }

    def _build_game_over_buttons(self):
        self.buttons = {
            'restart': Button(W//2-220, 310, 200, 55, self._t('Reiniciar nivel','Restart level'), ACCENT2, WHITE, ACCENT, 0.7),
            'levels':  Button(W//2+20,  310, 200, 55, self._t('Elegir nivel','Choose level'), BG_PANEL, TEXT_LIGHT, DIVIDER, 0.7),
        }

    def _build_confirm_buttons(self):
        btn_w, btn_h = 130, 45
        self.confirm_buttons = {
            'no':  Button(W//2 - btn_w - 10, H//2 + 10, btn_w, btn_h, self._t('NO', 'NO'), (40, 120, 40), WHITE, GREEN, 0.8),
            'yes': Button(W//2 + 10, H//2 + 10, btn_w, btn_h, self._t('SI', 'YES'), (60, 30, 40), WHITE, RED, 0.8)
        }

    def _t(self, es_text, en_text):
        return en_text if self.language == 'en' else es_text

    # ── Mouse Callback ─────────────────────────
    def _on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            if self.show_exit_confirm:
                for btn in self.confirm_buttons.values():
                    btn.hovered = btn.is_clicked(x, y)
                return
            for btn in self.buttons.values():
                btn.hovered = btn.is_clicked(x, y)

        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.show_exit_confirm:
            if self.confirm_buttons['yes'].is_clicked(x, y):
                self.show_exit_confirm = False
                self._active = False
            elif self.confirm_buttons['no'].is_clicked(x, y):
                self.show_exit_confirm = False
            return

        if x < 60 and y < 60:
            if self.screen == 'intro':
                self._active = False
            else:
                self.show_exit_confirm = True
            return

        if self.screen == 'intro':
            if self.buttons.get('start') and self.buttons['start'].is_clicked(x, y):
                self.screen = 'level'
                self._build_level_buttons()
            elif self.buttons.get('back') and self.buttons['back'].is_clicked(x, y):
                self._active = False

        elif self.screen == 'level':
            for lvl in ('facil', 'medio', 'dificil'):
                if self.buttons.get(lvl) and self.buttons[lvl].is_clicked(x, y):
                    self.level = lvl
                    self._start_level()
                    return
            if self.buttons.get('back') and self.buttons['back'].is_clicked(x, y):
                self.screen = 'intro'
                self._build_intro_buttons()

        elif self.screen == 'playing':
            for key, btn in list(self.buttons.items()):
                if key.startswith('letter_') and btn.is_clicked(x, y):
                    letter = key.replace('letter_', '')
                    if letter not in self.guessed:
                        self.guessed.add(letter)
                        norm = self._normalize(self.current_word)
                        if letter not in norm:
                            self.errors += 1

                        try:
                            idx = self.letters_shown.index(letter)
                        except ValueError:
                            idx = None

                        if idx is not None:
                            self.letters_shown[idx] = ''

                            correct_on_screen  = sum(1 for l in self.letters_shown if l != '' and l in norm and l != ' ')
                            unguessed_correct  = [l for l in set(norm) if l not in self.guessed and l not in self.letters_shown and l != ' ']

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
                            self.result_win = True
                            self.result_msg = self._t('Correcto! La palabra era: ', 'Correct! The word was: ') + self.current_word
                            if self.word_index >= len(self.word_list) - 1:
                                self.screen = 'level_complete'
                                self._build_level_complete_buttons()
                            else:
                                self.screen = 'word_result'
                                self._build_word_result_buttons()
                        elif self.errors >= self.max_lives:
                            self.result_win = False
                            self.result_msg = self._t('Perdiste! La palabra era: ', 'You lost! The word was: ') + self.current_word
                            self.screen = 'game_over'
                            self._build_game_over_buttons()
                    break

        elif self.screen == 'word_result':
            if self.buttons.get('next') and self.buttons['next'].is_clicked(x, y):
                self.word_index += 1
                self._load_word()
            elif self.buttons.get('restart') and self.buttons['restart'].is_clicked(x, y):
                self._start_level()
            elif self.buttons.get('levels') and self.buttons['levels'].is_clicked(x, y):
                self.screen = 'level'
                self._build_level_buttons()

        elif self.screen == 'level_complete':
            if self.buttons.get('next_level') and self.buttons['next_level'].is_clicked(x, y):
                levels = list(LEVEL_CONFIG.keys())
                idx = levels.index(self.level)
                if idx < len(levels) - 1:
                    self.level = levels[idx + 1]
                    self._start_level()
                else:
                    self.screen = 'level'
                    self._build_level_buttons()
            elif self.buttons.get('levels') and self.buttons['levels'].is_clicked(x, y):
                self.screen = 'level'
                self._build_level_buttons()

        elif self.screen == 'game_over':
            if self.buttons.get('restart') and self.buttons['restart'].is_clicked(x, y):
                self._start_level()
            elif self.buttons.get('levels') and self.buttons['levels'].is_clicked(x, y):
                self.screen = 'level'
                self._build_level_buttons()

    # ── Draw Helpers Básicos ────────────────────────────────
    def _draw_bg(self, frame):
        frame[:] = _gradient_bg(H, W, BG_DARK, (28, 18, 45))

    def _draw_title(self, frame, text, y=60, color=YELLOW, scale=1.5):
        font = cv2.FONT_HERSHEY_DUPLEX
        (tw, _), _ = cv2.getTextSize(text, font, scale, 2)
        cv2.putText(frame, text, ((W-tw)//2, y), font, scale, color, 2, cv2.LINE_AA)

    def _draw_text_center(self, frame, text, y, color=WHITE, scale=0.8, thickness=1):
        font = cv2.FONT_HERSHEY_DUPLEX
        (tw, _), _ = cv2.getTextSize(text, font, scale, thickness)
        cv2.putText(frame, text, ((W-tw)//2, y), font, scale, color, thickness, cv2.LINE_AA)

    def _draw_close_button(self, frame):
        _draw_rounded_rect(frame, (4, 4), (54, 54), PANEL_COLOR, radius=10, thickness=-1)
        _draw_rounded_rect(frame, (4, 4), (54, 54), DIVIDER, radius=10, thickness=2)
        cv2.putText(frame, "X", (16, 39), cv2.FONT_HERSHEY_DUPLEX, 1.1, RED, 2, cv2.LINE_AA)

    def _draw_heart(self, frame, cx, cy, color, scale=1.0):
        r = int(6 * scale)
        cv2.circle(frame, (cx - int(5*scale), cy - int(3*scale)), r, color, -1, cv2.LINE_AA)
        cv2.circle(frame, (cx + int(5*scale), cy - int(3*scale)), r, color, -1, cv2.LINE_AA)
        pts = np.array([[cx - int(10*scale), cy],
                        [cx + int(10*scale), cy],
                        [cx, cy + int(10*scale)]], np.int32)
        cv2.fillPoly(frame, [pts], color, cv2.LINE_AA)

    # ── Screen Renderers ───────────────────────
    def _render_intro(self, frame):
        self._draw_bg(frame)
        px1, py1 = 30, 18
        px2, py2 = W - 30, H - 18
        _draw_rounded_rect(frame, (px1, py1), (px2, py2), BG_PANEL, radius=20, alpha=0.97)
        _draw_rounded_rect(frame, (px1, py1), (px2, py2), DIVIDER, radius=20, thickness=2)

        title = "HANGMAN GAME" if self.is_english else "JUEGO DEL AHORCADO"
        font_d = cv2.FONT_HERSHEY_DUPLEX
        font_s = cv2.FONT_HERSHEY_SIMPLEX

        (tw, th), _ = cv2.getTextSize(title, font_d, 1.4, 3)
        tx = W // 2 - tw // 2
        cv2.putText(frame, title, (tx + 2, 66), font_d, 1.4, (10, 6, 20), 3, cv2.LINE_AA)
        cv2.putText(frame, title, (tx, 64), font_d, 1.4, ACCENT, 3, cv2.LINE_AA)
        cv2.line(frame, (px1 + 40, 84), (px2 - 40, 84), DIVIDER, 1)
        draw_hangman(frame, 6, ox=W//2, oy=90, scale=0.5)

        if self.is_english:
            lines = [
                ("Guess the hidden word letter by letter", False),
                ("before you run out of lives.", False),
                ("", False),
                ("10 words  |  3 difficulty levels  |  Choose wisely!", True),
            ]
        else:
            lines = [
                ("Adivina la palabra oculta letra por letra", False),
                ("antes de quedarte sin vidas.", False),
                ("", False),
                ("10 palabras  |  3 niveles de dificultad  |  Escoge bien!", True),
            ]

        y_text = 240
        for line, highlight in lines:
            if line == "":
                y_text += 14
                continue
            scale = 0.60
            (lw, lh), _ = cv2.getTextSize(line, font_s, scale, 1)
            lx  = W // 2 - lw // 2
            col = ACCENT if highlight else TEXT_DIM
            thick = 2 if highlight else 1
            cv2.putText(frame, line, (lx, y_text), font_s, scale, col, thick, cv2.LINE_AA)
            y_text += lh + 16

        for btn in self.buttons.values():
            btn.draw(frame)

    def _render_level(self, frame):
        self._draw_bg(frame)
        self._draw_title(frame, self._t('ELIGE TU NIVEL', 'CHOOSE YOUR LEVEL'), 70, YELLOW, 1.3)

        configs = [
            ('facil',   GREEN,  self._t('10 palabras','10 words'), self._t('6 vidas','6 lives')),
            ('medio',   YELLOW, self._t('5 palabras', '5 words'),  self._t('5 vidas','5 lives')),
            ('dificil', RED,    self._t('1 palabra',  '1 word'),   self._t('3 vidas','3 lives')),
        ]
        cx_list = [160, 400, 640]
        for i, (lvl, col, words, lives) in enumerate(configs):
            cx = cx_list[i]
            _draw_rounded_rect(frame, (cx-105, 140), (cx+105, 360), DARK_GRAY, radius=15)
            stars = '*' * (i+1) + '-' * (2-i)
            font  = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(frame, stars, (cx-85, 180), font, 0.8, col, 1, cv2.LINE_AA)
            cv2.putText(frame, words, (cx-85, 310), font, 0.55, WHITE, 1, cv2.LINE_AA)
            cv2.putText(frame, lives, (cx-85, 340), font, 0.55, WHITE, 1, cv2.LINE_AA)

        for btn in self.buttons.values():
            btn.draw(frame)
        self._draw_close_button(frame)

    def _render_playing(self, frame):
        self._draw_bg(frame)
        cfg  = LEVEL_CONFIG[self.level]
        font = cv2.FONT_HERSHEY_DUPLEX

        cv2.rectangle(frame, (0, 0), (W, 55), BG_PANEL, -1)
        cv2.line(frame, (0, 55), (W, 55), DIVIDER, 2)
        self._draw_close_button(frame)

        level_label = cfg[f'label_{self.language}']
        cv2.putText(frame, level_label, (75, 36), font, 0.75, cfg['color'], 1, cv2.LINE_AA)

        prog_txt = f"{self._t('Palabra','Word')} {self.word_index+1}/{len(self.word_list)}"
        (pw, _), _ = cv2.getTextSize(prog_txt, font, 0.7, 1)
        cv2.putText(frame, prog_txt, ((W-pw)//2, 36), font, 0.7, TEXT_LIGHT, 1, cv2.LINE_AA)

        lives_left   = self.max_lives - self.errors
        heart_x_start = W - 30 - (self.max_lives * 25)
        for i in range(self.max_lives):
            hx    = heart_x_start + i * 25
            hy    = 28
            color = HEART_COLOR if i < lives_left else (60, 50, 75)
            self._draw_heart(frame, hx, hy, color, scale=1.1)

        _draw_rounded_rect(frame, (30, 80), (290, 450), BG_PANEL, radius=15, alpha=0.8)
        draw_hangman(frame, self.errors, ox=160, oy=130, scale=1.1)

        norm   = self._normalize(self.current_word)
        max_w  = 460
        ideal_w = len(norm) * 50 + (len(norm) - 1) * 10
        scale  = min(1.0, max_w / ideal_w) if ideal_w > 0 else 1.0
        slot_w = int(50 * scale)
        slot_h = int(60 * scale)
        gap    = int(10 * scale)
        total_w = len(norm) * slot_w + (len(norm) - 1) * gap
        start_x = 310 + (460 - total_w) // 2
        start_y = 160

        for i, ch in enumerate(norm):
            x = start_x + i * (slot_w + gap)
            _draw_rounded_rect(frame, (x, start_y), (x+slot_w, start_y+slot_h), (25, 18, 35), radius=8)
            _draw_rounded_rect(frame, (x, start_y), (x+slot_w, start_y+slot_h), DIVIDER, radius=8, thickness=2)
            if ch in self.guessed or ch == ' ':
                (lw, lh), _ = cv2.getTextSize(ch, font, 1.4 * scale, 2)
                lx = x + (slot_w - lw) // 2
                ly = start_y + (slot_h + lh) // 2
                cv2.putText(frame, ch, (lx, ly), font, 1.4 * scale, YELLOW, 2, cv2.LINE_AA)

        wrong = [l for l in sorted(self.guessed) if l not in norm]
        if wrong:
            wrong_txt = self._t('Errores: ','Errors: ') + ' '.join(wrong)
            (tw2, _), _ = cv2.getTextSize(wrong_txt, font, 0.6, 1)
            wx = 310 + (460 - tw2) // 2
            cv2.putText(frame, wrong_txt, (wx, 260), font, 0.6, RED, 1, cv2.LINE_AA)

        hint = self._t('Toca una letra para adivinar', 'Touch a letter to guess')
        (hw, _), _ = cv2.getTextSize(hint, font, 0.5, 1)
        cv2.putText(frame, hint, (310 + (460-hw)//2, 290), font, 0.5, TEXT_DIM, 1, cv2.LINE_AA)

        for btn in self.buttons.values():
            btn.bg_color    = PANEL_COLOR
            btn.text_color  = TEXT_LIGHT
            btn.border_color = DIVIDER
            btn.draw(frame)

    def _render_word_result(self, frame):
        self._draw_bg(frame)
        _draw_rounded_rect(frame, (100, 90), (700, 390), BG_PANEL, radius=20, alpha=0.95)
        _draw_rounded_rect(frame, (100, 90), (700, 390), DIVIDER, radius=20, thickness=2)
        color = GREEN if self.result_win else RED
        icon  = self._t('Correcto!', 'Correct!') if self.result_win else self._t('Incorrecto!','Incorrect!')
        self._draw_title(frame, icon, 170, color, 1.4)
        self._draw_text_center(frame, self.result_msg, 220, WHITE, 0.8)
        if self.result_win and self.word_index < len(self.word_list) - 1:
            nxt = self._t(f'Siguiente: palabra {self.word_index+2}/{len(self.word_list)}',
                          f'Next: word {self.word_index+2}/{len(self.word_list)}')
            self._draw_text_center(frame, nxt, 260, TEXT_DIM, 0.65)
        for btn in self.buttons.values():
            btn.draw(frame)
        self._draw_close_button(frame)

    def _render_level_complete(self, frame):
        self._draw_bg(frame)
        _draw_rounded_rect(frame, (100, 90), (700, 390), BG_PANEL, radius=20, alpha=0.95)
        _draw_rounded_rect(frame, (100, 90), (700, 390), DIVIDER, radius=20, thickness=2)
        self._draw_title(frame, self._t('NIVEL COMPLETADO!','LEVEL COMPLETE!'), 160, GREEN, 1.3)
        cfg = LEVEL_CONFIG[self.level]
        msg = self._t(f'Completaste el nivel {cfg["label_es"]}',
                      f'You completed the {cfg["label_en"]} level')
        self._draw_text_center(frame, msg, 220, WHITE, 0.8)
        levels = list(LEVEL_CONFIG.keys())
        if self.level != levels[-1]:
            next_lvl = levels[levels.index(self.level)+1]
            next_cfg  = LEVEL_CONFIG[next_lvl]
            hint = self._t(f'Intentas el nivel {next_cfg["label_es"]}?',
                           f'Try the {next_cfg["label_en"]} level?')
            self._draw_text_center(frame, hint, 260, YELLOW, 0.7)
        else:
            self._draw_text_center(frame, self._t('Eres un campeon!','You are a champion!'), 260, YELLOW, 0.8)
        for btn in self.buttons.values():
            btn.draw(frame)
        self._draw_close_button(frame)

    def _render_game_over(self, frame):
        self._draw_bg(frame)
        _draw_rounded_rect(frame, (100, 90), (700, 390), BG_PANEL, radius=20, alpha=0.95)
        _draw_rounded_rect(frame, (100, 90), (700, 390), DIVIDER, radius=20, thickness=2)
        self._draw_title(frame, self._t('GAME OVER!','GAME OVER!'), 160, RED, 1.5)
        self._draw_text_center(frame, self.result_msg, 220, WHITE, 0.75)
        draw_hangman(frame, 6, ox=W//2, oy=250, scale=0.6)
        for btn in self.buttons.values():
            btn.draw(frame)
        self._draw_close_button(frame)

    def _render_confirm_overlay(self, frame):
        ov = frame.copy()
        cv2.rectangle(ov, (0, 0), (W, H), (0, 0, 0), -1)
        cv2.addWeighted(ov, 0.75, frame, 0.25, 0, frame)
        pw, ph = 420, 200
        px, py = (W - pw) // 2, (H - ph) // 2
        _draw_rounded_rect(frame, (px, py), (px + pw, py + ph), BG_PANEL, radius=18)
        _draw_rounded_rect(frame, (px, py), (px + pw, py + ph), DIVIDER, radius=18, thickness=2)
        msg = self._t('Seguro que quieres salir?', 'Are you sure you want to quit?')
        self._draw_text_center(frame, msg, py + 70, WHITE, 0.75)
        for btn in self.confirm_buttons.values():
            btn.draw(frame)

    # ── Main Display Loop ────────────────────────────────────────────────────
    def run_display_main_thread(self):
        win_name = 'YAREN - Ahorcado'

        while rclpy.ok():
            # Esperar a que el nodo sea activado externamente (sin watchdog)
            while not self._active and rclpy.ok():
                time.sleep(0.05)

            if not rclpy.ok():
                break

            # ── Abrir ventana en el main thread ──────────────────────────
            cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(win_name, W, H)
            cv2.setWindowProperty(win_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            cv2.setWindowProperty(win_name, cv2.WND_PROP_TOPMOST, 1)
            cv2.setMouseCallback(win_name, self._on_mouse)

            def _focus():
                time.sleep(0.4)
                os.system(f"xdotool search --sync --name '{win_name}' "
                          "windowactivate --sync windowraise 2>/dev/null")
            threading.Thread(target=_focus, daemon=True).start()

            frame = np.zeros((H, W, 3), dtype=np.uint8)

            # ── Loop de render ────────────────────────────────────────────
            while rclpy.ok() and self._active:
                
                # FIX: Verificación segura de cierre de ventana en Linux (AUTOSIZE)
                try:
                    if cv2.getWindowProperty(win_name, cv2.WND_PROP_AUTOSIZE) == -1:
                        self._active = False
                        break
                except Exception:
                    self._active = False
                    break

                frame[:] = BG_DARK

                if   self.screen == 'intro':          self._render_intro(frame)
                elif self.screen == 'level':          self._render_level(frame)
                elif self.screen == 'playing':        self._render_playing(frame)
                elif self.screen == 'word_result':    self._render_word_result(frame)
                elif self.screen == 'level_complete': self._render_level_complete(frame)
                elif self.screen == 'game_over':      self._render_game_over(frame)

                if self.show_exit_confirm:
                    self._render_confirm_overlay(frame)

                cv2.imshow(win_name, frame)
                key = cv2.waitKey(1000 // FPS) & 0xFF
                if key == 27:
                    self._active = False
                    break

            # ── Limpieza al salir del loop ────────────────────────────────
            try:
                cv2.destroyAllWindows()
                cv2.waitKey(1)
            except Exception:
                pass

            msg = Bool()
            msg.data = False
            self._game_active_pub.publish(msg)

            if not rclpy.ok():
                break
            break


# ─────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = AhorcadoNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run_display_main_thread()
    except KeyboardInterrupt:
        pass
    finally:
        node._active = False
        try:
            cv2.destroyAllWindows()
            cv2.waitKey(1)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=3.0)


if __name__ == '__main__':
    main()
