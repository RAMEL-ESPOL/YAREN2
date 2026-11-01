#!/usr/bin/env python3
"""
fondo_virtual.py  —  LifecycleNode
─────────────────────────────────────────────
on_configure  → responde de inmediato; carga MediaPipe y fondos en hilo
on_activate   → suscribe a la cámara, crea timer de render, habilita ventana
on_deactivate → desuscribe, destruye timer, cierra ventanas
on_cleanup    → libera MediaPipe y fondos
"""

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import os
import glob
import math
import random
import threading
import time

TARGET_W = 800
TARGET_H = 480


class VirtualBackgroundNode(LifecycleNode):

    def __init__(self):
        super().__init__('virtual_background_node')
        self._bridge       = CvBridge()
        self._segmentator  = None
        self._bg_images    = []
        self._thumbnails   = []
        self._bg_names     = []
        self._pressed      = None
        self._btn_rects    = {}
        self._menu_cache   = {}
        self._is_english   = False
        self._lang_sub     = None

        # Bandera para saber si la carga pesada ya terminó
        self._configured_ready = False
        self._init_lock        = threading.Lock()

        # Variables de estado y UI
        self._state            = 'MENU'
        self._bg_index         = 0
        self._thumb_w          = 220
        self._thumb_h          = 140
        self._scroll_y         = 0
        self._max_scroll       = 0
        self._is_dragging      = False
        self._drag_start_y     = 0
        self._click_start_pos  = (0, 0)
        self._thumbnail_rects  = []
        self._exit_btn_rect    = (15, 10, 125, 45)

        # Caché para optimización de fondo
        self._cached_bg        = None
        self._cached_bg_index  = -1

        self._active       = False
        self._cam_sub      = None
        self._latest_frame = None
        self._frame_lock   = threading.Lock()
        self._window_name  = 'YAREN - Fondos Virtuales'

        self._render_timer = None

    # ── Lifecycle ──────────────────────────────────────────────────────

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        lang_qos = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        )
        self._lang_sub = self.create_subscription(
            Bool, '/yaren/is_english', self._lang_callback, lang_qos)

        # Carga pesada en hilo — responde configure inmediatamente
        self._configured_ready = False
        threading.Thread(target=self._init_heavy, daemon=True).start()

        self.get_logger().info(
            'Configurando — cargando MediaPipe y fondos en background...')
        return TransitionCallbackReturn.SUCCESS

    def _init_heavy(self):
        """Carga MediaPipe y fondos en un hilo separado para no bloquear configure."""
        try:
            import mediapipe as mp
            mp_selfie = mp.solutions.selfie_segmentation
            with self._init_lock:
                self._segmentator = mp_selfie.SelfieSegmentation(model_selection=1)
            self._load_backgrounds()
            self._configured_ready = True
            self.get_logger().info(
                f'MediaPipe listo. {len(self._bg_images)} fondos cargados en RAM.')
        except Exception as e:
            self.get_logger().error(f'_init_heavy falló: {e}')
            self._configured_ready = True  # Dejar pasar para no bloquear activate

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # Si la carga aún no terminó, esperar hasta 10s antes de fallar
        deadline = time.time() + 10.0
        while not self._configured_ready and time.time() < deadline:
            self.get_logger().info('Esperando que MediaPipe termine de cargar...')
            time.sleep(0.5)

        if not self._configured_ready:
            self.get_logger().warn(
                'MediaPipe no terminó de cargar a tiempo, activando de todas formas.')

        self._cam_sub = self.create_subscription(
            Image, '/csi_camera/image_raw',
            self._image_callback, qos_profile_sensor_data)

        self._state    = 'MENU'
        self._scroll_y = 0
        self._active   = True

        # Timer a ~30 Hz para no asfixiar el ejecutor de ROS 2
        self._render_timer = self.create_timer(0.033, self._spin_render)
        self._window_created = False  # <-- NUEVO: Reiniciar bandera al activar

        self.get_logger().info('ACTIVADO — mostrando menú de fondos.')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._active = False

        if self._render_timer:
            self.destroy_timer(self._render_timer)
            self._render_timer = None

        if self._cam_sub:
            self.destroy_subscription(self._cam_sub)
            self._cam_sub = None

        try:
            cv2.destroyWindow(self._window_name)
        except Exception:
            pass

        self.get_logger().info('DESACTIVADO.')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        with self._init_lock:
            if self._segmentator:
                self._segmentator.close()
                self._segmentator = None

        if self._lang_sub:
            self.destroy_subscription(self._lang_sub)
            self._lang_sub = None

        self._bg_images.clear()
        self._thumbnails.clear()
        self._bg_names.clear()
        self._cached_bg = None
        self._cached_bg_index = -1
        self._configured_ready = False

        self.get_logger().info('Limpieza completada.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._active = False

        if self._render_timer:
            self.destroy_timer(self._render_timer)
            self._render_timer = None

        with self._init_lock:
            if self._segmentator:
                self._segmentator.close()
                self._segmentator = None

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        return TransitionCallbackReturn.SUCCESS

    # ── Callbacks ──────────────────────────────────────────────────────

    def _lang_callback(self, msg: Bool):
        self._is_english = msg.data

    def _image_callback(self, msg: Image):
        if not self._active:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            frame = cv2.flip(frame, 0)
            with self._frame_lock:
                self._latest_frame = frame
        except Exception as e:
            self.get_logger().error(f'image_callback: {e}')

    # ── Render principal (llamado desde el Timer) ──────────────────────

    def _spin_render(self):
        """Llamado automáticamente por el Timer a ~30Hz."""
        if not self._active:
            return

        # Crear/verificar ventana
        try:
            visible = cv2.getWindowProperty(self._window_name, cv2.WND_PROP_VISIBLE)
            window_exists = visible >= 1
        except cv2.error:
            window_exists = False

        if not window_exists:
            # Si no existe la ventana, evaluamos por qué:
            if not getattr(self, '_window_created', False):
                # 1. Es la primera vez que arranca, la creamos normalmente
                cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
                cv2.setWindowProperty(self._window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                cv2.setMouseCallback(self._window_name, self._mouse_callback)
                self.get_logger().info('Ventana creada.')
                self._window_created = True
            else:
                # 2. Ya había sido creada, significa que el usuario pulsó la X
                self.get_logger().info('Ventana cerrada con X. Publicando idle...')
                self._window_created = False
                self._go_idle()
                return

        # Renderizar según estado
        if self._state == 'MENU':
            output = self._draw_menu(TARGET_W, TARGET_H)
        else:
            frame = None
            with self._frame_lock:
                if self._latest_frame is not None:
                    frame = self._latest_frame.copy()

            if frame is None:
                # Sin frame aún: pantalla de espera
                output = np.zeros((TARGET_H, TARGET_W, 3), dtype=np.uint8)
                output[:] = (15, 15, 25)
                cv2.putText(
                    output,
                    'Esperando camara...' if not self._is_english
                    else 'Waiting for camera...',
                    (TARGET_W // 2 - 160, TARGET_H // 2),
                    cv2.FONT_HERSHEY_DUPLEX, 0.9,
                    (100, 150, 200), 1, cv2.LINE_AA)
            elif not self._configured_ready or self._segmentator is None:
                # MediaPipe aún cargando: mostrar frame crudo + aviso
                output = cv2.resize(frame, (TARGET_W, TARGET_H))
                cv2.putText(
                    output,
                    'Cargando segmentacion...' if not self._is_english
                    else 'Loading segmentation...',
                    (20, 40),
                    cv2.FONT_HERSHEY_PLAIN, 1.2,
                    (0, 220, 255), 1, cv2.LINE_AA)
            else:
                output = self._apply_background(frame)
                output = cv2.resize(output, (TARGET_W, TARGET_H))

        cv2.imshow(self._window_name, output)
        key = cv2.waitKey(1)
        if key == 27:  # ESC
            if self._state == 'INMERSIVO':
                self._state = 'MENU'
            else:
                self._go_idle()

    # ── Lógica de fondos ───────────────────────────────────────────────

    def _load_backgrounds(self):
        try:
            pkg_dir     = get_package_share_directory('yaren_filters')
            folder_path = os.path.join(pkg_dir, 'fondos')
        except Exception as e:
            self.get_logger().error(f'Paquete no encontrado: {e}')
            return

        if not os.path.exists(folder_path):
            self.get_logger().warn(f"Carpeta '{folder_path}' no existe.")
            return

        exts = ['*.jpg', '*.jpeg', '*.png', '*.JPG', '*.JPEG', '*.PNG']
        files = []
        for ext in exts:
            files += glob.glob(os.path.join(folder_path, ext))
        files = sorted(set(files))

        for f in files:
            img = cv2.imread(f)
            if img is not None:
                self._bg_images.append(img)
                self._bg_names.append(os.path.splitext(os.path.basename(f))[0])
                thumb = cv2.resize(img, (self._thumb_w, self._thumb_h))
                self._thumbnails.append(thumb)

        self.get_logger().info(f'Cargados {len(self._bg_images)} fondos.')

    def _apply_background(self, frame: np.ndarray) -> np.ndarray:
        """Aplica el fondo virtual optimizado para alto rendimiento."""
        with self._init_lock:
            seg = self._segmentator

        if seg is None or not self._bg_images:
            return frame.copy()

        try:
            h, w = frame.shape[:2]

            # Caché del fondo redimensionado
            if self._cached_bg_index != self._bg_index or self._cached_bg is None or self._cached_bg.shape[:2] != (h, w):
                self._cached_bg = cv2.resize(self._bg_images[self._bg_index], (w, h))
                self._cached_bg_index = self._bg_index
            
            bg = self._cached_bg

            # Segmentación
            rgb     = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = seg.process(rgb)
            mask    = results.segmentation_mask  # float32 [0..1]

            # Optimización: np.where en lugar de operaciones float32
            condition = np.stack((mask,) * 3, axis=-1) > 0.5
            output = np.where(condition, frame, bg)

        except Exception as e:
            self.get_logger().error(f'apply_background: {e}')
            output = frame.copy()

        return output

    # ── Menú de selección ──────────────────────────────────────────────

    # ── Menú de selección (estilo videojuego infantil) ─────────────────

    HEADER_H = 106

    # Paletas BGR: (base, brillo, borde)
    _PAL_EXIT   = ((30, 120, 235), (90, 175, 255), (70, 215, 255))
    _PAL_RANDOM = ((225, 110, 40), (255, 175, 105), (255, 215, 160))
    _PAL_DONE   = ((50, 170, 30), (115, 220, 95), (160, 245, 140))

    @staticmethod
    def _rrect(img, x1, y1, x2, y2, r, color):
        """Rectángulo redondeado relleno."""
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        if x2 <= x1 or y2 <= y1:
            return
        r = int(max(0, min(r, (x2 - x1) // 2, (y2 - y1) // 2)))
        cv2.rectangle(img, (x1 + r, y1), (x2 - r, y2), color, -1)
        cv2.rectangle(img, (x1, y1 + r), (x2, y2 - r), color, -1)
        for cx, cy in ((x1 + r, y1 + r), (x2 - r, y1 + r),
                       (x1 + r, y2 - r), (x2 - r, y2 - r)):
            cv2.circle(img, (cx, cy), r, color, -1, cv2.LINE_AA)

    @staticmethod
    def _text_c(img, text, cx, cy, scale, color, outline, thick=1):
        """Texto centrado con contorno grueso (look de caricatura)."""
        font = cv2.FONT_HERSHEY_DUPLEX
        (tw, th), _ = cv2.getTextSize(text, font, scale, thick)
        org = (int(cx - tw / 2), int(cy + th / 2))
        cv2.putText(img, text, org, font, scale, outline, thick + 3, cv2.LINE_AA)
        cv2.putText(img, text, org, font, scale, color, thick, cv2.LINE_AA)

    @staticmethod
    def _sparkle(img, cx, cy, r, color):
        k = max(1, r // 4)
        pts = np.array([(cx, cy - r), (cx + k, cy - k), (cx + r, cy),
                        (cx + k, cy + k), (cx, cy + r), (cx - k, cy + k),
                        (cx - r, cy), (cx - k, cy - k)], np.int32)
        cv2.fillPoly(img, [pts], color, cv2.LINE_AA)

    @staticmethod
    def _blit(dst, src, x, y):
        """Copia src en dst recortando lo que se salga de la pantalla."""
        dh, dw = dst.shape[:2]
        sh, sw = src.shape[:2]
        x1, y1 = max(0, x), max(0, y)
        x2, y2 = min(dw, x + sw), min(dh, y + sh)
        if x2 > x1 and y2 > y1:
            dst[y1:y2, x1:x2] = src[y1 - y:y2 - y, x1 - x:x2 - x]

    def _button(self, img, rect, label, pal, pressed=False, scale=0.75):
        """Botón gordito con sombra, borde, brillo y efecto de presionado."""
        x, y, w, h = rect
        base, light, border = pal
        dy = 3 if pressed else 0
        dark = tuple(int(c * 0.40) for c in base)
        self._rrect(img, x, y + 6, x + w, y + h + 6, 15, (25, 12, 6))
        self._rrect(img, x, y + dy, x + w, y + h + dy, 15, border)
        self._rrect(img, x + 3, y + 3 + dy, x + w - 3, y + h - 3 + dy, 13, base)
        self._rrect(img, x + 7, y + 5 + dy, x + w - 7, y + h // 2 + dy, 10, light)
        self._text_c(img, label, x + w / 2, y + h / 2 + dy,
                     scale, (255, 255, 255), dark)

    def _layout_buttons(self, w):
        by, bh = 10, 44
        return {
            'exit':   (14, by, 118, bh),
            'random': ((w - 190) // 2, by, 190, bh),
            'done':   (w - 14 - 150, by, 150, bh),
        }

    def _menu_bg(self, w, h):
        """Fondo degradado + cabecera. Se genera una sola vez (caché)."""
        key = (w, h, self._is_english)
        c = self._menu_cache
        if c.get('key') == key:
            return c['bg'], c['header']

        top = np.array((72, 34, 14), np.float32)
        bot = np.array((36, 14, 6), np.float32)
        t = np.linspace(0, 1, h, dtype=np.float32)[:, None, None]
        bg = np.repeat(top * (1 - t) + bot * t, w, axis=1).astype(np.uint8)

        hd = self.HEADER_H
        header = np.zeros((hd, w, 3), np.uint8)
        header[:] = (125, 62, 24)
        header[64:hd] = (104, 50, 18)
        cv2.line(header, (0, 63), (w, 63), (150, 80, 30), 2)
        cv2.rectangle(header, (0, hd - 4), (w, hd), (232, 200, 79), -1)
        cv2.rectangle(header, (0, hd - 1), (w, hd), (60, 25, 8), -1)

        title = 'CHOOSE YOUR BACKGROUND!' if self._is_english else 'ELIGE TU FONDO!'
        scale = 0.85
        (tw, _), _ = cv2.getTextSize(title, cv2.FONT_HERSHEY_DUPLEX, scale, 2)
        self._text_c(header, title, w // 2, 83, scale,
                     (240, 215, 100), (60, 25, 8), 2)
        for sx, sy, sr in ((w // 2 - tw // 2 - 28, 80, 11),
                           (w // 2 + tw // 2 + 28, 80, 11),
                           (w // 2 - tw // 2 - 50, 90, 6),
                           (w // 2 + tw // 2 + 50, 90, 6)):
            self._sparkle(header, sx, sy, sr, (110, 230, 255))
        for x in range(w // 2 - 170, w // 2 + 171, 9):
            cv2.circle(header, (x, 99), 1, (170, 120, 70), -1, cv2.LINE_AA)

        self._menu_cache = {'key': key, 'bg': bg, 'header': header}
        return bg, header

    def _draw_menu(self, w: int, h: int) -> np.ndarray:
        bg, header = self._menu_bg(w, h)
        menu = bg.copy()
        hd = self.HEADER_H
        en = self._is_english
        btns = self._layout_buttons(w)

        # ── Pantalla de carga (solo botón salir) ──
        if not self._configured_ready:
            menu[:hd] = header
            self._btn_rects = {'exit': btns['exit']}
            self._exit_btn_rect = btns['exit']
            self._button(menu, btns['exit'], 'EXIT' if en else 'SALIR',
                         self._PAL_EXIT, self._pressed == 'exit')
            self._text_c(menu, 'Loading...' if en else 'Cargando...',
                         w // 2, (h + hd) // 2, 1.1,
                         (255, 230, 120), (60, 20, 5), 2)
            return menu

        # ── Geometría de la grilla ──
        cols, pad, gap, frame, label_h = 3, 14, 14, 6, 22
        panel_x1, panel_x2 = 10, w - 24
        card_w = (panel_x2 - panel_x1 - 2 * pad - gap * (cols - 1)) // cols
        thumb_w = card_w - 2 * frame
        thumb_h = int(thumb_w * 0.52)
        card_h = thumb_h + frame + label_h
        panel_h = card_h + 20
        stride = panel_h + 12

        if self._bg_images and (thumb_w != self._thumb_w or thumb_h != self._thumb_h):
            self._thumb_w, self._thumb_h = thumb_w, thumb_h
            self._thumbnails = [cv2.resize(im, (thumb_w, thumb_h))
                                for im in self._bg_images]

        n = len(self._thumbnails)
        rows = math.ceil(n / cols) if n else 0
        total_h = hd + 10 + rows * stride
        self._max_scroll = max(0, total_h - h + 6)
        self._scroll_y = max(0, min(self._max_scroll, self._scroll_y))

        # Paneles azules (una "bandeja" por fila)
        for r in range(rows):
            py1 = hd + 10 + r * stride - self._scroll_y
            if py1 + panel_h < hd or py1 > h:
                continue
            self._rrect(menu, panel_x1, py1 + 4, panel_x2, py1 + panel_h + 4, 18, (110, 55, 15))
            self._rrect(menu, panel_x1, py1, panel_x2, py1 + panel_h, 18, (205, 125, 40))
            self._rrect(menu, panel_x1 + 3, py1 + 3, panel_x2 - 3, py1 + panel_h - 3, 15, (245, 200, 90))

        # Tarjetas
        self._thumbnail_rects = []
        pulse = 0.5 + 0.5 * math.sin(time.time() * 5.0)
        for i, thumb in enumerate(self._thumbnails):
            r, c = divmod(i, cols)
            cx = panel_x1 + pad + c * (card_w + gap)
            cy = hd + 10 + r * stride + 10 - self._scroll_y
            self._thumbnail_rects.append((cx, cy, card_w, card_h))
            if cy + card_h < hd or cy > h:
                continue

            sel = (i == self._bg_index)
            pressed = (self._pressed == ('card', i))
            self._rrect(menu, cx, cy + 4, cx + card_w, cy + card_h + 4, 10, (170, 95, 25))
            if sel:
                self._rrect(menu, cx, cy, cx + card_w, cy + card_h, 10,
                            (int(70 * pulse), 225, 255))
                self._rrect(menu, cx + 3, cy + 3, cx + card_w - 3, cy + card_h - 3, 8,
                            (255, 255, 255))
            else:
                col = (255, 225, 110) if pressed else (255, 255, 255)
                self._rrect(menu, cx, cy, cx + card_w, cy + card_h, 10, col)
            self._blit(menu, thumb, cx + frame, cy + frame)

            name = self._bg_names[i] if i < len(self._bg_names) else str(i + 1)
            cv2.putText(menu, name.replace('_', ' ')[:24],
                        (cx + frame + 2, cy + frame + thumb_h + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (150, 70, 0) if sel else (90, 45, 15), 1, cv2.LINE_AA)

            if sel:  # insignia verde con palomita
                bx, by = cx + 4, cy + 4
                cv2.circle(menu, (bx, by), 14, (255, 255, 255), -1, cv2.LINE_AA)
                cv2.circle(menu, (bx, by), 11, (50, 175, 35), -1, cv2.LINE_AA)
                cv2.polylines(menu, [np.array([(bx - 6, by), (bx - 2, by + 5), (bx + 6, by - 5)], np.int32)],
                              False, (255, 255, 255), 3, cv2.LINE_AA)

        if n == 0:
            self._text_c(menu, 'No backgrounds found' if en else 'No hay fondos',
                         w // 2, (h + hd) // 2, 0.9, (200, 200, 230), (60, 20, 5), 1)

        # Cabecera y botones por encima de todo
        menu[:hd] = header
        self._btn_rects = btns
        self._exit_btn_rect = btns['exit']
        for key, pal, lbl in (('exit', self._PAL_EXIT, 'EXIT' if en else 'SALIR'),
                              ('random', self._PAL_RANDOM, 'RANDOM' if en else 'ALEATORIO'),
                              ('done', self._PAL_DONE, 'GO!' if en else 'LISTO!')):
            self._button(menu, btns[key], lbl, pal, self._pressed == key)

        # Scrollbar redondeada
        if self._max_scroll > 0:
            sx, sy0, sh = w - 17, hd + 8, h - hd - 16
            knob = max(30, int(sh * (h - hd) / max(1, total_h - hd)))
            ky = sy0 + int((sh - knob) * self._scroll_y / self._max_scroll)
            self._rrect(menu, sx, sy0, sx + 9, sy0 + sh, 4, (70, 35, 12))
            self._rrect(menu, sx, ky, sx + 9, ky + knob, 4, (255, 190, 70))

        return menu

    # ── Mouse callback ─────────────────────────────────────────────────

    def _hit(self, x, y):
        for key, (bx, by, bw, bh) in self._btn_rects.items():
            if bx <= x <= bx + bw and by <= y <= by + bh:
                return key
        if y >= self.HEADER_H and self._configured_ready:
            for i, (rx, ry, rw, rh) in enumerate(self._thumbnail_rects):
                if rx <= x <= rx + rw and ry <= y <= ry + rh:
                    return ('card', i)
        return None

    def _start_immersive(self, idx):
        self._bg_index = idx
        self._state = 'INMERSIVO'
        self.get_logger().info(f'Fondo #{idx} seleccionado → INMERSIVO')

    def _mouse_callback(self, event, x, y, flags, param):
        # Scroll con rueda
        if event == cv2.EVENT_MOUSEWHEEL:
            delta = 1 if flags > 0 else -1
            self._scroll_y = max(
                0, min(self._max_scroll, self._scroll_y - delta * 60))
            return

        if self._state == 'MENU':
            if event == cv2.EVENT_LBUTTONDOWN:
                self._is_dragging     = True
                self._drag_start_y    = y
                self._click_start_pos = (x, y)
                self._pressed         = self._hit(x, y)

            elif event == cv2.EVENT_MOUSEMOVE and self._is_dragging:
                if (abs(x - self._click_start_pos[0]) +
                        abs(y - self._click_start_pos[1])) >= 12:
                    self._pressed = None  # es un arrastre, no un clic
                dy = self._drag_start_y - y
                self._scroll_y = min(
                    self._max_scroll, max(0, self._scroll_y + dy))
                self._drag_start_y = y

            elif event == cv2.EVENT_LBUTTONUP:
                self._is_dragging = False
                self._pressed = None
                dist = (abs(x - self._click_start_pos[0]) +
                        abs(y - self._click_start_pos[1]))
                if dist >= 12:
                    return
                target = self._hit(x, y)
                if target == 'exit':
                    self._go_idle()
                elif not self._configured_ready or not self._bg_images:
                    return
                elif target == 'random':
                    pool = [i for i in range(len(self._bg_images))
                            if i != self._bg_index] or [self._bg_index]
                    self._start_immersive(random.choice(pool))
                elif target == 'done':
                    self._start_immersive(self._bg_index)
                elif isinstance(target, tuple):
                    self._start_immersive(target[1])

        elif self._state == 'INMERSIVO':
            # Cualquier clic primario regresa al menú
            if event == cv2.EVENT_LBUTTONDOWN:
                self._state = 'MENU'
                self.get_logger().info('Volviendo al menú de fondos.')

    # ── Utilidades ─────────────────────────────────────────────────────

    def _go_idle(self):
        """Publicar idle y desactivarse."""
        self._active = False
        try:
            cv2.destroyWindow(self._window_name)
        except Exception:
            pass
        if rclpy.ok():
            pub = self.create_publisher(String, '/yaren_mode', 10)
            msg = String()
            msg.data = 'idle'
            for _ in range(3):
                pub.publish(msg)
                time.sleep(0.05)
            self.get_logger().info('Publicado idle → face_screen.')


# ── Main ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = VirtualBackgroundNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
