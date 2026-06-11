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
            visible = cv2.getWindowProperty(
                self._window_name, cv2.WND_PROP_VISIBLE)
            window_exists = visible >= 1
        except cv2.error:
            window_exists = False

        if not window_exists:
            cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
            cv2.setWindowProperty(
                self._window_name, cv2.WND_PROP_FULLSCREEN,
                cv2.WINDOW_FULLSCREEN)
            cv2.setMouseCallback(self._window_name, self._mouse_callback)
            self.get_logger().info('Ventana creada.')

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

    def _draw_menu(self, w: int, h: int) -> np.ndarray:
        menu = np.zeros((h, w, 3), dtype=np.uint8)
        menu[:] = (22, 15, 30)

        # Gradiente sutil de fondo
        for y in range(h):
            alpha = y / h * 0.3
            menu[y] = np.clip(
                menu[y].astype(np.float32) + alpha * np.array([10, 5, 20]),
                0, 255).astype(np.uint8)

        # Aviso si MediaPipe aún está cargando
        if not self._configured_ready:
            cv2.putText(menu,
                        'Cargando...' if not self._is_english else 'Loading...',
                        (w // 2 - 60, h // 2),
                        cv2.FONT_HERSHEY_DUPLEX, 1.0,
                        (0, 200, 255), 1, cv2.LINE_AA)
            return menu

        # Título
        title = ('SELECT A BACKGROUND' if self._is_english
                 else 'SELECCIONA UN FONDO')
        hint  = ('(Click to select, drag to scroll)'
                 if self._is_english
                 else '(Click para seleccionar, arrastra para ver mas)')
        cv2.putText(menu, title,
                    (w // 2 - 230, 42),
                    cv2.FONT_HERSHEY_DUPLEX, 1.1,
                    (220, 60, 255), 2, cv2.LINE_AA)
        cv2.line(menu, (w // 2 - 230, 52), (w // 2 + 230, 52),
                 (80, 20, 100), 1, cv2.LINE_AA)
        cv2.putText(menu, hint,
                    (w // 2 - 230, 72),
                    cv2.FONT_HERSHEY_PLAIN, 1.0,
                    (130, 100, 150), 1, cv2.LINE_AA)

        # Botón SALIR
        ex, ey, ew, eh = 15, 10, 125, 45
        self._exit_btn_rect = (ex, ey, ew, eh)
        cv2.rectangle(menu, (ex, ey), (ex + ew, ey + eh),
                      (30, 30, 60), -1)
        cv2.rectangle(menu, (ex, ey), (ex + ew, ey + eh),
                      (80, 80, 160), 1, cv2.LINE_AA)
        lbl_exit = 'EXIT' if self._is_english else 'SALIR'
        cv2.putText(menu, lbl_exit,
                    (ex + 22, ey + 30),
                    cv2.FONT_HERSHEY_DUPLEX, 0.80,
                    (180, 180, 220), 1, cv2.LINE_AA)

        # Grid de miniaturas
        self._thumbnail_rects = []
        cols    = 3
        pad_x   = 25
        pad_top = 90
        gap_x   = 18
        gap_y   = 18

        thumb_w = (w - pad_x * 2 - gap_x * (cols - 1)) // cols
        thumb_h = int(thumb_w * 0.60)

        rows    = math.ceil(max(1, len(self._thumbnails)) / cols)
        total_h = pad_top + rows * (thumb_h + gap_y)
        self._max_scroll = max(0, total_h - h + 20)

        # Regenerar thumbs si el tamaño cambió
        if (self._bg_images and
                (thumb_w != self._thumb_w or thumb_h != self._thumb_h)):
            self._thumb_w = thumb_w
            self._thumb_h = thumb_h
            self._thumbnails = [
                cv2.resize(img, (thumb_w, thumb_h))
                for img in self._bg_images
            ]

        # Obtener lista de archivos para nombres
        try:
            pkg_dir     = get_package_share_directory('yaren_filters')
            folder_path = os.path.join(pkg_dir, 'fondos')
            exts = ['*.jpg', '*.jpeg', '*.png', '*.JPG', '*.JPEG', '*.PNG']
            all_files = []
            for ext in exts:
                all_files += glob.glob(os.path.join(folder_path, ext))
            all_files = sorted(set(all_files))
        except Exception:
            all_files = []

        for i, thumb in enumerate(self._thumbnails):
            row = i // cols
            col = i % cols
            x   = pad_x + col * (thumb_w + gap_x)
            y   = pad_top + row * (thumb_h + gap_y) - self._scroll_y

            y1  = max(0, int(y))
            y2  = min(h, int(y + thumb_h))
            ty1 = y1 - int(y)
            ty2 = ty1 + (y2 - y1)

            if y2 > y1 and 0 <= ty1 < ty2 <= thumb_h:
                region = menu[y1:y2, x:x + thumb_w]
                src    = thumb[ty1:ty2, :]
                if region.shape == src.shape:
                    menu[y1:y2, x:x + thumb_w] = src

                is_selected = (i == self._bg_index)
                color = (0, 229, 255) if is_selected else (160, 160, 160)
                thick = 3 if is_selected else 1
                cv2.rectangle(menu,
                              (x - 1, int(y) - 1),
                              (x + thumb_w + 1, int(y + thumb_h) + 1),
                              color, thick, cv2.LINE_AA)

                # Nombre del archivo debajo de la miniatura
                if i < len(all_files):
                    name = os.path.splitext(
                        os.path.basename(all_files[i]))[0]
                else:
                    name = str(i)
                cv2.putText(menu, name[:18],
                            (x + 4, min(h - 4, int(y + thumb_h) + 14)),
                            cv2.FONT_HERSHEY_PLAIN, 0.85,
                            (180, 180, 200) if not is_selected
                            else (0, 229, 255),
                            1, cv2.LINE_AA)

            self._thumbnail_rects.append((x, y, thumb_w, thumb_h))

        # Scrollbar lateral
        if self._max_scroll > 0:
            sb_x  = w - 10
            sb_y0 = pad_top
            sb_h  = h - pad_top - 20
            fill  = max(20, int(sb_h * h / total_h))
            fill_y = sb_y0 + int(
                (sb_h - fill) * self._scroll_y / self._max_scroll)
            cv2.rectangle(menu,
                          (sb_x, sb_y0), (sb_x + 6, sb_y0 + sb_h),
                          (40, 40, 55), -1)
            cv2.rectangle(menu,
                          (sb_x, fill_y), (sb_x + 6, fill_y + fill),
                          (120, 60, 200), -1, cv2.LINE_AA)

        return menu

    # ── Mouse callback ─────────────────────────────────────────────────

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

            elif event == cv2.EVENT_MOUSEMOVE and self._is_dragging:
                dy = self._drag_start_y - y
                self._scroll_y = min(
                    self._max_scroll, max(0, self._scroll_y + dy))
                self._drag_start_y = y

            elif event == cv2.EVENT_LBUTTONUP:
                self._is_dragging = False
                dist = (abs(x - self._click_start_pos[0]) +
                        abs(y - self._click_start_pos[1]))
                if dist < 12:
                    # Botón SALIR
                    ex, ey, ew, eh = self._exit_btn_rect
                    if ex <= x <= ex + ew and ey <= y <= ey + eh:
                        self._go_idle()
                        return
                    # Miniaturas — solo si MediaPipe ya cargó
                    if self._configured_ready:
                        for i, (rx, ry, rw, rh) in enumerate(
                                self._thumbnail_rects):
                            if rx <= x <= rx + rw and ry <= y <= ry + rh:
                                self._bg_index = i
                                self._state    = 'INMERSIVO'
                                self.get_logger().info(
                                    f'Fondo #{i} seleccionado → INMERSIVO')
                                return

        elif self._state == 'INMERSIVO':
            # Ya no hay botón, cualquier clic primario regresa al menú
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
