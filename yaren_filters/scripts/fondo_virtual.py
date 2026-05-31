#!/usr/bin/env python3
"""
fondo_virtual.py  —  LifecycleNode
─────────────────────────────────────────────
on_configure  → carga fondos e inicializa MediaPipe (una sola vez en RAM)
on_activate   → suscribe a la cámara, lanza hilo de UI
on_deactivate → desuscribe, para hilo de UI, libera ventanas
on_cleanup    → libera MediaPipe y fondos
"""

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from rclpy.qos import qos_profile_sensor_data # <--- IMPORTANTE PARA LA CÁMARA
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import mediapipe as mp
import os
import glob
import math
import threading

TARGET_W = 800
TARGET_H = 480

class VirtualBackgroundNode(LifecycleNode):

    def __init__(self):
        # Este nombre DEBE coincidir con el que pusiste en face_screen.cpp
        super().__init__('virtual_background_node') 
        self._bridge      = CvBridge()
        self._segmentator = None   
        self._bg_images   = []
        self._thumbnails  = []
        self._is_english  = False

        self._lang_sub = None

        self._state           = 'MENU'
        self._bg_index        = 0
        self._thumb_w         = 240
        self._thumb_h         = 180
        self._scroll_y        = 0
        self._max_scroll      = 0
        self._is_dragging     = False
        self._drag_start_y    = 0
        self._click_start_pos = (0, 0)
        self._thumbnail_rects = []

        self._ui_thread  = None
        self._ui_running = False
        self._cam_sub    = None
        self._latest_frame = None
        self._frame_lock   = threading.Lock()

        self._window_name = 'YAREN - Modo Inmersivo'

    # ── Lifecycle callbacks ────────────────────────────────────────────

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        lang_qos = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        )
        self._lang_sub = self.create_subscription(
            Bool, '/yaren/is_english', self._lang_callback, lang_qos)

        mp_selfie = mp.solutions.selfie_segmentation
        self._segmentator = mp_selfie.SelfieSegmentation(model_selection=1)

        self._load_backgrounds()

        self.get_logger().info('Configurado. Fondos y MediaPipe listos en RAM.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # CORRECCIÓN: Usar qos_profile_sensor_data para conectar con csi_cam_pub.py
        self._cam_sub = self.create_subscription(
            Image, '/csi_camera/image_raw', self._image_callback, qos_profile_sensor_data)

        cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(self._window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.setWindowProperty(self._window_name, cv2.WND_PROP_TOPMOST, 1)
        cv2.setMouseCallback(self._window_name, self._mouse_callback)

        self._state      = 'MENU'
        self._scroll_y   = 0
        self._ui_running = True
        self._ui_thread  = threading.Thread(target=self._ui_loop, daemon=True)
        self._ui_thread.start()

        self.get_logger().info('ACTIVADO — mostrando menú de fondos.')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._ui_running = False
        if self._ui_thread and self._ui_thread.is_alive():
            self._ui_thread.join(timeout=2.0)

        if self._cam_sub:
            self.destroy_subscription(self._cam_sub)
            self._cam_sub = None

        cv2.destroyAllWindows()
        self.get_logger().info('DESACTIVADO.')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        if self._segmentator:
            self._segmentator.close()
            self._segmentator = None
        if self._lang_sub:
            self.destroy_subscription(self._lang_sub)
            self._lang_sub = None
        self._bg_images.clear()
        self._thumbnails.clear()
        self.get_logger().info('Limpieza completada.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._ui_running = False
        if self._ui_thread and self._ui_thread.is_alive():
            self._ui_thread.join(timeout=1.0)
        if self._segmentator:
            self._segmentator.close()
            self._segmentator = None
        cv2.destroyAllWindows()
        return TransitionCallbackReturn.SUCCESS

    # ── Suscriptor de idioma ───────────────────────────────────────────

    def _lang_callback(self, msg: Bool):
        self._is_english = msg.data

    # ── Carga de fondos ────────────────────────────────────────────────

    def _load_backgrounds(self):
        try:
            pkg_dir     = get_package_share_directory('yaren_filters')
            folder_path = os.path.join(pkg_dir, 'fondos')
        except Exception as e:
            self.get_logger().error(f'No se pudo encontrar el paquete: {e}')
            return

        if not os.path.exists(folder_path):
            self.get_logger().warn(f"Carpeta '{folder_path}' no existe.")
            bg = np.zeros((TARGET_H, TARGET_W, 3), dtype=np.uint8)
            bg[:] = (0, 255, 0)
            self._bg_images.append(bg)
            self._thumbnails.append(cv2.resize(bg, (self._thumb_w, self._thumb_h)))
            return

        files = glob.glob(os.path.join(folder_path, '*.[jp][pn]*[g]'))
        for f in sorted(files):
            img = cv2.imread(f)
            if img is not None:
                self._bg_images.append(img)
                self._thumbnails.append(cv2.resize(img, (self._thumb_w, self._thumb_h)))

        self.get_logger().info(f'Cargados {len(self._bg_images)} fondos.')

    # ── Callback de imagen ─────────────────────────────────────────────

    def _image_callback(self, msg: Image):
        if not self._ui_running:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            frame = cv2.flip(frame, 0)
            with self._frame_lock:
                self._latest_frame = frame
        except Exception as e:
            self.get_logger().error(f'Error en image_callback: {e}')

    # ── Hilo de UI ─────────────────────────────────────────────────────

    def _ui_loop(self):
        while self._ui_running:
            frame = None
            with self._frame_lock:
                if self._latest_frame is not None:
                    frame = self._latest_frame.copy()

            # CORRECCIÓN: Si estamos en el MENU, dibujarlo aunque la cámara no haya arrancado aún.
            if self._state == 'MENU':
                output = self._draw_menu(TARGET_W, TARGET_H)
            else:
                if frame is None:
                    cv2.waitKey(16)
                    continue
                output = self._apply_background(frame)
                output = cv2.resize(output, (TARGET_W, TARGET_H))

            cv2.imshow(self._window_name, output)
            cv2.setWindowProperty(self._window_name, cv2.WND_PROP_TOPMOST, 1)
            cv2.waitKey(16)

        self._publish_idle()

    def _apply_background(self, frame: np.ndarray) -> np.ndarray:
        try:
            bg = cv2.resize(
                self._bg_images[self._bg_index],
                (frame.shape[1], frame.shape[0]))
            rgb     = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self._segmentator.process(rgb)
            mask    = np.stack((results.segmentation_mask,) * 3, axis=-1) > 0.1
            output  = np.where(mask, frame, bg)
        except Exception:
            output = frame.copy()

        btn_lbl = 'BACK' if self._is_english else 'VOLVER'
        cv2.rectangle(output, (30, 30), (220, 100), (0, 160, 255), -1)
        cv2.putText(output, btn_lbl, (55, 75), cv2.FONT_HERSHEY_DUPLEX, 1.1, (255, 255, 255), 2)
        return output

    def _draw_menu(self, w: int, h: int) -> np.ndarray:
        menu = np.zeros((h, w, 3), dtype=np.uint8)
        menu[:] = (30, 20, 25)

        title = 'SELECT A BACKGROUND' if self._is_english else 'SELECCIONA UN FONDO'
        hint  = '(Drag to scroll)' if self._is_english else '(Arrastra para ver mas)'
        cv2.putText(menu, title, (w//2 - 220, 60), cv2.FONT_HERSHEY_DUPLEX, 1.2, (251, 64, 224), 2)
        cv2.putText(menu, hint, (w//2 - 180, 95), cv2.FONT_HERSHEY_PLAIN, 1.3, (200, 200, 200), 1)

        self._thumbnail_rects = []
        start_x, start_y = 120, 140
        gap_x,   gap_y   = 40, 40
        cols     = max(1, (w - 2*start_x) // (self._thumb_w + gap_x))
        if cols < 2: cols = 2 # forzar 2 columnas
        rows     = math.ceil(len(self._thumbnails) / cols)
        total_h  = start_y + rows * (self._thumb_h + gap_y)
        self._max_scroll = max(0, total_h - h + 120)

        for i, thumb in enumerate(self._thumbnails):
            row = i // cols
            col = i % cols
            x   = start_x + col * (self._thumb_w + gap_x)
            y   = start_y + row * (self._thumb_h + gap_y) - self._scroll_y

            if y + self._thumb_h > 0 and y < h:
                y1 = max(0, int(y))
                y2 = min(h, int(y + self._thumb_h))
                
                # CORRECCIÓN: Separado en dos líneas
                ty1 = y1 - int(y)
                ty2 = ty1 + (y2 - y1)
                
                if y2 > y1:
                    menu[y1:y2, x:x+self._thumb_w] = thumb[ty1:ty2, :]
                    cv2.rectangle(menu, (x, int(y)), (x+self._thumb_w, int(y+self._thumb_h)), (255, 255, 255), 3)

            self._thumbnail_rects.append((x, y, self._thumb_w, self._thumb_h))

        btn_y   = h - 90
        btn_lbl = 'EXIT' if self._is_english else 'SALIR'
        cv2.rectangle(menu, (30, btn_y), (220, btn_y+70), (50, 50, 200), -1)
        cv2.putText(menu, btn_lbl, (65, btn_y+45), cv2.FONT_HERSHEY_DUPLEX, 1.2, (255, 255, 255), 2)
        return menu

    # ── Mouse callback ─────────────────────────────────────────────────

    def _mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEWHEEL:
            delta = 1 if flags > 0 else -1
            self._scroll_y = max(0, min(self._max_scroll, self._scroll_y - delta * 80))
            return

        if self._state == 'MENU':
            if event == cv2.EVENT_LBUTTONDOWN:
                self._is_dragging     = True
                self._drag_start_y    = y
                self._click_start_pos = (x, y)

            elif event == cv2.EVENT_MOUSEMOVE and self._is_dragging:
                dy             = self._drag_start_y - y
                self._scroll_y = min(self._max_scroll, max(0, self._scroll_y + dy))
                self._drag_start_y = y

            elif event == cv2.EVENT_LBUTTONUP:
                self._is_dragging = False
                dist = abs(x - self._click_start_pos[0]) + abs(y - self._click_start_pos[1])
                if dist < 10:
                    btn_y = TARGET_H - 90
                    if 30 <= x <= 220 and btn_y <= y <= btn_y + 70:
                        self._ui_running = False
                        return
                    for i, (rx, ry, rw, rh) in enumerate(self._thumbnail_rects):
                        if rx <= x <= rx+rw and ry <= y <= ry+rh:
                            self._bg_index = i
                            self._state    = 'INMERSIVO'
                            return

        elif self._state == 'INMERSIVO':
            if event == cv2.EVENT_LBUTTONDOWN:
                btn_w = int(190 * (TARGET_W / 640.0))
                btn_h = int(70 * (TARGET_H / 480.0))
                if 30 <= x <= 30 + btn_w and 30 <= y <= 30 + btn_h:
                    self._state = 'MENU'

    def _publish_idle(self):
        if rclpy.ok():
            pub = self.create_publisher(String, '/yaren_mode', 10)
            msg = String()
            msg.data = 'idle'
            for _ in range(3):
                pub.publish(msg)
                import time; time.sleep(0.06)
            self.get_logger().info('Publicado idle → face_screen.')

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