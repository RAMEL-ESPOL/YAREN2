#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import mediapipe as mp
import os
import glob
import time
import signal
import math

class VirtualBackgroundNode(Node):
    def __init__(self):
        super().__init__('virtual_background_node')
        self.bridge = CvBridge()
        
        # Estados: 'MENU' o 'INMERSIVO'
        self.state = 'MENU'
        
        # Inicializar MediaPipe para segmentación
        self.mp_selfie = mp.solutions.selfie_segmentation
        self.segmentator = self.mp_selfie.SelfieSegmentation(model_selection=1)
        
        # Variables UI y Tamaños (Escalado para que todo se vea grande)
        self.bg_images = []
        self.thumbnails = []
        self.thumbnail_rects = [] 
        self.bg_index = 0
        self.thumb_w, self.thumb_h = 320, 240 # Miniaturas gigantes
        self.screen_w, self.screen_h = 640, 480 # Valores por defecto
        
        # Variables de Scroll
        self.scroll_y = 0
        self.max_scroll = 0
        self.is_dragging = False
        self.drag_start_y = 0
        self.click_start_pos = (0, 0)
        
        # Configurar la ventana de OpenCV en PANTALLA COMPLETA
        self.window_name = 'YAREN - Modo Inmersivo'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        
        self.load_backgrounds()
        
        # Suscripción a la cámara CSI
        self.subscription = self.create_subscription(
            Image, '/csi_camera/image_raw', self.image_callback, 10)
        
        # Publicador de la imagen
        self.publisher = self.create_publisher(Image, '/yaren_screen/virtual_bg', 10)
        
        self.get_logger().info("Nodo Iniciado en Pantalla Completa. Scroll Activado.")

    def load_backgrounds(self):
        try:
            pkg_share_dir = get_package_share_directory('yaren_filters')
            folder_path = os.path.join(pkg_share_dir, 'fondos')
        except Exception as e:
            self.get_logger().error(f"No se pudo encontrar el paquete: {e}")
            return

        if not os.path.exists(folder_path):
            self.get_logger().warn(f"La carpeta '{folder_path}' no existe.")
            return
        
        files = glob.glob(os.path.join(folder_path, '*.[jp][pn]*[g]'))
        
        for file in files:
            img = cv2.imread(file)
            if img is not None:
                self.bg_images.append(img)
                thumb = cv2.resize(img, (self.thumb_w, self.thumb_h))
                self.thumbnails.append(thumb)
                
        if len(self.bg_images) > 0:
            self.get_logger().info(f"Se cargaron {len(self.bg_images)} fondos.")
        else:
            green_bg = np.zeros((480, 640, 3), dtype=np.uint8)
            green_bg[:] = (0, 255, 0)
            self.bg_images.append(green_bg)
            self.thumbnails.append(cv2.resize(green_bg, (self.thumb_w, self.thumb_h)))

    def kill_everything(self):
        self.get_logger().info("Ejecutando Secuencia de Apagado...")
        
        # 1. Apagar la cámara CSI correctamente
        # Reemplaza 'self.cap' con el nombre de tu variable de VideoCapture
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("Cámara CSI liberada.")

        # 2. Destruye ventanas para liberar la pantalla inmediatamente
        cv2.destroyAllWindows()
        
        # 3. Libera memoria de MediaPipe
        self.segmentator.close()
        
        # 4. Espera 1 segundo
        time.sleep(1.0)
        
        # 5. Matar SOLO este proceso (sin afectar el Launch ni otros Pythons)
        try:
            # os.getpid() apunta solo a ESTE script, no al grupo
            os.kill(os.getpid(), signal.SIGKILL)
        except Exception as e:
            self.get_logger().error(f"Falla al matar el proceso: {e}")

    def mouse_callback(self, event, x, y, flags, param):
        # ── Scroll con la Rueda del Ratón ──
        if event == cv2.EVENT_MOUSEWHEEL:
            if flags > 0: # Arriba
                self.scroll_y = max(0, self.scroll_y - 80)
            else:         # Abajo
                self.scroll_y = min(self.max_scroll, self.scroll_y + 80)
            return

        # ── MÁQUINA DE ESTADOS CLIC ──
        if self.state == 'MENU':
            if event == cv2.EVENT_LBUTTONDOWN:
                self.is_dragging = True
                self.drag_start_y = y
                self.click_start_pos = (x, y)
                
            elif event == cv2.EVENT_MOUSEMOVE:
                if self.is_dragging:
                    dy = self.drag_start_y - y
                    self.scroll_y = min(self.max_scroll, max(0, self.scroll_y + dy))
                    self.drag_start_y = y # Scroll continuo
                    
            elif event == cv2.EVENT_LBUTTONUP:
                self.is_dragging = False
                dist = abs(x - self.click_start_pos[0]) + abs(y - self.click_start_pos[1])
                
                if dist < 10: # Fue un clic normal, no un arrastre
                    # 1. Comprobar Botón "SALIR"
                    btn_y = self.screen_h - 90
                    if 30 <= x <= 220 and btn_y <= y <= btn_y + 70:
                        self.kill_everything()
                        return

                    # 2. Comprobar Miniaturas
                    for i, (rx, ry, rw, rh) in enumerate(self.thumbnail_rects):
                        if rx <= x <= rx + rw and ry <= y <= ry + rh:
                            self.bg_index = i
                            self.state = 'INMERSIVO'
                            return

        elif self.state == 'INMERSIVO':
            if event == cv2.EVENT_LBUTTONDOWN:
                # Comprobar Botón "VOLVER"
                if 30 <= x <= 220 and 30 <= y <= 100:
                    self.state = 'MENU'

    def draw_menu(self, w, h):
        menu_canvas = np.zeros((h, w, 3), dtype=np.uint8)
        menu_canvas[:] = (30, 20, 25)
        
        cv2.putText(menu_canvas, "SELECCIONA UN FONDO", (w//2 - 220, 60), 
                    cv2.FONT_HERSHEY_DUPLEX, 1.2, (251, 64, 224), 2)
        cv2.putText(menu_canvas, "(Arrastra la pantalla para ver mas)", (w//2 - 180, 95), 
                    cv2.FONT_HERSHEY_PLAIN, 1.3, (200, 200, 200), 1)
        
        self.thumbnail_rects = []
        start_x, start_y = 60, 140
        gap_x, gap_y = 40, 40
        
        # Calcular cuadricula y máximo scroll
        cols = max(1, (w - 2 * start_x) // (self.thumb_w + gap_x))
        rows = math.ceil(len(self.thumbnails) / cols)
        total_h = start_y + rows * (self.thumb_h + gap_y)
        self.max_scroll = max(0, total_h - h + 120)
        
        for i, thumb in enumerate(self.thumbnails):
            row = i // cols
            col = i % cols
            
            x = start_x + col * (self.thumb_w + gap_x)
            y = start_y + row * (self.thumb_h + gap_y) - self.scroll_y
            
            # Dibujar solo si es visible en pantalla
            if y + self.thumb_h > 0 and y < h:
                y1 = max(0, int(y))
                y2 = min(h, int(y + self.thumb_h))
                thumb_y1 = y1 - int(y)
                thumb_y2 = thumb_y1 + (y2 - y1)
                
                if y2 > y1:
                    menu_canvas[y1:y2, x:x+self.thumb_w] = thumb[thumb_y1:thumb_y2, :]
                    cv2.rectangle(menu_canvas, (x, int(y)), (x+self.thumb_w, int(y+self.thumb_h)), (255, 255, 255), 3)
            
            # Guardar siempre la posición rectificada para el clic
            self.thumbnail_rects.append((x, y, self.thumb_w, self.thumb_h))
                
        # ── Botón Fijo de SALIR ──
        # Se dibuja al final para que quede sobre las imágenes
        btn_y = h - 90
        cv2.rectangle(menu_canvas, (30, btn_y), (220, btn_y + 70), (50, 50, 200), -1)
        cv2.putText(menu_canvas, "SALIR", (65, btn_y + 45), cv2.FONT_HERSHEY_DUPLEX, 1.2, (255, 255, 255), 2)
        
        return menu_canvas

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = cv2.flip(frame, 0)
            self.screen_h, self.screen_w, _ = frame.shape
            
            if self.state == 'MENU':
                output_image = self.draw_menu(self.screen_w, self.screen_h)
                
            elif self.state == 'INMERSIVO':
                current_bg = self.bg_images[self.bg_index]
                bg_resized = cv2.resize(current_bg, (self.screen_w, self.screen_h))
                
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = self.segmentator.process(frame_rgb)
                
                mask = np.stack((results.segmentation_mask,) * 3, axis=-1) > 0.1
                output_image = np.where(mask, frame, bg_resized)
                
                # ── Botón de VOLVER ──
                cv2.rectangle(output_image, (30, 30), (220, 100), (0, 160, 255), -1)
                cv2.putText(output_image, "VOLVER", (55, 75), cv2.FONT_HERSHEY_DUPLEX, 1.1, (255, 255, 255), 2)
                
                out_msg = self.bridge.cv2_to_imgmsg(output_image, encoding='bgr8')
                self.publisher.publish(out_msg)

            cv2.imshow(self.window_name, output_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error procesando frame: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VirtualBackgroundNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
