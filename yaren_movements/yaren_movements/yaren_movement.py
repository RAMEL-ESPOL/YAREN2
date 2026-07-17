#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import time
import threading
import cv2
import numpy as np

class YarenMotionControl(Node):
    def __init__(self):
        super().__init__('yaren_motion_control')
        
        self.publisher = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10)
        
        self.joint_names = [
            "joint_1", "joint_2", "joint_3", "joint_4", 
            "joint_5", "joint_6", "joint_7", "joint_8", 
            "joint_9", "joint_10", "joint_11", "joint_12"
        ]
        
        # 1. Definimos la rutina cruda (Rutina 8)
        raw_routine = [
            [0.0, 0.0, 0, 0.02, 0.04, 0.1, 0.1, -1.1, -0.18, 0.1, -0.17, -1.1],
            [0.17, 0.0, -0.0, 0.22, 0.46, -0.23, -1.76, 0.31, -0.55, -0.24, 1.76, 0.34],
            [0.24, 0.0, 0.08, 0.12, 1.64, 0.54, 0.5, -0.85, -1.71, 0.69, -0.72, -1.15],
            [0.34, 0.0, -0.01, 0.07, 0.03, -0.13, -0.37, -1.78, 0.01, -0.25, -0.01, -1.98],
            [0.23, 0.0, 0.08, 0.13, 0.21, -0.13, -0.01, -2.49, -4.67, -0.13, 4.99, -2.53],
            [0.23, 0.0, 0.08, 0.13, 0.21, -0.13, -0.01, -2.49, -4.67, 1.05, 4.97, -2.53],
            [0.23, 0.0, 0.08, 0.13, 0.21, -0.13, -0.01, -2.49, -4.67, 0.07, 4.97, -2.53],
            [0.23, 0.0, 0.08, 0.13, 4.85, 0.73, -2.63, -0.42, -4.67, 0.63, 2.28, -0.65],
            [0.24, 0.0, -0.54, 0.16, 4.85, 0.73, -2.63, -0.42, -4.67, 0.63, 2.28, -0.65],
            [0.24, 0.0, 0.54, 0.23, 4.85, 0.73, -2.63, -0.42, -4.67, 0.63, 2.28, -0.65],
            [0.24, 0.0, 0.02, 0.14, 4.85, 0.73, -2.63, -0.42, -4.67, 0.63, 2.28, -0.65],
            [0.24, 0.0, 0.02, 0.14, 2.53, -0.02, 5.0, -0.39, -2.16, -0.07, -0.06, 0.34],
            [0.24, 0.0, 0.02, 0.14, 2.53, -0.06, -3.92, -0.39, -2.16, -0.06, 1.4, 0.34],
            [0.24, 0.0, 0.02, 0.15, 2.53, -0.05, 3.93, -0.39, -2.16, -0.17, -1.01, 0.34],   
            [0.24, 0.0, 0.02, 0.14, 1.67, -0.34, -0.38, -1.46, -1.73, -0.54, 0.03, -1.33],
            [-1.21, 0.0, -0.01, 0.12, 0.03, -0.13, -0.37, -1.78, 0.01, -0.25, -0.01, -1.98],
            [-1.21, 0.0, -0.8, -0.06, 0.03, -0.13, -0.37, -1.78, 0.01, -0.25, -0.01, -1.98],
            [-1.21, 0.0, -0.8, -0.06, 0.03, -0.13, -0.37, -1.78, 0.01, -0.25, -0.01, -1.98],
            [-1.21, 0.0, -0.03, 0.64, 0.03, -0.13, -0.37, -1.78, 0.01, -0.25, -0.01, -1.98],
            [0.34, 0.0, -0.01, 0.07, 0.03, -0.13, -0.37, -1.78, 0.01, -0.25, -0.01, -1.98]
        ]
        
        # 2. Procesamos la rutina con la fórmula matemática (tiempo por defecto = 1.0)
        self.processed_routine = self.from_gui(raw_routine, 2.5)
        
        # Posición segura por defecto al detenerse
        self.home_position = self.processed_routine[0][0]
        
        # Lógica de hilos
        self.is_running = False
        self.routine_thread = None
        
        # UI State (Resolución 800x480)
        self.W, self.H = 800, 480
        self.status_msg = "RUTINA 8 LISTA..."
        
        # Botones: [x, y, w, h]
        self.btn_stop_rect = [self.W//2 - 120, self.H//2 - 20, 240, 50]
        self.btn_exit_rect = [self.W//2 - 120, self.H//2 + 60, 240, 50]
        
        self.hover_stop = False
        self.hover_exit = False
        self.exit_requested = False

        self.get_logger().info("Nodo Yaren Listo. UI OpenCV Iniciada para Rutina 8.")

    def from_gui(self, poses_gui, t=1.0):
        """Convierte poses capturadas al sistema de radianes."""
        K = math.pi / 5
        OFFSET_CODO = math.pi - 1.75  # ≈ 1.3916

        result = []
        for pos in poses_gui:
            converted = []
            for idx, val in enumerate(pos):
                if idx == 7 or idx == 11:  # joint_8 y joint_12
                    converted.append(round(val * K + OFFSET_CODO, 4))
                else:
                    converted.append(round(val * K, 4))
            result.append((converted, t))
        return result

    def send_movement(self, positions, seconds):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        
        msg.points.append(point)
        self.publisher.publish(msg)

    def routine_loop(self):
        while self.is_running:
            for pos, t in self.processed_routine:
                if not self.is_running:
                    break
                self.send_movement(pos, t)
                time.sleep(t)

    def start_routine(self):
        if not self.is_running:
            self.is_running = True
            self.status_msg = "REPRODUCIENDO RUTINA 8 EN CICLOS..."
            self.routine_thread = threading.Thread(target=self.routine_loop)
            self.routine_thread.start()

    def stop_and_home(self):
        if self.is_running:
            self.is_running = False
            self.status_msg = "DETENIENDO... VOLVIENDO A ORIGEN."
            if self.routine_thread is not None:
                self.routine_thread.join()
            
            # Movimiento suave a home (estado original)
            self.send_movement(self.home_position, 2.0)
            self.status_msg = "RUTINA DETENIDA. (ESTADO ORIGEN)"

    # =========================================================================
    #  OpenCV UI Logic
    # =========================================================================
    
    def rect_contains(self, rect, x, y):
        rx, ry, rw, rh = rect
        return (rx <= x <= rx + rw) and (ry <= y <= ry + rh)

    def handleMouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            self.hover_stop = self.rect_contains(self.btn_stop_rect, x, y)
            self.hover_exit = self.rect_contains(self.btn_exit_rect, x, y)
            
        elif event == cv2.EVENT_LBUTTONDOWN:
            if self.hover_stop and self.is_running:
                threading.Thread(target=self.stop_and_home).start()
            
            if self.hover_exit:
                self.exit_requested = True

    def drawFlatButton(self, frame, rect, label, accent_color, hovered, disabled=False):
        x, y, w, h = rect
        
        if disabled:
            bg_color = (30, 30, 35)      
            border_color = (60, 60, 60)
            text_color = (100, 100, 100)
        else:
            bg_color = (int(accent_color[0]*0.2), int(accent_color[1]*0.2), int(accent_color[2]*0.2)) if hovered else (35, 20, 16)
            border_color = accent_color if hovered else (int(accent_color[0]*0.4), int(accent_color[1]*0.4), int(accent_color[2]*0.4))
            text_color = (255, 255, 255) if hovered else (190, 175, 170)

        cv2.rectangle(frame, (x, y), (x+w, y+h), bg_color, cv2.FILLED)
        cv2.rectangle(frame, (x, y), (x+w, y+h), border_color, 2 if hovered else 1, cv2.LINE_AA)
        
        txt_sz, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_DUPLEX, 0.6, 1)
        tx = x + (w - txt_sz[0]) // 2
        ty = y + (h + txt_sz[1]) // 2
        cv2.putText(frame, label, (tx, ty), cv2.FONT_HERSHEY_DUPLEX, 0.6, text_color, 1, cv2.LINE_AA)

    def render(self):
        frame = np.zeros((self.H, self.W, 3), dtype=np.uint8)
        frame[:] = (30, 15, 20) 
        
        color_status = (0, 255, 0) if self.is_running else (0, 160, 255) 
        txt_sz, _ = cv2.getTextSize(self.status_msg, cv2.FONT_HERSHEY_DUPLEX, 0.8, 2)
        cv2.putText(frame, self.status_msg, ((self.W - txt_sz[0])//2, 100), 
                    cv2.FONT_HERSHEY_DUPLEX, 0.8, color_status, 2, cv2.LINE_AA)
        
        cv2.line(frame, (self.W//2 - 250, 130), (self.W//2 + 250, 130), (85, 45, 0), 1, cv2.LINE_AA)

        self.drawFlatButton(frame, self.btn_stop_rect, "DETENER RUTINA", 
                            accent_color=(60, 60, 255), 
                            hovered=self.hover_stop, 
                            disabled=not self.is_running)

        self.drawFlatButton(frame, self.btn_exit_rect, "SALIR", 
                            accent_color=(150, 150, 150), 
                            hovered=self.hover_exit)

        return frame

def main(args=None):
    rclpy.init(args=args)
    node = YarenMotionControl()
    
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    node.start_routine()

    window_name = "Yaren Routine 8"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.setMouseCallback(window_name, node.handleMouse)

    try:
        while rclpy.ok() and not node.exit_requested:
            frame = node.render()
            cv2.imshow(window_name, frame)
            
            key = cv2.waitKey(30)
            if key == 27: # ESC
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.is_running = False
        node.stop_and_home()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()