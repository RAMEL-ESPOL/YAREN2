#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import time
import threading
import cv2
import numpy as np

TIME_SCALE = 1.0     # >1 = mas lento, <1 = mas rapido
START_DELAY = 2.0    # s para llegar a la 1ra pose desde donde este


def smooth_points(routine, scale=TIME_SCALE, start_delay=START_DELAY, min_dt=0.1):
    """routine = [(pose, t)]; t = tiempo para llegar a esa pose desde la anterior.
    Devuelve (puntos, duracion_total) con velocidades continuas: el controlador
    hace spline cubica y NO frena en cada pose (si el joint invierte direccion
    o la pose se repite, velocidad 0 para evitar rebotes)."""
    import math as _math
    P = [[float(x) for x in pos] for pos, _ in routine]
    n, J = len(P), len(P[0])
    T, acc = [], 0.0
    for i, (_, t) in enumerate(routine):
        dt = max(float(t) * scale, min_dt)
        if i == 0:
            dt = max(dt, start_delay)
        acc += dt
        T.append(acc)
    points = []
    for i in range(n):
        vel = [0.0] * J
        if 0 < i < n - 1:
            for j in range(J):
                s0 = (P[i][j] - P[i - 1][j]) / (T[i] - T[i - 1])
                s1 = (P[i + 1][j] - P[i][j]) / (T[i + 1] - T[i])
                if s0 * s1 > 0:
                    v = (s0 + s1) / 2.0
                    lim = 3.0 * min(abs(s0), abs(s1))
                    vel[j] = _math.copysign(min(abs(v), lim), v)
        pt = JointTrajectoryPoint()
        pt.positions = P[i]
        pt.velocities = vel
        pt.time_from_start = Duration(sec=int(T[i]), nanosec=int((T[i] % 1) * 1e9))
        points.append(pt)
    return points, T[-1]


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
        
        # 1. Definimos la rutina cruda
        raw_routine = [
            ([0.0, 0.0, 0, 0.02, 0.04, 0.1, 0.1, -1.1, -0.18, 0.1, -0.17, -1.1], 2.0),
            ([0.0, 0.0, 0, 0.02, 0.04, 0.1, 0.1, -1.1, -0.18, 0.1, -0.17, -1.1], 2.0),
            ([0.0, 0.0, 0, 0.02, 0.04, 0.1, 0.1, -1.1, -0.18, 0.1, -0.17, -1.1], 2.0),
            ([0.0, 0.0, 0.0, 0.08, 4.64, -0.18, -4.73, -2.51, -0.18, 0.07, -0.17, -1.11], 2.0),
            ([0.0, 0.0, 0.0, 0.08, 4.64, -0.18, -4.73, -2.51, -0.18, 0.07, -0.17, -1.11], 0.5),
            ([0.0, 0.0, 0.0, 0.08, 4.65, 1.07, -4.72, -2.51, -0.18, 0.07, -0.17, -1.11], 0.5),
            ([0.0, 0.0, 0.0, 0.08, 4.65, 1.07, -4.72, -2.51, -0.18, 0.07, -0.17, -1.11], 0.5),
            ([0.0, 0.0, -0.0, 0.08, 4.64, -0.16, -4.74, -2.51, -0.18, 0.07, -0.17, -1.11], 0.5),
            ([0.0, 0.0, -0.0, 0.08, 4.64, -0.16, -4.74, -2.51, -0.18, 0.07, -0.17, -1.11], 1.0),
            ([0.0, 0.0, 0.0, 0.08, 4.65, 1.07, -4.72, -2.51, -0.18, 0.07, -0.17, -1.11], 0.5),
            ([0.0, 0.0, 0.0, 0.08, 4.65, 1.07, -4.72, -2.51, -0.18, 0.07, -0.17, -1.11], 0.5),
            ([0.0, 0.0, -0.0, 0.08, 4.64, -0.16, -4.74, -2.51, -0.18, 0.07, -0.17, -1.11], 0.5),
            ([0.0, 0.0, -0.0, 0.08, 4.64, -0.16, -4.74, -2.51, -0.18, 0.07, -0.17, -1.11], 0.5),
            ([0.0, 0.0, -0.0, 0.08, 2.35, -0.18, -2.44, -2.51, -0.18, 0.07, -0.17, -1.11], 1.0),
            ([0.0, 0.0, -0.0, 0.08, 2.35, -0.18, -2.44, -2.51, -0.18, 0.07, -0.17, -1.11], 1.0),
            ([0.0, 0.0, -0.0, 0.08, 2.35, -0.18, -2.44, -2.51, -0.18, 0.07, -0.17, -1.11], 1.0),
            ([0.0, 0.0, 0.0, 0.08, 2.50, -0.18, -2.44, -2.51, -0.18, 0.07, -0.17, -1.11], 1.0),
            ([0.0, 0.0, 0.0, 0.08, 2.50, -0.18, -2.44, -2.51, -0.18, 0.07, -0.17, -1.11], 1.0),
            ([0.0, 0.0, -0.0, 0.08, 2.35, -0.17, -2.44, -2.51, -0.18, 0.07, -0.17, -1.11], 2.0),
            ([0.0, 0.0, 0.0, 0.1, 2.66, -0.17, -1.52, 0.2, -2.8, -0.2, 1.65, 0.05], 4.0),
            ([0.01, 0.0, 0.0, 0.2, 1.96, 0.28, -1.72, 0.22, -2.13, -0.24, 1.75, -0.31], 2.0),
            ([0.01, 0.0, 0.0, 0.2, 1.96, 0.28, -1.72, 0.22, -2.13, -0.24, 1.75, -0.31], 2.0),
            ([0.01, 0.0, 0.0, 0.2, 1.96, 0.28, -1.72, -1.5, -2.11, -0.24, 1.76, -1.86], 2.0),
            ([0.01, 0.0, 0.0, 0.2, 1.96, 0.28, -1.72, 0.22, -2.13, -0.24, 1.75, -0.31], 2.0),
            ([0.01, 0.0, 0.0, 0.2, 1.96, 0.28, -1.72, 0.22, -2.13, -0.24, 1.75, -0.31], 2.0),
            ([0.01, 0.0, 0.0, 0.2, 1.96, 0.28, -1.72, -1.5, -2.11, -0.24, 1.76, -1.86], 2.0),
            ([0.0, 0.0, 0.0, 0.17, 2.77, 0.66, -2.81, -1.73, -2.81, 1.02, 2.81, -1.31], 2.0),
            ([0.0, 0.0, 0.0, 0.17, 2.77, 0.66, -2.81, -1.73, -2.81, 1.02, 2.81, -1.31], 2.0),
            ([0.0, 0.0, 0.0, 0.11, 4.63, -0.17, -4.73, -2.51, -4.92, 0.04, 4.98, -2.52], 1.5),
            ([0.0, 0.0, 0.0, 0.08, 4.65, 1.07, -4.72, -2.51, -4.92, 1, 4.98, -2.52], 0.5),
            ([0.0, 0.0, 0.0, 0.11, 4.63, -0.17, -4.73, -2.51, -4.92, 0.04, 4.98, -2.52], 0.5),
            ([0.0, 0.0, 0.0, 0.08, 4.65, 1.07, -4.72, -2.51, -4.92, 1, 4.98, -2.52], 0.5),
            ([0.0, 0.0, 0.0, 0.11, 4.63, -0.17, -4.73, -2.51, -4.92, 0.04, 4.98, -2.52], 0.5),
            ([0.0, 0.0, 0.0, 0.08, 4.65, 1.07, -4.72, -2.51, -4.92, 1, 4.98, -2.52], 0.5)
        ]
        
        # 2. Procesamos la rutina con la fórmula matemática SOLO UNA VEZ
        self.processed_routine = self.from_gui(raw_routine)
        
        # Extraemos la posición inicial (home) ya convertida (el primer array de posiciones)
        self.home_position = self.processed_routine[0][0]
        
        # Lógica de hilos
        self.is_running = False
        self.routine_thread = None
        
        # UI State (Resolución 800x480)
        self.W, self.H = 800, 480
        self.status_msg = "RUTINA PERSONALIZADA LISTA..."
        
        # Botones: [x, y, w, h]
        self.btn_stop_rect = [self.W//2 - 120, self.H//2 - 20, 240, 50]
        self.btn_exit_rect = [self.W//2 - 120, self.H//2 + 60, 240, 50]
        
        self.hover_stop = False
        self.hover_exit = False
        self.exit_requested = False

        self.get_logger().info("Nodo Yaren Listo. UI OpenCV Iniciada.")

    def from_gui(self, poses_gui, default_t=1.0):
        """Convierte poses capturadas al sistema de radianes."""
        K = math.pi / 5
        OFFSET_CODO = math.pi - 1.75  # ≈ 1.3916

        result = []
        for item in poses_gui:
            if isinstance(item, tuple) and len(item) == 2:
                pos, current_t = item
            else:
                pos = item
                current_t = default_t
                
            converted = []
            for idx, val in enumerate(pos):
                if idx == 7 or idx == 11:  # joint_8 y joint_12
                    converted.append(round(val * K + OFFSET_CODO, 4))
                else:
                    converted.append(round(val * K, 4))
            result.append((converted, current_t))
        return result

    def send_movement(self, positions, seconds):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        
        msg.points.append(point)
        self.publisher.publish(msg)

    def wait_match(self, timeout=40.0, check_running=True):
        """Espera a que el controlador este enlazado. False solo si te detienen."""
        t0 = time.time()
        waited = False
        while rclpy.ok() and self.publisher.get_subscription_count() == 0:
            if check_running and not self.is_running:
                return False
            if time.time() - t0 > timeout:
                self.get_logger().warn("Controlador sin enlazar, envio igual")
                return True
            waited = True
            time.sleep(0.1)
        if waited:
            time.sleep(0.5)   # margen tras el match
        return True

    def send_routine(self, routine):
        """Envia TODA la rutina como una sola trayectoria suave."""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        msg.points, total = smooth_points(routine)
        if not self.wait_match():
            return 0.0
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Trayectoria enviada ({len(msg.points)} puntos, {total:.1f} s), "
            f"suscriptores: {self.publisher.get_subscription_count()}")
        return total

    def wait_interruptible(self, secs):
        end = time.time() + secs
        while self.is_running and rclpy.ok() and time.time() < end:
            time.sleep(0.05)

    def go_home(self, secs=2.5):
        """Vuelve a la posicion base (reemplaza cualquier trayectoria en curso)."""
        try:
            self.wait_match(timeout=5.0, check_running=False)
            self.send_movement(self.home_position, secs)
            time.sleep(secs + 0.3)
        except Exception as e:
            self.get_logger().error(f"No se pudo volver a base: {e}")

    def routine_loop(self):
        while self.is_running and rclpy.ok():
            total = self.send_routine(self.processed_routine)
            self.wait_interruptible(total)

    def start_routine(self):
        if not self.is_running:
            self.is_running = True
            self.status_msg = "REPRODUCIENDO RUTINA EN CICLOS..."
            self.routine_thread = threading.Thread(target=self.routine_loop)
            self.routine_thread.start()

    def stop_and_home(self):
        self.is_running = False
        self.status_msg = "DETENIENDO... VOLVIENDO A ORIGEN."
        t = self.routine_thread
        if t is not None and t.is_alive() and t is not threading.current_thread():
            t.join()
        self.go_home()
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
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = YarenMotionControl()
    
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    node.start_routine()

    window_name = "Yaren Routine Custom"
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
"""
1. ([0.0, 0.0, 0, 0.02, 0.04, 0.1, 0.1, -1.1, -0.18, 0.1, -0.17, -1.1], 2.0)
2. ([0.0, 0.0, 0, 0.02, 0.04, 0.1, 0.1, -1.1, -0.18, 0.1, -0.17, -1.1], 1.0)
3. ([0.0, 0.0, 0.0, 0.08, 4.64, -0.18, -4.73, -2.51, -0.18, 0.07, -0.17, -1.11], 1.0)
4. ([0.0, 0.0, 0.0, 0.08, 4.64, -0.18, -4.73, -2.51, -0.18, 0.07, -0.17, -1.11], 0.5)
5. ([0.0, 0.0, 0.0, 0.08, 4.65, 1.07, -4.72, -2.51, -0.18, 0.07, -0.17, -1.11], 0.5)
6. ([0.0, 0.0, 0.0, 0.08, 4.65, 1.07, -4.72, -2.51, -0.18, 0.07, -0.17, -1.11], 0.5)
7. ([0.0, 0.0, -0.0, 0.08, 4.64, -0.16, -4.74, -2.51, -0.18, 0.07, -0.17, -1.11], 0.5)
8. ([0.0, 0.0, -0.0, 0.08, 4.64, -0.16, -4.74, -2.51, -0.18, 0.07, -0.17, -1.11], 1.0)
9.([0.0, 0.0, 0.0, 0.08, 4.65, 1.07, -4.72, -2.51, -0.18, 0.07, -0.17, -1.11], 0.5)
10.([0.0, 0.0, 0.0, 0.08, 4.65, 1.07, -4.72, -2.51, -0.18, 0.07, -0.17, -1.11], 0.5)
11.([0.0, 0.0, -0.0, 0.08, 4.64, -0.16, -4.74, -2.51, -0.18, 0.07, -0.17, -1.11], 0.5)
12.([0.0, 0.0, -0.0, 0.08, 4.64, -0.16, -4.74, -2.51, -0.18, 0.07, -0.17, -1.11], 0.5)
13.([0.0, 0.0, -0.0, 0.08, 2.35, -0.18, -2.44, -2.51, -0.18, 0.07, -0.17, -1.11], 1.0)
14.([0.0, 0.0, -0.0, 0.08, 2.35, -0.18, -2.44, -2.51, -0.18, 0.07, -0.17, -1.11], 1.0)
15.([0.0, 0.0, -0.0, 0.08, 2.35, -0.18, -2.44, -2.51, -0.18, 0.07, -0.17, -1.11], 1.0)
16. ([0.0, 0.0, 0.0, 0.08, 2.50, -0.18, -2.44, -2.51, -0.18, 0.07, -0.17, -1.11], 1.0)
17. ([0.0, 0.0, 0.0, 0.08, 2.50, -0.18, -2.44, -2.51, -0.18, 0.07, -0.17, -1.11], 1.0)
18. ([0.0, 0.0, -0.0, 0.08, 2.35, -0.17, -2.44, -2.51, -0.18, 0.07, -0.17, -1.11], 1.0)
19.([0.0, 0.0, 0.0, 0.1, 2.66, -0.17, -1.52, 0.2, -2.8, -0.2, 1.65, 0.05], 4.0)
20.([0.01, 0.0, 0.0, 0.2, 1.96, 0.28, -1.72, 0.22, -2.13, -0.24, 1.75, -0.31],2)
21.([0.01, 0.0, 0.0, 0.2, 1.96, 0.28, -1.72, 0.22, -2.13, -0.24, 1.75, -0.31],2)
22.([0.01, 0.0, 0.0, 0.2, 1.96, 0.28, -1.72, -1.5, -2.11, -0.24, 1.76, -1.86],2)
23.([0.01, 0.0, 0.0, 0.2, 1.96, 0.28, -1.72, 0.22, -2.13, -0.24, 1.75, -0.31],2)
24.([0.01, 0.0, 0.0, 0.2, 1.96, 0.28, -1.72, 0.22, -2.13, -0.24, 1.75, -0.31],2)
25.([0.01, 0.0, 0.0, 0.2, 1.96, 0.28, -1.72, -1.5, -2.11, -0.24, 1.76, -1.86],2)
35. ([0.0, 0.0, 0.0, 0.17, 2.77, 0.66, -2.81, -1.73, -2.81, 1.02, 2.81, -1.31], 2.0)
36. ([0.0, 0.0, 0.0, 0.17, 2.77, 0.66, -2.81, -1.73, -2.81, 1.02, 2.81, -1.31], 2.0)
4.([0.0, 0.0, 0.0, 0.11, 4.63, -0.17, -4.73, -2.51, -4.92, 0.04, 4.98, -2.52],0.5)
5. ([0.0, 0.0, 0.0, 0.08, 4.65, 1.07, -4.72, -2.51, -4.92, 1, 4.98, -2.52], 0.5)
4.([0.0, 0.0, 0.0, 0.11, 4.63, -0.17, -4.73, -2.51, -4.92, 0.04, 4.98, -2.52],0.5)
5. ([0.0, 0.0, 0.0, 0.08, 4.65, 1.07, -4.72, -2.51, -4.92, 1, 4.98, -2.52], 0.5)
4.([0.0, 0.0, 0.0, 0.11, 4.63, -0.17, -4.73, -2.51, -4.92, 0.04, 4.98, -2.52],0.5)
5. ([0.0, 0.0, 0.0, 0.08, 4.65, 1.07, -4.72, -2.51, -4.92, 1, 4.98, -2.52], 0.5)



"""