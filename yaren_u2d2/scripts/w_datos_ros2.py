#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dynamixel_sdk import *
from motor_classes import XCseries_motor, MX_motor
import threading
import tkinter as tk
from tkinter import scrolledtext, filedialog
import ast
import time
import math

class MotorTorqueControlNode(Node):
    def __init__(self):
        super().__init__('yaren_motor_data')
        
        self.declare_parameter('usb_port', '/dev/ttyUSB0')
        self.declare_parameter('dxl_baud_rate', 1000000)
        
        usb_port = self.get_parameter('usb_port').get_parameter_value().string_value
        dxl_baud_rate = self.get_parameter('dxl_baud_rate').get_parameter_value().integer_value

        self.portHandler = PortHandler(usb_port)
        self.packetHandler = PacketHandler(2.0)
        self.num_joints = 12
        self.general_joint_position = [0.0] * self.num_joints
        self.execution_running = False

        if self.portHandler.openPort():
            self.get_logger().info(f"Puerto {usb_port} abierto con éxito.")
        else:
            self.get_logger().error("Error al abrir el puerto.")

        self.init_motors(usb_port, dxl_baud_rate)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

    def init_motors(self, usb_port, dxl_baud_rate):
        # ORDEN CORREGIDO SEGÚN IDS (1 al 12) PARA SINCRONIZAR CON URDF
        # Joint 1 y 2: MX Series
        self.trunk = MX_motor(usb_port, dxl_baud_rate, [1], self.portHandler, self.packetHandler, None, 20, {1: ['-5', 5]}, {1: [100, 0, 0]})
        self.hip = MX_motor(usb_port, dxl_baud_rate, [2], self.portHandler, self.packetHandler, None, 20, {2: ['-5', 5]}, {2: [100, 0, 0]})
        
        # Joint 3 al 12: XC Series
        self.neck = XCseries_motor(usb_port, dxl_baud_rate, [3, 4], self.portHandler, self.packetHandler, None, 20, {3: ['-5', 5], 4: ['-5', 5]}, {3: [100, 0, 0], 4: [100, 0, 0]})
        self.sh_r = XCseries_motor(usb_port, dxl_baud_rate, [5], self.portHandler, self.packetHandler, None, 20, {5: ['-5', 5]}, {5: [100, 0, 0]})
        self.fa_r = XCseries_motor(usb_port, dxl_baud_rate, [6, 7], self.portHandler, self.packetHandler, None, 20, {6: ['-5', 5], 7: ['-5', 5]}, {6: [100, 0, 0], 7: [100, 0, 0]})
        self.el_r = XCseries_motor(usb_port, dxl_baud_rate, [8], self.portHandler, self.packetHandler, None, 20, {8: ['-5', 5]}, {8: [100, 0, 0]})
        self.sh_l = XCseries_motor(usb_port, dxl_baud_rate, [9], self.portHandler, self.packetHandler, None, 20, {9: ['-5', 5]}, {9: [100, 0, 0]})
        self.fa_l = XCseries_motor(usb_port, dxl_baud_rate, [10, 11], self.portHandler, self.packetHandler, None, 20, {10: ['-5', 5], 11: ['-5', 5]}, {10: [100, 0, 0], 11: [100, 0, 0]})
        self.el_l = XCseries_motor(usb_port, dxl_baud_rate, [12], self.portHandler, self.packetHandler, None, 20, {12: ['-5', 5]}, {12: [100, 0, 0]})

        # Esta lista controla el orden de los botones en la GUI
        self.list_motors = [self.trunk, self.hip, self.neck, self.sh_r, self.fa_r, self.el_r, self.sh_l, self.fa_l, self.el_l]
        
        names = ["Trunk (J1)", "Hip (J2)", "Neck (J3-4)", "Sh_R (J5)", "Forearm_R (J6-7)", "Elbow_R (J8)", "Sh_L (J9)", "Forearm_L (J10-11)", "Elbow_L (J12)"]
        for motor, name in zip(self.list_motors, names):
            motor.name = name
            motor.torque_state = False

    def get_positions(self):
        """Lee posiciones y las ordena estrictamente por ID del 1 al 12."""
        for motor in self.list_motors:
            for id in motor.list_ids:
                dxl_pos, res, err = self.packetHandler.read4ByteTxRx(self.portHandler, id, 132)
                if res == COMM_SUCCESS:
                    # El ID 1 va al índice 0, ID 2 al índice 1...
                    self.general_joint_position[id-1] = round((dxl_pos - 2048) * 0.088 * 5 / 180, 2)
        return self.general_joint_position

    def set_positions(self, positions_list):
        """Escribe posiciones basándose en el índice de la lista = ID-1."""
        try:
            if len(positions_list) < self.num_joints:
                self.get_logger().error(f"Se requieren {self.num_joints} valores.")
                return

            ADDR_GOAL_POSITION = 116 

            for motor in self.list_motors:
                for dxl_id in motor.list_ids:
                    # Sincronización: El valor para el ID X está en positions_list[X-1]
                    rad = positions_list[dxl_id - 1]
                    dxl_val = int((rad * 180) / (0.088 * 5) + 2048)
                    dxl_val = max(0, min(4095, dxl_val))
                    
                    self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_GOAL_POSITION, dxl_val)

            self.get_logger().info("Comando enviado: Base es el primer elemento.")
            
        except Exception as e:
            self.get_logger().error(f"Error en set_positions: {e}")

# --- GUI ---
def run_gui(node):
    root = tk.Tk()
    root.title("Yaren Robot Control - SYNC OK")
    root.geometry("1100x650")

    frame_left = tk.Frame(root)
    frame_left.pack(side=tk.LEFT, padx=20, pady=10, fill=tk.Y)

    motor_buttons = {}
    def toggle_motor(m, btn):
        state = not m.torque_state
        m.torque(1 if state else 0, 64)
        m.torque_state = state
        btn.configure(bg="#4CAF50" if state else "#E0E0E0")

    def toggle_all():
        current_any_on = any(m.torque_state for m in node.list_motors)
        target_state = not current_any_on
        for m in node.list_motors:
            m.torque(1 if target_state else 0, 64)
            m.torque_state = target_state
            motor_buttons[m].configure(bg="#4CAF50" if target_state else "#E0E0E0")

    tk.Label(frame_left, text="Torque Control (By ID Order)", font=("Arial", 10, "bold")).pack(pady=5)
    for m in node.list_motors:
        btn = tk.Button(frame_left, text=m.name, bg="#E0E0E0", width=25,
                        command=lambda mot=m: toggle_motor(mot, motor_buttons[mot]))
        motor_buttons[m] = btn
        btn.pack(pady=1)
    
    tk.Button(frame_left, text="ALL TORQUE ON/OFF", bg="#2196F3", fg="white", width=25, command=toggle_all).pack(pady=10)

    pos_display = tk.Entry(frame_left, width=55)
    pos_display.pack(pady=(20, 5))
    tk.Button(frame_left, text="PRINT CURRENT POS (ID 1-12)", command=lambda: [pos_display.delete(0, tk.END), pos_display.insert(0, str(node.get_positions()))]).pack()

    tk.Label(frame_left, text="SET POSITIONS [J1, J2, ... J12]").pack(pady=(25, 0))
    set_entry = tk.Entry(frame_left, width=55)
    set_entry.pack(pady=5)
    tk.Button(frame_left, text="SET", command=lambda: node.set_positions(ast.literal_eval(set_entry.get()))).pack()

    frame_right = tk.Frame(root)
    frame_right.pack(side=tk.RIGHT, expand=True, fill=tk.BOTH, padx=20, pady=10)
    tk.Label(frame_right, text="Motion Sequences", font=("Arial", 10, "bold")).pack()
    seq_text = scrolledtext.ScrolledText(frame_right, width=65, height=28)
    seq_text.pack(pady=5)

    def run_seq():
        node.execution_running = True
        lines = seq_text.get("1.0", tk.END).strip().split('\n')
        
        def task():
            for line in lines:
                if not node.execution_running: break
                try:
                    # Limpiar prefijos numéricos como "1. " de la línea
                    clean = line.strip()
                    if '.' in clean:
                        parts = clean.split('.', 1)
                        if parts[0].isdigit():
                            clean = parts[1].strip()
                            
                    if not clean: continue
                    
                    # Evaluamos la línea (puede ser lista o tupla)
                    evaluated_data = ast.literal_eval(clean)
                    
                    # Verificamos si tiene formato ([posiciones], tiempo)
                    if isinstance(evaluated_data, tuple) and len(evaluated_data) == 2:
                        positions = evaluated_data[0]
                        t = evaluated_data[1]
                    else:
                        # Formato clásico: solo [posiciones]
                        positions = evaluated_data
                        t = 1.0 # 1 segundo por defecto
                        
                    node.set_positions(positions)
                    time.sleep(t)
                    
                except Exception as e:
                    node.get_logger().error(f"Error parseando línea: {e}")
                    continue
            node.execution_running = False
            
        threading.Thread(target=task, daemon=True).start()

    btn_frame = tk.Frame(frame_right)
    btn_frame.pack(pady=5)
    tk.Button(btn_frame, text="Execute", command=run_seq).pack(side=tk.LEFT, padx=5)
    tk.Button(btn_frame, text="Stop", command=lambda: setattr(node, 'execution_running', False)).pack(side=tk.LEFT, padx=5)
    tk.Button(btn_frame, text="Load File", command=lambda: [seq_text.delete("1.0", tk.END), seq_text.insert(tk.END, open(filedialog.askopenfilename(), 'r').read())]).pack(side=tk.LEFT, padx=5)

    root.mainloop()

def main():
    rclpy.init()
    node = MotorTorqueControlNode()
    threading.Thread(target=run_gui, args=(node,), daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.portHandler.closePort()
        rclpy.shutdown()

if __name__ == '__main__':
    main()