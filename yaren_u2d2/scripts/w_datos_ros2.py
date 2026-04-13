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

class MotorTorqueControlNode(Node):
    def __init__(self):
        super().__init__('yaren_motor_data')
        
        # Parámetros de ROS 2
        self.declare_parameter('usb_port', '/dev/ttyUSB0')
        self.declare_parameter('dxl_baud_rate', 1000000)
        
        usb_port = self.get_parameter('usb_port').get_parameter_value().string_value
        dxl_baud_rate = self.get_parameter('dxl_baud_rate').get_parameter_value().integer_value

        self.portHandler = PortHandler(usb_port)
        self.packetHandler = PacketHandler(2.0)
        self.num_joints = 12
        self.general_joint_position = [0.0] * self.num_joints
        self.execution_running = False

        # Intentar abrir puerto
        if self.portHandler.openPort():
            self.get_logger().info(f"Puerto {usb_port} abierto con éxito.")
        else:
            self.get_logger().error("Error al abrir el puerto.")

        # Inicialización de motores
        self.init_motors(usb_port, dxl_baud_rate)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

    def init_motors(self, usb_port, dxl_baud_rate):
        self.neck = XCseries_motor(usb_port, dxl_baud_rate, [3, 4], self.portHandler, self.packetHandler, None, 20, {3: [-1, 1], 4: [-1, 1]}, {3: [100, 0, 0], 4: [100, 0, 0]})
        self.sh_r = XCseries_motor(usb_port, dxl_baud_rate, [5], self.portHandler, self.packetHandler, None, 20, {5: [-1, 1]}, {5: [100, 0, 0]})
        self.fa_r = XCseries_motor(usb_port, dxl_baud_rate, [6, 7], self.portHandler, self.packetHandler, None, 20, {6: [-1, 1], 7: [-1, 1]}, {6: [100, 0, 0], 7: [100, 0, 0]})
        self.el_r = XCseries_motor(usb_port, dxl_baud_rate, [8], self.portHandler, self.packetHandler, None, 20, {8: [-1, 1]}, {8: [100, 0, 0]})
        self.sh_l = XCseries_motor(usb_port, dxl_baud_rate, [9], self.portHandler, self.packetHandler, None, 20, {9: [-1, 1]}, {9: [100, 0, 0]})
        self.fa_l = XCseries_motor(usb_port, dxl_baud_rate, [10, 11], self.portHandler, self.packetHandler, None, 20, {10: [-1, 1], 11: [-1, 1]}, {10: [100, 0, 0], 11: [100, 0, 0]})
        self.el_l = XCseries_motor(usb_port, dxl_baud_rate, [12], self.portHandler, self.packetHandler, None, 20, {12: [-1, 1]}, {12: [100, 0, 0]})
        self.trunk = MX_motor(usb_port, dxl_baud_rate, [1], self.portHandler, self.packetHandler, None, 20, {1: [-1, 1]}, {1: [100, 0, 0]})
        self.hip = MX_motor(usb_port, dxl_baud_rate, [2], self.portHandler, self.packetHandler, None, 20, {2: [-1, 1]}, {2: [100, 0, 0]})

        self.list_motors = [self.neck, self.sh_r, self.fa_r, self.el_r, self.sh_l, self.fa_l, self.el_l, self.trunk, self.hip]
        
        names = ["neck_motor", "shoulder_right", "forearm_right", "elbow_right", "shoulder_left", "forearm_left", "elbow_left", "trunk_motor", "hip_motor"]
        for motor, name in zip(self.list_motors, names):
            motor.name = name
            motor.torque_state = False

    def get_positions(self):
        for motor in self.list_motors:
            for id in motor.list_ids:
                dxl_pos, res, err = self.packetHandler.read4ByteTxRx(self.portHandler, id, 132)
                if res == COMM_SUCCESS:
                    self.general_joint_position[id-1] = round((dxl_pos - 2048) * 0.088 * 3.14 / 180, 2)
        return self.general_joint_position

    def set_positions(self, positions_list):
        """Escribe directamente en el registro Goal Position (116) de los motores."""
        try:
            if len(positions_list) < self.num_joints:
                self.get_logger().error(f"Se requieren {self.num_joints} valores.")
                return

            # Dirección del registro Goal Position para XC y MX (Protocolo 2.0)
            ADDR_GOAL_POSITION = 116 

            for motor in self.list_motors:
                for dxl_id in motor.list_ids:
                    rad = positions_list[dxl_id - 1]
                    
                    # Conversión: Radianes -> Valor Dynamixel (0-4095)
                    dxl_val = int((rad * 180) / (0.088 * 3.14) + 2048)
                    dxl_val = max(0, min(4095, dxl_val)) # Límite de seguridad
                    
                    # ESCRIBIMOS DIRECTO AL PUERTO (Sin depender de motor_classes.py)
                    dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
                        self.portHandler, 
                        dxl_id, 
                        ADDR_GOAL_POSITION, 
                        dxl_val
                    )
                    
                    if dxl_comm_result != COMM_SUCCESS:
                        print(f"ID {dxl_id}: Error de comunicación {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                    elif dxl_error != 0:
                        print(f"ID {dxl_id}: Error de hardware {self.packetHandler.getRxPacketError(dxl_error)}")

            self.get_logger().info("Comando de posición enviado correctamente.")
            
        except Exception as e:
            self.get_logger().error(f"Error crítico en set_positions: {e}")

# --- Interfaz Gráfica (Tkinter) ---
def run_gui(node):
    root = tk.Tk()
    root.title("Motor Torque Control - Yaren Robot")
    root.geometry("1100x650")

    # CONTENEDOR IZQUIERDO
    frame_left = tk.Frame(root)
    frame_left.pack(side=tk.LEFT, padx=20, pady=10, fill=tk.Y)

    # 1. Botones de Torque
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

    for m in node.list_motors:
        btn = tk.Button(frame_left, text=m.name, bg="#E0E0E0", width=25,
                        command=lambda mot=m: toggle_motor(mot, motor_buttons[mot]))
        motor_buttons[m] = btn
        btn.pack(pady=1)
    
    tk.Button(frame_left, text="All", bg="#2196F3", fg="white", width=25, command=toggle_all).pack(pady=5)

    # 2. Área de Lectura (Print)
    pos_display = tk.Entry(frame_left, width=55)
    pos_display.pack(pady=(20, 5))

    def read_and_print():
        positions = node.get_positions()
        pos_display.delete(0, tk.END)
        pos_display.insert(0, str(positions))

    tk.Button(frame_left, text="Print", command=read_and_print).pack()

    # 3. Área de Escritura (Set)
    tk.Label(frame_left, text="Set Motor Positions").pack(pady=(25, 0))
    set_entry = tk.Entry(frame_left, width=55)
    set_entry.pack(pady=5)

    def apply_set():
        try:
            raw_val = set_entry.get()
            pos_list = ast.literal_eval(raw_val)
            node.set_positions(pos_list)
        except Exception as e:
            print(f"Error al aplicar posición: {e}")

    tk.Button(frame_left, text="Set", command=apply_set).pack()

    # CONTENEDOR DERECHO (Secuencias)
    frame_right = tk.Frame(root)
    frame_right.pack(side=tk.RIGHT, expand=True, fill=tk.BOTH, padx=20, pady=10)

    tk.Label(frame_right, text="Motion Sequences", font=("Arial", 10, "bold")).pack()
    seq_text = scrolledtext.ScrolledText(frame_right, width=65, height=28)
    seq_text.pack(pady=5)

    def load_seq():
        path = filedialog.askopenfilename(filetypes=[("Text files", "*.txt")])
        if path:
            with open(path, 'r') as f:
                seq_text.delete("1.0", tk.END)
                seq_text.insert(tk.END, f.read())

    def run_seq():
        node.execution_running = True
        lines = seq_text.get("1.0", tk.END).strip().split('\n')
        
        def task():
            for line in lines:
                if not node.execution_running: break
                try:
                    # Limpia el "1. " si existe al inicio de la línea
                    clean = line.split('.', 1)[1].strip() if '.' in line else line.strip()
                    if not clean: continue
                    
                    data = ast.literal_eval(clean)
                    node.set_positions(data)
                    time.sleep(1.0) # Tiempo entre pasos
                except Exception as e:
                    print(f"Error en línea: {e}")
            node.execution_running = False

        threading.Thread(target=task, daemon=True).start()

    def stop_seq():
        node.execution_running = False

    btn_frame = tk.Frame(frame_right)
    btn_frame.pack(pady=5)
    tk.Button(btn_frame, text="Execute Sequences", command=run_seq).pack(side=tk.LEFT, padx=5)
    tk.Button(btn_frame, text="Stop Execution", command=stop_seq).pack(side=tk.LEFT, padx=5)
    tk.Button(btn_frame, text="Load Sequences", command=load_seq).pack(side=tk.LEFT, padx=5)

    root.mainloop()

def main():
    rclpy.init()
    node = MotorTorqueControlNode()
    
    gui_thread = threading.Thread(target=run_gui, args=(node,), daemon=True)
    gui_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.portHandler.closePort()
        rclpy.shutdown()

if __name__ == '__main__':
    main()