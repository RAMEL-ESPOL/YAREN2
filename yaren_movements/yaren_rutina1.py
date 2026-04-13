#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class YarenMotionControl(Node):
    def __init__(self):
        super().__init__('yaren_motion_control')
        
        # Publicamos al controlador de trayectoria para suavizar movimientos
        self.publisher = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10)
        
        self.joint_names = [
            "joint_1", "joint_2", "joint_3", "joint_4", 
            "joint_5", "joint_6", "joint_7", "joint_8", 
            "joint_9", "joint_10", "joint_11", "joint_12"
        ]
        
        self.get_logger().info("Controlador de Movimiento Yaren Listo.")
        self.get_logger().info("Selecciona: 1(Base), 2(Der), 3(Izq), 4(Frente), 5(Secuencia)")

    def send_movement(self, positions, seconds):
        """Envía un punto de trayectoria para que el robot se mueva suavemente."""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        # El tiempo define qué tan lento o rápido llega (esto quita lo 'robótico')
        point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        
        msg.points.append(point)
        self.publisher.publish(msg)

    def run_menu(self):
        while rclpy.ok():
            try:
                number = input("\nIngrese número: ")
                
                if number == '1':
                    print("Moviendo a Base...")
                    self.send_movement([0.0]*12, 2.0)

                elif number == '2':
                    print("Moviendo a Posición 2...")
                    self.send_movement([0.3, 0.69] + [0.0]*10, 1.5)

                elif number == '3':
                    print("Moviendo a Posición 3...")
                    self.send_movement([-0.3, -0.69] + [0.0]*10, 1.5)

                elif number == '4':
                    print("Moviendo a Posición 4...")
                    self.send_movement([0.7] + [0.0]*11, 1.5)

                elif number == '5':
                    print("Iniciando Secuencia Fluida...")
                    secuencia = [
                        ([0.0,0.0,0.0,0.0,0.78,0.0,0.0,0.52,0.0,0.52,0.52,0.0], 1.0),
                        ([0.4,0.0,0.0,0.0,0.78,0.0,0.0,0.52,0.0,0.52,0.52,0.0], 1.0),
                        ([0.4,0.0,0.0,0.0,0.78,0.52,0.52,0.52,-0.78,0.52,-0.52,0.52], 1.0),
                        ([0.0,0.0,0.0,0.0,0.78,0.52,0.52,1.5,-0.78,0.52,-0.52,1.5], 1.0),
                        ([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.48,0.0,0.0,0.0,0.48], 1.5)
                    ]
                    for pos, t in secuencia:
                        self.send_movement(pos, t)
                        import time
                        time.sleep(t) # Esperar a que termine cada paso

                elif number == '6':
                    print("Iniciando Rutina Larga...")
                    rutina_6 = [
                        ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 5.25),
                        ([-0.01, -0.01, -0.0, 0.35, 0.59, -0.19, -0.2, -0.07, -0.75, -0.2, 0.33, -0.01], 2.0),
                        ([-0.01, 0.0, -0.0, 0.35, 0.78, -0.04, -1.08, 0.21, -0.88, -0.1, 1.13, 0.21], 1.0),
                        ([-0.01, 0.21, -0.01, 0.35, 0.37, 0.13, -1.01, 0.1, -0.52, -0.1, 0.81, 0.13], 1.0),
                        ([0.0, -0.21, -0.01, 0.35, 0.52, 0.01, -0.96, 0.06, -0.23, -0.13, 0.83, -0.12], 1.0),
                        ([0.03, 0.01, -0.01, 0.35, 0.94, 0.03, -0.9, 0.08, -0.36, 0.0, 0.86, 0.11], 1.0),
                        ([0.01, 0.02, -0.01, 0.35, 0.58, -0.02, -1.09, 0.03, -0.85, -0.03, 0.78, 0.19], 1.0),
                        ([0.05, -0.02, -0.01, 0.35, 0.6, 0.08, 0.28, -0.03, -0.81, -0.04, -0.32, 0.02], 1.0),
                        ([-0.04, -0.02, -0.01, 0.35, 1.02, -0.25, -1.3, -0.28, -1.07, -0.15, 0.96, -0.24], 1.0),
                        ([-0.04, 0.21, -0.02, 0.35, 0.8, -0.25, -1.3, -0.26, -0.9, -0.16, 0.97, -0.21], 1.0),
                        ([-0.04, -0.21, -0.02, 0.35, 0.75, -0.25, -1.29, -0.26, -0.87, -0.16, 0.97, -0.22], 1.0),
                        ([0.0, 0.15, -0.03, 0.35, 0.58, -0.14, 0.41, -0.03, -0.04, -0.05, 0.76, -0.0], 2.0),
                        ([0.04, -0.15, -0.03, 0.35, 0.46, -0.19, 0.34, 0.03, -0.81, -0.15, -0.37, 0.04], 2.0),
                        ([-0.01, -0.01, -0.0, 0.35, 0.59, -0.19, -0.2, -0.07, -0.75, -0.2, 0.33, -0.01], 0.3),
                        ([-0.01, 0.0, -0.0, 0.35, 0.78, -0.04, -1.08, 0.21, -0.88, -0.1, 1.13, 0.21], 0.5),
                        ([-0.01, 0.21, -0.01, 0.35, 0.37, 0.13, -1.01, 0.1, -0.52, -0.1, 0.81, 0.13], 0.5),
                        ([0.0, -0.21, -0.01, 0.35, 0.52, 0.01, -0.96, 0.06, -0.23, -0.13, 0.83, -0.12], 0.5),
                        ([0.03, 0.01, -0.01, 0.35, 0.94, 0.03, -0.9, 0.08, -0.36, 0.0, 0.86, 0.11], 0.5),
                        ([0.01, 0.02, -0.01, 0.35, 0.58, -0.02, -1.09, 0.03, -0.85, -0.03, 0.78, 0.19], 0.5),
                        ([0.05, -0.02, -0.01, 0.35, 0.6, 0.08, 0.28, -0.03, -0.81, -0.04, -0.32, 0.02], 0.5),
                        ([-0.04, -0.02, -0.01, 0.35, 1.02, -0.25, -1.3, -0.28, -1.07, -0.15, 0.96, -0.24], 0.5),
                        ([-0.04, 0.21, -0.02, 0.35, 0.8, -0.25, -1.3, -0.26, -0.9, -0.16, 0.97, -0.21], 0.5),
                        ([-0.04, -0.21, -0.02, 0.35, 0.75, -0.25, -1.29, -0.26, -0.87, -0.16, 0.97, -0.22], 0.5),
                        ([0.0, 0.15, -0.03, 0.35, 0.58, -0.14, 0.41, -0.03, -0.04, -0.05, 0.76, -0.0], 0.5),
                        ([0.04, -0.15, -0.03, 0.35, 0.46, -0.19, 0.34, 0.03, -0.81, -0.15, -0.37, 0.04], 0.5),
                        ([-0.01, -0.08, -0.02, 0.35, 0.05, -0.13, 0.04, -0.17, -0.34, 0.08, 0.04, -0.13], 1.0)
                    ]
                    
                    for pos, t in rutina_6:
                        self.send_movement(pos, t)
                        import time
                        time.sleep(t) # Esperar a que termine cada paso

            except EOFError:
                break
def main(args=None):
    rclpy.init(args=args)
    node = YarenMotionControl()
    node.run_menu()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()