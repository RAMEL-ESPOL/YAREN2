#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class YarenDanceRadio(Node):
    def __init__(self):
        super().__init__('yaren_dance_radio')
        
        self.publisher = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10)
        
        self.joint_names = [
            "joint_1", "joint_2", "joint_3", "joint_4", 
            "joint_5", "joint_6", "joint_7", "joint_8", 
            "joint_9", "joint_10", "joint_11", "joint_12"
        ]
        self.get_logger().info("¡Yaren comenzó a bailar la música de la radio!")

    def send_movement(self, positions, seconds):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        
        # Convierte segundos a formato ROS
        point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        msg.points.append(point)
        self.publisher.publish(msg)

    def run_dance(self):
        # Rutina extraída directamente de la terminal (ya en radianes)
        rutina = [
            # 1. Posición Inicial (todo a 0) - (2 segundos)
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], 2.0),

            # 2. Brazos extendidos hacia adelante asimétricos - (1 segundo)
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, -3.0, 0.0, -1.5, 0.0, 3.0, 0.0], 2.0),

            # 3. Brazo derecho arriba (3.0), izquierdo extendido (-3.0) - (1 segundo)
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.5], 2.0),

            # 4. Brazo derecho abajo (0.0), izquierdo extendido (-3.0) - (1 segundo)
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, -1.5, 0.0, 3.0, 0.0], 2.0),

            # 7. Regreso a Posición Inicial - (2 segundos)
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], 2.0),

            # 8. Movimiento simétrico, ambos brazos arriba - (1 segundo)
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.0, -3.0, 0.0, -3.0, 0.0, 3.0, 0.0], 2.0),

            # 9. Manos levantadas con codos flexionados - (1 segundo)
            ([0.0, 0.0, 0.0, 0.0, 3.0, 1.0, -3.0, 0.0, -3.0, 1.0, 3.0, 0.0], 2.0),

            # 10. Mismo movimiento (mantener) - (1 segundo)
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.0, -3.0, 0.0, -3.0, 0.0, 3.0, 0.0], 2.0),

            # 11. Flexión de codos de nuevo - (1 segundo)
            ([0.0, 0.0, 0.0, 0.0, 3.0, 1.0, -3.0, 0.0, -3.0, 1.0, 3.0, 0.0], 2.0),

            # 12. Estirar codos - (1 segundo)
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.0, -3.0, 0.0, -3.0, 0.0, 3.0, 0.0], 2.0),

            # 13. Brazo derecho medio arriba, izquierdo abajo - (1 segundo)
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.5], 2.0),


            # 15. Regreso a Posición Inicial con leve ajuste en codos - (1 segundo)
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], 2.0),

            # 16. Ambos brazos hacia adelante - (1 segundo)
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, -1.5, 0.0, 0.0, 0.0], 2.0),

            # 17. Brazos hacia adelante con flexión leve de codos - (1 segundo)
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 1.0, -1.5, 0.0, 0.0, 0.0], 2.0),

            # 18. Flexión asimétrica de codos - (1 segundo)
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 1.0, -1.5, 0.0, 0.0, 1.0], 2.0),

            # 19. Desflexión asimétrica - (1 segundo)
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, -1.5, 0.0, 0.0, 1.0], 2.0),

            # 20. Regresar flexión - (1 segundo)
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, -1.5, 0.0, 0.0, 0.0], 2.0),

            # 21. Flexión asimétrica (repetida) - (1 segundo)
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 1.0, -1.5, 0.0, 0.0, 0.0], 2.0),

            # 22. Codos flexionados simultáneos - (1 segundo)
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 1.0, -1.5, 0.0, 1.5, 1.0], 2.0),

            # 23. Cambio de ángulo de brazo derecho - (1 segundo)
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, -1.5, 1.0, -1.5, 0.0, 0.0, 1.0], 2.0),

            # 24. Brazos cruzados arriba - (1 segundo)
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, -1.0, 1.0, -3.0, 0.5, 3.0, 0.0], 2.0),

            # 25. Brazos cruzados opuestos - (1 segundo)
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.5, -3.0, 0.0, -1.5, 0.0, 1.0, 1.0], 2.0),
            
            # 26. Alternancia rápida - (1 segundo)
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, -1.0, 1.0, -3.0, 0.5, 3.0, 0.0], 2.0),
            
            # 27. Alternancia final - (1 segundo)
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.5, -3.0, 0.0, -1.5, 0.0, 1.0, 1.0], 2.0)
        ]

        # Bucle infinito: mientras el nodo siga vivo (la música siga sonando), repetirá el baile
        while rclpy.ok():
            for pos, t in rutina:
                if not rclpy.ok():
                    break
                self.send_movement(pos, t)
                time.sleep(t)

def main(args=None):
    rclpy.init(args=args)
    node = YarenDanceRadio()
    try:
        node.run_dance()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()