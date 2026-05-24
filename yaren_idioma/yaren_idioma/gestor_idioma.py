#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class LanguageManager(Node):
    def __init__(self):
        super().__init__('language_manager')

        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # Publicador hacia STT, TTS y LLM
        self.lang_pub = self.create_publisher(
            String, '/yaren/current_language', qos_profile
        )

        # Servicio manual (por si se quiere cambiar desde terminal)
        self.srv = self.create_service(
            SetBool, '/yaren/set_language_english', self.change_lang_callback
        )

        # 🔹 NUEVO: suscriptor al topic Bool que publica el C++ (yaren_face_display)
        self.create_subscription(
            Bool,
            '/yaren/is_english',
            self.cb_is_english,
            qos_profile  # Transient local: recibe el último valor aunque llegue tarde
        )

        # Idioma por defecto al encender
        self.current_lang = "es"
        self.publish_language()
        self.get_logger().info(
            "Gestor de idioma inicializado. Yaren operando en español."
        )

    def cb_is_english(self, msg: Bool):
        """Recibe el Bool del C++ y lo convierte a String para los nodos de chat."""
        nuevo = "en" if msg.data else "es"
        if nuevo == self.current_lang:
            return
        self.current_lang = nuevo
        self.publish_language()
        self.get_logger().info(
            f"🌐 Idioma actualizado desde pantalla: {self.current_lang}"
        )

    def change_lang_callback(self, request, response):
        """Servicio manual: True = inglés, False = español."""
        viejo = self.current_lang
        self.current_lang = "en" if request.data else "es"

        if self.current_lang == "en":
            mensaje = "Yaren is now operating in English"
        else:
            mensaje = "Ahora Yaren esta operando en Español"

        if viejo != self.current_lang:
            self.publish_language()
            self.get_logger().info(mensaje)

        response.success = True
        response.message = mensaje
        return response

    def publish_language(self):
        msg = String()
        msg.data = self.current_lang
        self.lang_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LanguageManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()