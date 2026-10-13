#!/usr/bin/env python3
"""Chatbot OFFLINE (base de conocimiento). Si no sabe y hay internet, delega al yaren_chat normal."""
import os, socket, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from kb import KB

NO_ENCONTRADO = ("No tengo una respuesta para eso por ahora. "
                 "Cuando tenga internet, usaré el modelo de inteligencia artificial en línea para ayudarte.")


class ChatNode(Node):
    def __init__(self):
        super().__init__("chat_node")
        share = get_package_share_directory("yaren_chatbot")
        self.declare_parameter("db_path", os.path.join(share, "resource", "kb.db"))
        self.declare_parameter("threshold", 0.45)
        self.declare_parameter("use_online", True)              # delegar a yaren_chat si hay internet
        self.declare_parameter("in_topic", "stt_text")          # ajusta a tu tópico de STT
        self.declare_parameter("out_topic", "tts_text")         # ajusta a tu tópico de TTS
        self.declare_parameter("online_topic", "chat_online_in")  # entrada de tu yaren_chat normal

        self.kb = KB(self.get_parameter("db_path").value, self.get_parameter("threshold").value)
        self.pub = self.create_publisher(String, self.get_parameter("out_topic").value, 10)
        self.pub_online = self.create_publisher(String, self.get_parameter("online_topic").value, 10)
        self.create_subscription(String, self.get_parameter("in_topic").value, self.on_text, 10)
        self._net_ts, self._net_ok = 0.0, False
        self.get_logger().info("chat_node listo")

    def hay_internet(self, ttl=10.0):
        if time.time() - self._net_ts > ttl:
            try:
                socket.create_connection(("8.8.8.8", 53), timeout=1.5).close()
                self._net_ok = True
            except OSError:
                self._net_ok = False
            self._net_ts = time.time()
        return self._net_ok

    def on_text(self, msg):
        pregunta = msg.data.strip()
        if not pregunta:
            return
        resp, score = self.kb.search(pregunta)
        self.get_logger().info(f"'{pregunta}' -> score={score:.2f}")
        if resp is not None:
            self.pub.publish(String(data=resp))
        elif self.get_parameter("use_online").value and self.hay_internet():
            self.get_logger().info("sin respuesta offline -> delegando a yaren_chat")
            self.pub_online.publish(String(data=pregunta))   # yaren_chat responde por su cuenta
        else:
            self.kb.log_pending(pregunta)
            self.pub.publish(String(data=NO_ENCONTRADO))


def main():
    rclpy.init()
    rclpy.spin(ChatNode())


if __name__ == "__main__":
    main()
