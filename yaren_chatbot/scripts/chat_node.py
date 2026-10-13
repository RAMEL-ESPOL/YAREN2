#!/usr/bin/env python3
"""YAREN chatbot OFFLINE: conversación guiada + base de conocimiento.
Si no sabe y hay internet, delega al yaren_chat normal."""
import json, os, socket, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from ament_index_python.packages import get_package_share_directory
from kb import KB
from dialogue import Dialogue

NO_ENCONTRADO = ("No tengo una respuesta para eso por ahora. "
                 "Cuando tenga internet, usaré el modelo de inteligencia artificial en línea para ayudarte.")


class ChatNode(Node):
    def __init__(self):
        super().__init__("chat_node")
        share = get_package_share_directory("yaren_chatbot")
        self.declare_parameter("db_path", os.path.join(share, "resource", "kb.db"))
        self.declare_parameter("flow_path", os.path.join(share, "data", "conversacion.json"))
        self.declare_parameter("threshold", 0.45)
        self.declare_parameter("modo_conversacion", True)       # False = solo preguntas y respuestas
        self.declare_parameter("session_timeout", 180.0)        # segundos sin hablar → nueva persona
        self.declare_parameter("use_online", True)              # delegar a yaren_chat si hay internet
        self.declare_parameter("in_topic", "stt_text")          # ajusta a tu tópico de STT
        self.declare_parameter("out_topic", "tts_text")         # ajusta a tu tópico de TTS
        self.declare_parameter("online_topic", "chat_online_in")  # entrada de tu yaren_chat normal
        self.declare_parameter("start_topic", "chat_start")     # Empty → YAREN inicia la charla
        self.declare_parameter("hint_topic", "stt_hint")        # palabras esperadas (JSON) para limitar el STT

        self.kb = KB(self.get_parameter("db_path").value, self.get_parameter("threshold").value)
        self.dlg = Dialogue(self.get_parameter("flow_path").value, self.kb,
                            self.get_parameter("session_timeout").value)
        self.pub = self.create_publisher(String, self.get_parameter("out_topic").value, 10)
        self.pub_hint = self.create_publisher(String, self.get_parameter("hint_topic").value, 10)
        self.pub_online = self.create_publisher(String, self.get_parameter("online_topic").value, 10)
        self.create_subscription(String, self.get_parameter("in_topic").value, self.on_text, 10)
        self.create_subscription(Empty, self.get_parameter("start_topic").value, self.on_start, 10)
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

    def say(self, text):
        self.pub.publish(String(data=text))
        # pista para el STT: lista de palabras esperadas ([] = respuesta libre)
        self.pub_hint.publish(String(data=json.dumps(self.dlg.expected_words(), ensure_ascii=False)))

    def on_start(self, _):
        self.say(self.dlg.start())

    def on_text(self, msg):
        pregunta = msg.data.strip()
        if not pregunta:
            return
        if self.get_parameter("modo_conversacion").value:
            resp, kind = self.dlg.handle(pregunta)
        else:
            resp, _ = self.kb.search(pregunta)
            kind = "kb" if resp else None
        self.get_logger().info(f"'{pregunta}' -> {kind} (nodo={self.dlg.node})")
        if kind == "ignore":          # ruido / muletillas: no responder
            return
        if resp is not None:
            self.say(resp)
        elif self.get_parameter("use_online").value and self.hay_internet():
            self.get_logger().info("sin respuesta offline -> delegando a yaren_chat")
            self.pub_online.publish(String(data=pregunta))   # yaren_chat responde por su cuenta
        else:
            self.kb.log_pending(pregunta)
            self.say((NO_ENCONTRADO + " " + self.dlg.reprompt()).strip())


def main():
    rclpy.init()
    rclpy.spin(ChatNode())


if __name__ == "__main__":
    main()
