#!/usr/bin/env python3
"""
Bridge WebSocket <-> ROS2 para Yaren Remote.

Protocolo JSON (WebSocket, puerto 9090):
  Flutter → Robot:  {"type": "command", "data": "open_radio"}
  Robot → Flutter:  {"type": "ui_state", "data": "radio_selector"}
                    {"type": "song_list", "data": [...]}   (al conectar)

Tópicos ROS2:
  Publica en:       /yaren/command   (std_msgs/String)
  Suscribe a:       /yaren/ui_state  (std_msgs/String)

Compatible con websockets 15.x
"""

import asyncio
import json
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import websockets

# ──────────────────────────────────────────────────────────────────────────────
#  Lista de canciones (debe coincidir con el orden en face_screen.cpp)
# ──────────────────────────────────────────────────────────────────────────────
SONGS = [
    {"id": 0, "title": "ARA RAM SAM SAM",        "artist": "Luli Pampín"},
    {"id": 1, "title": "BAILE DE GORILA",         "artist": "CantaJuego"},
    {"id": 2, "title": "BARNEY ES UN DINOSAURIO", "artist": "Barney y sus amigos"},
    {"id": 3, "title": "CHOPI CHOPI",             "artist": "Christell"},
    {"id": 4, "title": "LIBRE SOY",               "artist": "Martina Stoessel"},
    {"id": 5, "title": "SA SA",                   "artist": "Luli Pampín"},
    {"id": 6, "title": "SI TIENES GANAS",         "artist": "Luli Pampín"},
]

# ──────────────────────────────────────────────────────────────────────────────
#  Globales
# ──────────────────────────────────────────────────────────────────────────────
_LOOP: asyncio.AbstractEventLoop = None
_NODE: "BridgeNode" = None


# ──────────────────────────────────────────────────────────────────────────────
#  Nodo ROS2
# ──────────────────────────────────────────────────────────────────────────────
class BridgeNode(Node):
    def __init__(self):
        super().__init__("yaren_ws_bridge")

        self.command_pub = self.create_publisher(String, "/yaren/command", 10)

        self.ui_state_sub = self.create_subscription(
            String, "/yaren/ui_state", self._on_ui_state, 10
        )

        self._clients: set = set()
        self._clients_lock = threading.Lock()

        self.get_logger().info("BridgeNode listo. Esperando clientes WebSocket...")

    def _on_ui_state(self, msg: String):
        payload = json.dumps({"type": "ui_state", "data": msg.data})
        self._broadcast(payload)
        self.get_logger().info(f"ui_state → clientes: {msg.data}")

    def send_command(self, cmd: str):
        ros_msg = String()
        ros_msg.data = cmd
        self.command_pub.publish(ros_msg)
        self.get_logger().info(f"command publicado: {cmd}")

    def _broadcast(self, text: str):
        with self._clients_lock:
            clients_snapshot = set(self._clients)
        if not clients_snapshot:
            return
        for ws in clients_snapshot:
            try:
                asyncio.run_coroutine_threadsafe(ws.send(text), _LOOP)
            except Exception as e:
                self.get_logger().warn(f"Error enviando a cliente: {e}")

    def register_client(self, ws):
        with self._clients_lock:
            self._clients.add(ws)
        self.get_logger().info(f"Clientes conectados: {len(self._clients)}")

    def unregister_client(self, ws):
        with self._clients_lock:
            self._clients.discard(ws)
        self.get_logger().info(f"Clientes conectados: {len(self._clients)}")


# ──────────────────────────────────────────────────────────────────────────────
#  Manejador de clientes WebSocket
# ──────────────────────────────────────────────────────────────────────────────
async def _handle_client(websocket):
    _NODE.register_client(websocket)
    remote = websocket.remote_address
    _NODE.get_logger().info(f"Cliente conectado: {remote}")

    try:
        # Al conectar enviar lista de canciones
        await websocket.send(json.dumps({"type": "song_list", "data": SONGS}))

        async for raw in websocket:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                _NODE.get_logger().warn(f"Mensaje no-JSON ignorado: {raw!r}")
                continue

            msg_type = msg.get("type", "")
            data     = msg.get("data", "")

            if msg_type == "command":
                _NODE.send_command(data)

            elif msg_type == "ping":
                await websocket.send(json.dumps({"type": "pong"}))

            else:
                _NODE.get_logger().warn(f"Tipo desconocido: {msg_type!r}")

    except websockets.exceptions.ConnectionClosed:
        pass
    except Exception as e:
        _NODE.get_logger().warn(f"Error en cliente {remote}: {e}")
    finally:
        _NODE.unregister_client(websocket)
        _NODE.get_logger().info(f"Cliente desconectado: {remote}")


# ──────────────────────────────────────────────────────────────────────────────
#  Servidor WebSocket
# ──────────────────────────────────────────────────────────────────────────────
async def _ws_server():
    host = "0.0.0.0"
    port = 9090
    _NODE.get_logger().info(f"Servidor WebSocket en ws://{host}:{port}")
    async with websockets.serve(_handle_client, host, port):
        await asyncio.get_running_loop().create_future()  # correr indefinidamente


# ──────────────────────────────────────────────────────────────────────────────
#  Hilo ROS2
# ──────────────────────────────────────────────────────────────────────────────
def _ros_spin():
    rclpy.spin(_NODE)


# ──────────────────────────────────────────────────────────────────────────────
#  main
# ──────────────────────────────────────────────────────────────────────────────
def main():
    global _LOOP, _NODE

    rclpy.init()
    _NODE = BridgeNode()

    # Spin ROS2 en hilo separado
    ros_thread = threading.Thread(target=_ros_spin, daemon=True)
    ros_thread.start()

    # Correr asyncio en el hilo principal
    _LOOP = asyncio.new_event_loop()
    asyncio.set_event_loop(_LOOP)
    try:
        _LOOP.run_until_complete(_ws_server())
    except KeyboardInterrupt:
        pass
    finally:
        _NODE.get_logger().info("Bridge detenido.")
        rclpy.shutdown()


if __name__ == "__main__":
    main()