#!/usr/bin/env python3
"""
ws_bridge.py — puente entre la app remota (WebSocket) y face_screen (ROS2).

App  --{"type":"command","data":"..."}-->  bridge  --> <command_topic>  (String)
App  <--{"type":"ui_state","data":"..."}--  bridge  <-- <ui_state_topic> (String)

Parametros opcionales:
  python3 ws_bridge.py --ros-args \
      -p command_topic:=/yaren/command \
      -p ui_state_topic:=/yaren/ui_state \
      -p allow_poweroff:=false
"""
import asyncio
import json
import re
import threading

import rclpy
import websockets
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

HOST = "0.0.0.0"
PORT = 9090

# Comandos que el robot entiende (ver onRemoteCommand en face_screen.cpp)
EXACT_COMMANDS = {"open_menu", "back", "exit", "stop", "open_settings", "go_home"}
PREFIX_COMMANDS = ("open_submenu_", "select_")
ID_RE = re.compile(r"^[a-z0-9_]+$")


class WsBridge(Node):
    def __init__(self):
        super().__init__("ws_bridge")
        self.declare_parameter("command_topic", "/yaren/command")
        self.declare_parameter("ui_state_topic", "/yaren/ui_state")
        # Apagar el robot desde el celular: desactivado por defecto
        self.declare_parameter("allow_poweroff", False)
        cmd_topic = self.get_parameter("command_topic").value
        state_topic = self.get_parameter("ui_state_topic").value
        self.allow_poweroff = bool(self.get_parameter("allow_poweroff").value)

        # transient_local: recibe el ultimo estado aunque el bridge arranque despues del robot
        state_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.command_pub = self.create_publisher(String, cmd_topic, 10)
        self.state_sub = self.create_subscription(
            String, state_topic, self._on_ui_state, state_qos
        )

        self.ws_clients = set()
        self.last_state = None
        self.loop = None  # se asigna cuando arranca asyncio

        self.get_logger().info(
            f"Comandos -> {cmd_topic} | Estado <- {state_topic} | poweroff={self.allow_poweroff}"
        )

    def is_valid_command(self, cmd: str) -> bool:
        if cmd in EXACT_COMMANDS:
            return True
        if cmd == "power_off":
            return self.allow_poweroff
        for prefix in PREFIX_COMMANDS:
            if cmd.startswith(prefix):
                return bool(ID_RE.match(cmd[len(prefix):]))
        return False

    # ---------- ROS -> WebSocket ----------
    @staticmethod
    def _state_payload(state: str) -> str:
        return json.dumps({"type": "ui_state", "data": state})

    def _on_ui_state(self, msg: String):
        self.last_state = msg.data
        self.get_logger().info(f"ui_state: {msg.data}")
        if self.loop is not None:
            self.loop.call_soon_threadsafe(self._broadcast, self._state_payload(msg.data))

    def _broadcast(self, payload: str):
        for ws in list(self.ws_clients):
            asyncio.ensure_future(self._safe_send(ws, payload))

    @staticmethod
    async def _safe_send(ws, payload: str):
        try:
            await ws.send(payload)
        except Exception:
            pass

    # ---------- WebSocket -> ROS ----------
    def send_command(self, cmd: str):
        self.command_pub.publish(String(data=cmd))
        self.get_logger().info(f"comando enviado: {cmd}")

    # `path=None` mantiene compatibilidad con versiones viejas de websockets (<10.1)
    async def handle_client(self, ws, path=None):
        self.ws_clients.add(ws)
        self.get_logger().info(f"cliente conectado ({len(self.ws_clients)} en total)")
        try:
            # Estado actual apenas se conecta
            if self.last_state is not None:
                await ws.send(self._state_payload(self.last_state))

            async for raw in ws:
                try:
                    msg = json.loads(raw)
                    msg_type = msg.get("type")
                except (ValueError, AttributeError):
                    continue

                if msg_type == "command":
                    cmd = str(msg.get("data", "")).strip()
                    if self.is_valid_command(cmd):
                        self.send_command(cmd)
                    else:
                        self.get_logger().warn(f"comando rechazado: {cmd!r}")
                        await ws.send(json.dumps({"type": "error", "data": "comando no permitido"}))
                elif msg_type == "ping":
                    await ws.send(json.dumps({"type": "pong"}))
        except websockets.ConnectionClosed:
            pass
        finally:
            self.ws_clients.discard(ws)
            self.get_logger().info(f"cliente desconectado ({len(self.ws_clients)} en total)")


async def serve(node: WsBridge):
    node.loop = asyncio.get_running_loop()
    async with websockets.serve(node.handle_client, HOST, PORT):
        node.get_logger().info(f"WebSocket escuchando en ws://{HOST}:{PORT}")
        await asyncio.Future()  # correr para siempre


def main():
    rclpy.init()
    node = WsBridge()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        asyncio.run(serve(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()