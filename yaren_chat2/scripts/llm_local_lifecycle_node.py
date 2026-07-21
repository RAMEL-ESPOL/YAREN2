#!/usr/bin/env python3
import os
import sys
import re
import threading
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String, Bool
from yaren_interfaces.action import ProcessResponse

WS = os.path.expanduser("~/robotis_ws/src/YAREN2")

MODEL_PATHS = [
    os.path.join(WS, "yaren_chat2", "models", "LLM", "model.gguf"),
    os.path.join(WS, "yaren_face_display", "models", "LLM",
                 "models--MaziyarPanahi--Llama-3.2-1B-Instruct-GGUF",
                 "snapshots",
                 "b64ae94264258a3d7516a34a8c54928d32b19869",
                 "Llama-3.2-1B-Instruct.Q4_K_M.gguf"),
]

MAX_TURNS = 5  # Máximo de intercambios a recordar (user + assistant por turno)

SYSTEM_PROMPTS = {
    "es": (
        "Eres Yaren, un robot amigable que habla con niños de 7 a 12 años en hospitales.\n"
        "REGLAS ESTRICTAS:\n"
        "1. Respuestas CORTAS (máximo 2 oraciones).\n"
        "2. SIEMPRE termina tu respuesta con UNA sola pregunta abierta para continuar la conversación.\n"
        "3. Al inicio, pregunta primero el nombre, luego la edad, luego qué le gusta hacer.\n"
        "4. Nunca des consejos médicos ni diagnósticos.\n"
        "5. Usa lenguaje simple, positivo y apropiado para niños.\n"
        "6. NUNCA uses emojis, asteriscos ni descripciones de acciones como '(sonríe)' o '*saluda*'.\n"
        "7. Solo genera texto hablado, exactamente lo que dirías en voz alta."
    ),
    "en": (
        "You are Yaren, a friendly robot that talks with children aged 7 to 12 in hospitals.\n"
        "STRICT RULES:\n"
        "1. Keep answers SHORT (maximum 2 sentences).\n"
        "2. ALWAYS end your response with ONE open-ended question to keep the conversation going.\n"
        "3. At the start, first ask their name, then their age, then what they like to do.\n"
        "4. Never give medical advice or diagnoses.\n"
        "5. Use simple, positive language appropriate for children.\n"
        "6. NEVER use emojis, asterisks, or action descriptions like '(smiles)' or '*waves*'.\n"
        "7. Only generate spoken text, exactly what you would say out loud."
    )
}

class LLMLocalLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__("llm_local_lifecycle_node")
        self._llm = None
        self._action_server = None
        self._lock = threading.Lock()
        self._idioma = "es"
        self._is_active = False
        self._history = []

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.lang_sub = self.create_subscription(String, "/yaren/current_language", self._idioma_callback, qos)

    def _idioma_callback(self, msg: String):
        if msg.data in ["es", "en"]:
            self._idioma = msg.data
            self.get_logger().info(f"🌐 Idioma LLM → {self._idioma}")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("⚙️ Configurando LLM local...")

        try:
            from llama_cpp import Llama
        except ImportError:
            self.get_logger().error("❌ llama-cpp-python no instalado")
            return TransitionCallbackReturn.FAILURE

        model_path = None
        for path in MODEL_PATHS:
            if os.path.isfile(path):
                model_path = path
                break

        if not model_path:
            self.get_logger().error("❌ No se encontró modelo GGUF")
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info(f"📂 Cargando modelo: {model_path}")

        try:
            self._llm = Llama(model_path=model_path, n_ctx=2048, n_threads=4, n_gpu_layers=0, verbose=False)
            self.get_logger().info("✅ Modelo cargado correctamente")
        except Exception as e:
            self.get_logger().error(f"💥 Error: {e}")
            return TransitionCallbackReturn.FAILURE

        self._action_server = ActionServer(
            self, ProcessResponse, "/response_llama_local", self._execute_callback,
            goal_callback=self._goal_callback, cancel_callback=self._cancel_callback
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._is_active = True
        self._history = []  # Historial limpio al activar
        self.get_logger().info("▶️ LLM local activado")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._is_active = False
        self._history = []  # Limpiar historial al desactivar
        self.get_logger().info("⏸️ LLM local desactivado")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        if self._action_server:
            self._action_server.destroy()
            self._action_server = None
        self._llm = None
        self._history = []
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        if self._action_server:
            self._action_server.destroy()
            self._action_server = None
        self._llm = None
        self._history = []
        return TransitionCallbackReturn.SUCCESS

    def _goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        feedback = ProcessResponse.Feedback()
        result = ProcessResponse.Result()

        if not self._is_active or self._llm is None:
            result.completed = False
            goal_handle.abort()
            return result

        user_text = goal_handle.request.input_text

        # Agregar mensaje del usuario al historial
        self._history.append({"role": "user", "content": user_text})

        # Construir mensajes: system + últimos MAX_TURNS turnos
        system_prompt = SYSTEM_PROMPTS.get(self._idioma, SYSTEM_PROMPTS["es"])
        messages = [{"role": "system", "content": system_prompt}] + self._history[-(MAX_TURNS * 2):]

        try:
            with self._lock:
                stream = self._llm.create_chat_completion(
                    messages=messages,
                    max_tokens=150,   # Reducido: respuestas cortas + menos carga
                    temperature=0.7,
                    top_p=0.9,
                    stream=True
                )
                buffer = ""
                full_response = ""  # Para guardar en historial

                for chunk in stream:
                    if goal_handle.is_cancel_requested:
                        result.completed = False
                        goal_handle.canceled()
                        return result

                    delta = (chunk.get("choices", [{}])[0].get("delta", {}).get("content", ""))
                    if not delta:
                        continue

                    buffer += delta
                    full_response += delta

                    # Detectar oraciones completas para enviar como feedback
                    for punct in [".", "!", "?"]:
                        if punct in buffer:
                            parts = buffer.split(punct, 1)
                            if len(parts) > 1:
                                sentence = parts[0] + punct
                                buffer = parts[1]
                                feedback.current_chunk = sentence
                                feedback.is_last_chunk = False
                                self.get_logger().info(sentence)          # ← agregar esto
                                goal_handle.publish_feedback(feedback)
                                break

                # Enviar lo que quede en el buffer
                if buffer.strip():
                    full_response += buffer.strip()
                    feedback.current_chunk = buffer.strip()
                    feedback.is_last_chunk = True
                    self.get_logger().info(buffer.strip())    # ← agregar esto
                    goal_handle.publish_feedback(feedback)

            # Guardar respuesta del asistente en historial
            if full_response.strip():
                self._history.append({"role": "assistant", "content": full_response.strip()})

            result.completed = True
            goal_handle.succeed()
            return result

        except Exception as e:
            self.get_logger().error(f"💥 Error: {e}")
            # Si falló, quitar el mensaje del usuario que se agregó
            if self._history and self._history[-1]["role"] == "user":
                self._history.pop()
            result.completed = False
            goal_handle.abort()
            return result


def main(args=None):
    rclpy.init(args=args)
    node = LLMLocalLifecycleNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        node.get_logger().info("🚀 Nodo LLM Local iniciado")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()