import re
import time
from langgraph.graph import StateGraph, END
from react_state import ChatState
from movement_agent import MovementDetectionAgent
from langchain_core.messages import AIMessage, SystemMessage, HumanMessage
from langchain_groq import ChatGroq
from langgraph.checkpoint.memory import MemorySaver
from config import CONFIGURATIONS

MAX_MSGS_SENT = 30  # mensajes (sin contar el system prompt) que se reenvian al LLM


class StateGraphLLM:
    def __init__(self, node):
        self.node = node
        self.on_sentence = None   # callback(str): se llama por cada frase apenas el LLM la genera

        self.memory = MemorySaver()
        self.llm = ChatGroq(
            model=CONFIGURATIONS['model'],
            streaming=True,
            max_tokens=CONFIGURATIONS['max_completion_tokens'],
            temperature=CONFIGURATIONS['temperature'],
            reasoning_effort="low",
            model_kwargs={"top_p": CONFIGURATIONS['top_p']},
        )
        

        self.movement_agent = MovementDetectionAgent()

        self.graph = StateGraph(ChatState)
        self.graph.add_node("check_movement", self.check_movement_intent)
        self.graph.add_node("chatbot", self.chatbot)
        self.graph.add_node("execute_movement", self.execute_movement)

        self.graph.set_entry_point("check_movement")

        self.graph.add_edge("chatbot", END)
        self.graph.add_edge("execute_movement", END)

        self.graph.add_conditional_edges(
            "check_movement",
            self.go_to_next_state,
        )

        self.app = self.graph.compile(checkpointer=self.memory)

    def process_info(self, state, config_id):
        """Procesa la informacion del usuario y ejecuta el grafo de estados"""
        self.node.get_logger().info("Procesando información del usuario...")
        result = self.app.invoke(state, config=config_id)
        return result

    def check_movement_intent(self, state: ChatState):
        t0 = time.time()
        user_text = state["messages"][-1].content if state["messages"] else ""
        self.node.get_logger().info(f"🔍 Input: '{user_text}'")

        movement_result = self.movement_agent.process_movement_intent(state["messages"])

        self.node.get_logger().info(
            f"🤖 Detection: {movement_result.get('movement_detected')} | "
            f"Type: {movement_result.get('movement_type')} | "
            f"Method: {movement_result.get('detection_method', 'unknown')}"
        )
        self.node.get_logger().info(f"⏱️ check_movement: {time.time()-t0:.2f}s")

        if movement_result.get("joints_to_move"):
            self.node.get_logger().info(f"🔧 Joints: {movement_result['joints_to_move']}")

        return {
            "messages": state["messages"],
            "movement_detected": movement_result["movement_detected"],
            "movement_intent": movement_result,
            "robot_action_required": movement_result["movement_detected"]
        }

    def go_to_next_state(self, state: ChatState):
        self.node.get_logger().info(f"Checking if movement is required: {state['robot_action_required']}")
        if state["robot_action_required"]:
            self.node.get_logger().info("Movement detected, proceeding to execute movement")
            return "execute_movement"
        self.node.get_logger().info("No movement detected, proceeding to chatbot")
        return "chatbot"

    # ---------- historial recortado ----------
    @staticmethod
    def _trim(messages):
        system = None
        rest = []
        for m in messages:
            if isinstance(m, SystemMessage):
                system = m            # nos quedamos con el ultimo
            else:
                rest.append(m)
        rest = rest[-MAX_MSGS_SENT:]
        while rest and not isinstance(rest[0], HumanMessage):
            rest.pop(0)               # que empiece con un mensaje del usuario
        return ([system] if system else []) + rest

    # ---------- streaming por frases ----------
    def _emit(self, text):
        text = text.strip()
        if text and self.on_sentence:
            self.on_sentence(text)

    def _flush(self, buf, first, final):
        while True:
            cut = -1
            m = re.search(r"[.!?\u2026]+(?=\s)", buf)
            if m:
                cut = m.end()
            elif first and len(buf) >= 30:
                m2 = re.search(r"^.{25,}?[,;:](?=\s)", buf, re.S)
                if m2:
                    cut = m2.end()
            if cut == -1:
                break
            self._emit(buf[:cut])
            buf = buf[cut:]
            first = False
        if final:
            self._emit(buf)
            buf = ""
        return buf, first

    def chatbot(self, state: ChatState):
        msgs = self._trim(state["messages"])
        self.node.get_logger().info(f"💬 Enviando {len(msgs)} mensajes al LLM (de {len(state['messages'])} en historial)")
        t0 = time.time()
        full, buf, first, got_first = "", "", True, False
        for chunk in self.llm.stream(msgs):
            piece = chunk.content or ""
            if not piece:
                continue
            if not got_first:
                got_first = True
                self.node.get_logger().info(f"⏱️ LLM primer token: {time.time()-t0:.2f}s")
            full += piece
            buf += piece
            buf, first = self._flush(buf, first, final=False)
        self._flush(buf, first, final=True)
        self.node.get_logger().info(f"⏱️ LLM respuesta completa: {time.time()-t0:.2f}s")
        return {"messages": [AIMessage(content=full)]}

    def execute_movement(self, state: ChatState):
        """Ejecuta el movimiento del robot basado en la intencion detectada"""
        self.node.get_logger().info(f"Movement detected: {state['movement_intent']['movement_type']}")

        idioma = getattr(self.node, 'idioma_actual', 'es')
        if idioma == "en":
            texto_respuesta = f"I have executed the {state['movement_intent']['movement_type']} movement! Is there anything else I can help you with?"
        else:
            texto_respuesta = f"¡He realizado el movimiento de {state['movement_intent']['movement_type']}! ¿Hay algo más en lo que pueda ayudarte?"

        state["messages"].append(AIMessage(content=texto_respuesta))
        return {"messages": state["messages"]}
