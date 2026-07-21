#!/usr/bin/env python3

from react_state import MovementState
from langgraph.graph import StateGraph, END
from langchain_groq import ChatGroq
from langchain_core.messages import SystemMessage
from config import CONFIGURATIONS
import re
import json


class MovementDetectionAgent:
    def __init__(self):
        # LLM solo se usa si el regex falla completamente
        self._llm = None

    @property
    def llm(self):
        """Lazy init — solo carga el LLM si realmente lo necesita."""
        if self._llm is None:
            self._llm = ChatGroq(
                model="llama-3.1-8b-instant",
                streaming=False,          # No necesitamos streaming para detección
                max_tokens=60,            # Solo necesita responder un JSON corto
                temperature=0.0,          # Determinístico para clasificación
                model_kwargs={"top_p": 1.0}
            )
        return self._llm

    def setup_graph(self):
        graph = StateGraph(MovementState)
        graph.add_node("detect_movement", self.detect_movement_intent)
        graph.add_node("plan_movement", self.plan_joint_movement)
        graph.set_entry_point("detect_movement")
        graph.add_edge("detect_movement", "plan_movement")
        graph.add_edge("plan_movement", END)
        self.movement_graph = graph.compile()

    def __init__(self):
        self._llm = None
        self.setup_graph()

    VALID_MOVEMENTS = {
        "levanta_el_brazo_derecho", "levanta_el_brazo_izquierdo",
        "alza_los_brazos", "estira_los_brazos", "mueve_la_cabeza",
        "mira_a_la_izquierda", "mira_a_la_derecha", "gira_el_cuerpo",
        "gira_a_la_izquierda", "gira_a_la_derecha",
        "hazte_el_loco", "vuelve_a_la_posicion_original", "ninguno"
    }

    JOINT_MAP = {
        "levanta_el_brazo_derecho":   {"joint_5": 3.0,  "joint_6": 0.0, "joint_7": -3.0, "joint_8": 0.0,  "joint_12": 0.5},
        "levanta_el_brazo_izquierdo": {"joint_8": 0.5,  "joint_9": -3.0,"joint_10": 0.0, "joint_11": 3.0, "joint_12": 0.0},
        "alza_los_brazos":            {"joint_5": 3.0,  "joint_6": 0.0, "joint_7": -3.0, "joint_8": 0.0,  "joint_9": -3.0, "joint_10": 0.0, "joint_11": 3.0, "joint_12": 0.0},
        "estira_los_brazos":          {"joint_5": 1.5,  "joint_9": -1.5},
        "mueve_la_cabeza":            {"joint_3": 0.8,  "joint_4": 0.2, "joint_8": 0.5,  "joint_12": 0.5},
        "mira_a_la_izquierda":        {"joint_3": -0.8, "joint_4": 0.2, "joint_8": 0.5,  "joint_12": 0.5},
        "mira_a_la_derecha":          {"joint_3": 0.8,  "joint_4": 0.2, "joint_8": 0.5,  "joint_12": 0.5},
        "gira_el_cuerpo":             {"joint_1": 1.5,  "joint_8": 0.5, "joint_12": 0.5},
        "gira_a_la_izquierda":        {"joint_1": -1.5, "joint_8": 0.5, "joint_12": 0.5},
        "gira_a_la_derecha":          {"joint_1": 1.5,  "joint_8": 0.5, "joint_12": 0.5},
        "hazte_el_loco":              {"joint_5": 3.0,  "joint_6": 0.5, "joint_7": -1.5, "joint_8": 1.0,  "joint_9": -3.0, "joint_10": 0.5, "joint_11": 1.5, "joint_12": 1.0},
        "vuelve_a_la_posicion_original": {
            "joint_1": 0.0,  "joint_2": 0.0,  "joint_3": 0.0,  "joint_4": 0.0,
            "joint_5": 0.0,  "joint_6": 0.0,  "joint_7": 0.0,  "joint_8": 0.5,
            "joint_9": 0.0,  "joint_10": 0.0, "joint_11": 0.0, "joint_12": 0.5,
        },
    }

    # ── Regex (≈0ms, sin API) ─────────────────────────────────────────
    def _regex_detect(self, text: str) -> dict:
        t = text.lower()

        # Bloque de empatía — prioridad máxima
        if re.search(
            r"\b(triste|llorar|dolor|duele|roto|miedo|solo|sola|asustad|pena|"
            r"sad|cry|pain|hurt|broken|scared|fear|alone)\b", t
        ):
            return {"movement_detected": False, "movement_type": "ninguno", "method": "regex_empathy"}

        patterns = [
            # Brazos individuales
            (r"(?:levanta|sube|alza|raise|lift)\s+(?:el\s+|your\s+)?(?:brazo|arm)\s+(?:derech|right|1)", "levanta_el_brazo_derecho"),
            (r"(?:levanta|sube|alza|raise|lift)\s+(?:el\s+|your\s+)?(?:brazo|arm)\s+(?:izquierd|left|2)",  "levanta_el_brazo_izquierdo"),
            # Ambos brazos
            (r"(?:alz|levanta|sube|raise|lift)\s+(?:los\s+|your\s+)?(?:brazos|arms)",  "alza_los_brazos"),
            (r"(?:estira|extiende|stretch|extend)\s+(?:los\s+|your\s+)?(?:brazos|arms)", "estira_los_brazos"),
            (r"(?:ambos|dos|both)\s+(?:brazos|arms)",  "alza_los_brazos"),
            (r"(?:brazos|arms)\s+(?:arriba|alto|up)",  "alza_los_brazos"),
            # Cabeza
            (r"(?:cabeza|head|mira|look|voltea|turn).*(?:izquierd|izq|left)|(?:izquierd|izq|left).*(?:cabeza|head)", "mira_a_la_izquierda"),
            (r"(?:cabeza|head|mira|look|voltea|turn).*(?:derech|der|right)|(?:derech|der|right).*(?:cabeza|head)", "mira_a_la_derecha"),
            (r"\b(?:cabeza|head|gira.*cabe|turn.*head)\b", "mueve_la_cabeza"),
            # Cuerpo
            (r"(?:cuerpo|body|gira|turn).*(?:izquierd|izq|left)|(?:izquierd|izq|left).*(?:cuerpo|body)", "gira_a_la_izquierda"),
            (r"(?:cuerpo|body|gira|turn).*(?:derech|der|right)|(?:derech|der|right).*(?:cuerpo|body)", "gira_a_la_derecha"),
            (r"\b(?:cuerpo|body|date\s*la\s*vuelta|volt[eé]ate|turn\s*around)\b", "gira_el_cuerpo"),
            # Actitud
            (r"\b(?:no entiendo|no comprendo|confused|don.t understand|loco)\b", "hazte_el_loco"),
            # Reset
            (r"\b(?:posici[oó]n\s*(?:original|inicial)|original\s*position|quieto|reset|regresa|return)\b", "vuelve_a_la_posicion_original"),
        ]

        for pattern, movement_type in patterns:
            if re.search(pattern, t, re.IGNORECASE):
                return {"movement_detected": True, "movement_type": movement_type, "method": "regex"}

        return {"movement_detected": False, "movement_type": "ninguno", "method": "regex"}

    # ── LLM fallback (solo si regex no detectó nada concreto) ─────────
    def _llm_detect(self, user_message: str) -> dict:
        prompt = (
            f'Clasifica si hay una orden de movimiento físico en: "{user_message}"\n'
            f'Tipos válidos: {", ".join(sorted(self.VALID_MOVEMENTS - {"ninguno"}))}\n'
            'Responde SOLO JSON: {"movement_detected": true/false, "movement_type": "TIPO"}'
        )
        try:
            response = self.llm.invoke([SystemMessage(content=prompt)])
            raw = response.content.strip()
            # Extraer JSON
            match = re.search(r'\{[^}]+\}', raw)
            data = json.loads(match.group() if match else raw)
            m_type = data.get("movement_type", "ninguno").replace(" ", "_")
            if m_type not in self.VALID_MOVEMENTS:
                m_type = "ninguno"
            detected = m_type != "ninguno" and data.get("movement_detected", False)
            return {"movement_detected": detected, "movement_type": m_type, "method": "llm"}
        except Exception:
            return {"movement_detected": False, "movement_type": "ninguno", "method": "llm_error"}

    # ── Nodos del grafo ───────────────────────────────────────────────
    def detect_movement_intent(self, state: MovementState):
        user_message = state["messages"][-1].content if state["messages"] else ""

        # 1. Regex primero — rápido y sin costo de API
        result = self._regex_detect(user_message)

        # 2. Solo si el regex no encontró nada y el mensaje parece una orden,
        #    consultamos el LLM (evita llamada innecesaria en conversación normal)
        if not result["movement_detected"]:
            action_keywords = r"\b(mueve|gira|levanta|alza|estira|mira|voltea|move|turn|raise|lift|look|rotate)\b"
            if re.search(action_keywords, user_message, re.IGNORECASE):
                result = self._llm_detect(user_message)

        return {
            "movement_detected": result["movement_detected"],
            "movement_type":     result["movement_type"],
            "detection_method":  result["method"],
        }

    def plan_joint_movement(self, state: MovementState):
        movement_type = state.get("movement_type", "ninguno")
        joints = {}

        if state.get("movement_detected"):
            joints = self.JOINT_MAP.get(movement_type, {})

        return {"joints_to_move": joints}

    def process_movement_intent(self, messages):
        result = self.movement_graph.invoke({
            "messages":          messages,
            "movement_detected": False,
            "movement_type":     "ninguno",
            "joints_to_move":    {},
        })
        return result