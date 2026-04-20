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
        self.llm = ChatGroq(
            model="llama-3.1-8b-instant",
            streaming=True,
            max_tokens=CONFIGURATIONS['max_completion_tokens'],
            temperature=CONFIGURATIONS['temperature'],
            model_kwargs={"top_p": CONFIGURATIONS['top_p']}
        )
        self.setup_graph()

    def setup_graph(self):
        """Construye el grafo de LangGraph para detección y planificación de movimiento"""
        graph = StateGraph(MovementState)
        graph.add_node("detect_movement", self.detect_movement_intent)
        graph.add_node("plan_movement", self.plan_joint_movement)
        graph.set_entry_point("detect_movement")
        graph.add_edge("detect_movement", "plan_movement")
        graph.add_edge("plan_movement", END)
        self.movement_graph = graph.compile()

    def _extract_json_from_response(self, text: str) -> dict:
        """Extrae JSON válido y normaliza el nombre del movimiento"""
        text = text.strip()
        
        # Intento 1: JSON directo
        try:
            data = json.loads(text)
            return self._normalize_movement_type(data)
        except Exception:
            pass
        
        # Intento 2: Buscar JSON entre llaves con regex
        match = re.search(r'\{[\s\S]*"movement_detected"[\s\S]*\}', text)
        if match:
            try:
                data = json.loads(match.group())
                return self._normalize_movement_type(data)
            except Exception:
                pass
        
        # Intento 3: Fallback manual basado en palabras clave
        text_lower = text.lower()
        result = {"movement_detected": False, "movement_type": "ninguno"}
        
        if any(kw in text_lower for kw in ["true", "sí", "si", "detectado", "movimiento"]):
            result["movement_detected"] = True
            
            # Brazos
            if "derech" in text_lower:
                result["movement_type"] = "levanta_el_brazo_derecho"
            elif "izquierd" in text_lower:
                result["movement_type"] = "levanta_el_brazo_izquierdo"
            elif "ambos" in text_lower or "dos brazos" in text_lower or "alza" in text_lower:
                result["movement_type"] = "alza_los_brazos"
            elif "estira" in text_lower or "extiende" in text_lower:
                result["movement_type"] = "estira_los_brazos"
            
            # Cabeza
            elif "cabeza" in text_lower:
                if "izquierd" in text_lower or "izq" in text_lower:
                    result["movement_type"] = "mira_a_la_izquierda"
                elif "derech" in text_lower or "der" in text_lower:
                    result["movement_type"] = "mira_a_la_derecha"
                else:
                    result["movement_type"] = "mueve_la_cabeza"
                    
            # Cuerpo
            elif "cuerpo" in text_lower:
                if "izquierd" in text_lower or "izq" in text_lower:
                    result["movement_type"] = "gira_a_la_izquierda"
                elif "derech" in text_lower or "der" in text_lower:
                    result["movement_type"] = "gira_a_la_derecha"
                else:
                    result["movement_type"] = "gira_el_cuerpo"
                    
            # Actitud
            elif "confus" in text_lower or "no entiendo" in text_lower or "loco" in text_lower:
                result["movement_type"] = "hazte_el_loco"
                
            # Reset
            elif "original" in text_lower or "inicial" in text_lower or "quieto" in text_lower:
                result["movement_type"] = "vuelve_a_la_posicion_original"
                
        return self._normalize_movement_type(result)

    def _normalize_movement_type(self, data: dict) -> dict:
        """Asegura que el movement_type sea exactamente uno de los válidos"""
        m_type = data.get("movement_type", "ninguno")
        
        # Mapeo de posibles variaciones a los nombres exactos usados en plan_joint_movement
        normalization_map = {
            "mira a la derecha": "mira_a_la_derecha",
            "mira_a_la derecha": "mira_a_la_derecha",
            "gira a la derecha": "gira_a_la_derecha",
            "gira_a_la derecha": "gira_a_la_derecha",
            "mira a la izquierda": "mira_a_la_izquierda",
            "gira a la izquierda": "gira_a_la_izquierda",
            "hazte el loco": "hazte_el_loco",
            "hazte_el loco": "hazte_el_loco",
            "vuelve a la posicion original": "vuelve_a_la_posicion_original",
            "vuelve_a_la posicion original": "vuelve_a_la_posicion_original",
        }
        
        # Si existe en el mapa, lo reemplaza. Si no, lo deja igual (esperando que sea correcto)
        if m_type in normalization_map:
            data["movement_type"] = normalization_map[m_type]
            
        return data

    def _regex_fallback(self, user_message: str) -> dict:
        """Detección ultra-rápida con regex si el LLM falla o tarda demasiado"""
        text = user_message.lower()
        
        patterns = {
            # Brazos individuales
            r"\b(brazo\s*derech|derech.*brazo|brazo\s*2)\b": "levanta_el_brazo_derecho",
            r"\b(brazo\s*izquierd|izquierd.*brazo|brazo\s*1)\b": "levanta_el_brazo_izquierdo",
            
            # Ambos brazos
            r"\b(alz|levanta|sube).*brazo.*(\d|ambos|los|dos|izquierd|derech)|brazos.*arriba\b": "alza_los_brazos",
            r"\b(estira|extiende).*brazo.*(\d|ambos|los|dos|izquierd|derech)|brazos.*estir\b": "estira_los_brazos",
            
            # Cabeza (Nombres exactos normalizados)
            r"\b(cabeza.*izquierd|izquierd.*cabe|mira.*izq|voltea.*izq)\b": "mira_a_la_izquierda",
            r"\b(cabeza.*derech|derech.*cabe|mira.*der|voltea.*der)\b": "mira_a_la_derecha",
            r"\b(cabeza|mirar|gira.*cabe|volte)\b": "mueve_la_cabeza",
            
            # Cuerpo (Nombres exactos normalizados)
            r"\b(cuerpo.*izquierd|izquierd.*cuerpo|gira.*izq|voltea.*izq)\b": "gira_a_la_izquierda",
            r"\b(cuerpo.*derech|derech.*cuerpo|gira.*der|voltea.*der)\b": "gira_a_la_derecha",
            r"\b(cuerpo|gira|date\s*la\s*vuelta|volt[eé]ate)\b": "gira_el_cuerpo",
            
            # Actitud
            r"\b(confus|no entiendo|qué|eh|loco)\b": "hazte_el_loco",
            
            # Reset
            r"\b(posici[oó]n\s*(original|inicial)|vuelve\s*a\s*(estar\s*)?quieto|regresa)\b": "vuelve_a_la_posicion_original",
        }
        
        for pattern, movement_type in patterns.items():
            if re.search(pattern, text, re.IGNORECASE):
                return {
                    "movement_detected": True,
                    "movement_type": movement_type,
                    "method": "regex_fallback"
                }
        
        return {"movement_detected": False, "movement_type": "ninguno", "method": "regex_fallback"}

    def detect_movement_intent(self, state: MovementState):
        """Nodo del grafo: detecta intención de movimiento con LLM + fallback seguro"""
        user_message = state["messages"][-1].content if state["messages"] else ""
        
        detection_prompt = f"""Analiza si el usuario quiere movimiento físico del robot.
Mensaje: "{user_message}"

TIPOS VÁLIDOS (Usa EXACTAMENTE estos nombres):
levanta_el_brazo_derecho, levanta_el_brazo_izquierdo, alza_los_brazos, estira_los_brazos,
mueve_la_cabeza, mira_a_la_izquierda, mira_a_la_derecha,
gira_el_cuerpo, gira_a_la_izquierda, gira_a_la_derecha,
hazte_el_loco, vuelve_a_la_posicion_original, ninguno

Responde SOLO con este JSON: {{"movement_detected": true/false, "movement_type": "TIPO_VÁLIDO"}}"""

        try:
            response = self.llm.invoke([SystemMessage(content=detection_prompt)])
            result = self._extract_json_from_response(response.content)
            
            if "movement_detected" not in result or "movement_type" not in result:
                raise ValueError("JSON incompleto o mal formado")
                
        except Exception as e:
            result = self._regex_fallback(user_message)
            result["method"] = "llm_failed_regex_fallback"

        # Seguridad adicional: si el LLM no detectó movimiento, verificar con regex por si acaso
        if not result.get("movement_detected"):
            regex_result = self._regex_fallback(user_message)
            if regex_result["movement_detected"]:
                result = regex_result
                result["method"] = "llm+regex_fallback"
            else:
                result["method"] = result.get("method", "llm")

        return {
            "movement_detected": result["movement_detected"],
            "movement_type": result["movement_type"],
            "detection_method": result.get("method", "unknown")
        }

    def plan_joint_movement(self, state: MovementState):
        """Nodo del grafo: mapea tipo de movimiento a posiciones de joints"""
        movement_type = state.get("movement_type", "ninguno")
        joints_to_move = {}

        # === MOVIMIENTOS DEFINIDOS ===
        if movement_type == "levanta_el_brazo_derecho":
            joints_to_move = {
                "joint_5": 3.0, "joint_6": 0.0, "joint_7": -3.0,
                "joint_8": 0.0, "joint_12": 0.5
            }
        elif movement_type == "levanta_el_brazo_izquierdo":
            joints_to_move = {
                "joint_8": 0.5, "joint_9": -3.0, "joint_10": 0.0,
                "joint_11": 3.0, "joint_12": 0.0
            }
        elif movement_type == "mueve_la_cabeza":
            joints_to_move = {
                "joint_3": 0.8, "joint_4": 0.2,
                "joint_8": 0.5, "joint_12": 0.5
            }
        elif movement_type == "gira_el_cuerpo":
            joints_to_move = {
                "joint_1": 1.5, "joint_8": 0.5, "joint_12": 0.5
            }
        elif movement_type == "vuelve_a_la_posicion_original":
            joints_to_move = {
                "joint_1": 0.0, "joint_2": 0.0, "joint_3": 0.0, "joint_4": 0.0,
                "joint_5": 0.0, "joint_6": 0.0, "joint_7": 0.0, "joint_8": 0.5,
                "joint_9": 0.0, "joint_10": 0.0, "joint_11": 0.0, "joint_12": 0.5,
            }
        elif movement_type == "alza_los_brazos":
            joints_to_move = {
                "joint_5": 3.0, "joint_6": 0.0, "joint_7": -3.0, "joint_8": 0.0,
                "joint_9": -3.0, "joint_10": 0.0, "joint_11": 3.0, "joint_12": 0.0,
            }
        elif movement_type == "mira_a_la_izquierda":
            joints_to_move = {"joint_3": -0.8, "joint_4": 0.2, "joint_8": 0.5, "joint_12": 0.5}
        elif movement_type == "mira_a_la_derecha":
            joints_to_move = {"joint_3": 0.8, "joint_4": 0.2, "joint_8": 0.5, "joint_12": 0.5}
        elif movement_type == "gira_a_la_izquierda":
            joints_to_move = {"joint_1": -1.5, "joint_8": 0.5, "joint_12": 0.5}
        elif movement_type == "gira_a_la_derecha":
            joints_to_move = {"joint_1": 1.5, "joint_8": 0.5, "joint_12": 0.5}
        elif movement_type == "estira_los_brazos":
            joints_to_move = {"joint_5": 1.5, "joint_9": -1.5}
        elif movement_type == "hazte_el_loco":
            joints_to_move = {
                "joint_5": 3.0, "joint_6": 0.5, "joint_7": -1.5, "joint_8": 1.0,
                "joint_9": -3.0, "joint_10": 0.5, "joint_11": 1.5, "joint_12": 1.0
            }
        
        # === VALIDACIÓN FINAL ===
        # Si se detectó movimiento pero no hay joints definidos (ej. tipo desconocido),
        # forzamos un retorno vacío o un reset seguro para evitar errores en el controlador.
        if state.get("movement_detected") and not joints_to_move:
            # Opción A: Loggear advertencia y devolver vacío (el controlador ignorará)
            # Opción B: Forzar reset a posición original por seguridad
            # Aquí elegimos devolver vacío pero podrías cambiar a 'vuelve_a_la_posicion_original' si prefieres seguridad.
            pass 

        return {"joints_to_move": joints_to_move}

    def process_movement_intent(self, messages):
        """Punto de entrada: invoca el grafo y devuelve el resultado final"""
        initial_state = {
            "messages": messages,
            "movement_detected": False,
            "movement_type": "ninguno",
            "joints_to_move": {}
        }
        
        result = self.movement_graph.invoke(initial_state)
        return result