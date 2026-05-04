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
        """Extrae JSON válido y normaliza el nombre del movimiento.
        REGLA: Si movement_type es 'ninguno', movement_detected DEBE ser False.
        """
        text = text.strip()
        
        # Intento 1: JSON directo
        try:
            data = json.loads(text)
            return self._normalize_and_validate(data)
        except Exception:
            pass
        
        # Intento 2: Buscar JSON entre llaves con regex
        match = re.search(r'\{[\s\S]*"movement_detected"[\s\S]*\}', text)
        if match:
            try:
                data = json.loads(match.group())
                return self._normalize_and_validate(data)
            except Exception:
                pass
        
        # Intento 3: Fallback manual basado en palabras clave (Solo si detecta movimiento real)
        text_lower = text.lower()
        result = {"movement_detected": False, "movement_type": "ninguno"}
        
        # Solo buscamos movimientos activos aquí. 
        # Si no encuentra ninguno activo, devuelve False/Ninguno por defecto.
        
        if any(kw in text_lower for kw in ["true", "sí", "si", "detectado", "movimiento"]):
            
            # Brazos
            if "derech" in text_lower and "brazo" in text_lower:
                result["movement_detected"] = True
                result["movement_type"] = "levanta_el_brazo_derecho"
            elif "izquierd" in text_lower and "brazo" in text_lower:
                result["movement_detected"] = True
                result["movement_type"] = "levanta_el_brazo_izquierdo"
            elif ("ambos" in text_lower or "dos brazos" in text_lower or "alza" in text_lower) and "brazo" in text_lower:
                result["movement_detected"] = True
                result["movement_type"] = "alza_los_brazos"
            elif ("estira" in text_lower or "extiende" in text_lower) and "brazo" in text_lower:
                result["movement_detected"] = True
                result["movement_type"] = "estira_los_brazos"
            
            # Cabeza
            elif "cabeza" in text_lower:
                result["movement_detected"] = True
                if "izquierd" in text_lower or "izq" in text_lower:
                    result["movement_type"] = "mira_a_la_izquierda"
                elif "derech" in text_lower or "der" in text_lower:
                    result["movement_type"] = "mira_a_la_derecha"
                else:
                    result["movement_type"] = "mueve_la_cabeza"
                    
            # Cuerpo
            elif "cuerpo" in text_lower:
                result["movement_detected"] = True
                if "izquierd" in text_lower or "izq" in text_lower:
                    result["movement_type"] = "gira_a_la_izquierda"
                elif "derech" in text_lower or "der" in text_lower:
                    result["movement_type"] = "gira_a_la_derecha"
                else:
                    result["movement_type"] = "gira_el_cuerpo"
                    
            # Actitud
            elif "confus" in text_lower or "no entiendo" in text_lower or "loco" in text_lower:
                result["movement_detected"] = True
                result["movement_type"] = "hazte_el_loco"
                
            # Reset
            elif "original" in text_lower or "inicial" in text_lower or "quieto" in text_lower:
                result["movement_detected"] = True
                result["movement_type"] = "vuelve_a_la_posicion_original"
                
        return self._normalize_and_validate(result)

    VALID_MOVEMENTS = {
    "levanta_el_brazo_derecho", "levanta_el_brazo_izquierdo",
    "alza_los_brazos", "estira_los_brazos", "mueve_la_cabeza",
    "mira_a_la_izquierda", "mira_a_la_derecha", "gira_el_cuerpo",
    "gira_a_la_izquierda", "gira_a_la_derecha",
    "hazte_el_loco", "vuelve_a_la_posicion_original", "ninguno"
    }

    def _normalize_and_validate(self, data: dict) -> dict:
        m_type = data.get("movement_type", "ninguno")
        
        # Normalizar espacios a guiones bajos
        m_type = m_type.replace(" ", "_")
        
        # Si no es válido, forzar a ninguno
        if m_type not in VALID_MOVEMENTS:
            m_type = "ninguno"
        
        data["movement_type"] = m_type
        if m_type == "ninguno":
            data["movement_detected"] = False
        return data

    def _regex_fallback(self, user_message: str) -> dict:
        """Detección ultra-rápida con regex si el LLM falla o tarda demasiado"""
        text = user_message.lower()
        
        # 1. PRIORIDAD: Emociones Tristes/Dolor -> NO MOVER (Empatía)
        if re.search(r"\b(triste|llorar|dolor|duele|roto|miedo|solo|sola|asustad|pena)\b", text):
            return {
                "movement_detected": False,
                "movement_type": "ninguno",
                "method": "regex_empathy_block"
            }

        patterns = {
            #  BRAZOS INDIVIDUALES (Prioridad ALTA: van primero para capturar antes que "ambos")
            r"(?:levanta|sube|alza)\s+(?:el\s+)?brazo\s+(?:derech|1)": "levanta_el_brazo_derecho",
            r"(?:levanta|sube|alza)\s+(?:el\s+)?brazo\s+(?:izquierd|2)": "levanta_el_brazo_izquierdo",
            
            #  AMBOS BRAZOS (Solo se activan si detectan PLURAL o palabras de cantidad)
            r"(?:alz|levanta|sube)\s+(?:los\s+)?brazos": "alza_los_brazos",
            r"(?:estira|extiende)\s+(?:los\s+)?brazos": "estira_los_brazos",
            r"(?:ambos|dos)\s+brazos": "alza_los_brazos",
            r"brazos\s+(?:arriba|alto)": "alza_los_brazos",
            
            #  CABEZA (Sin \b final para que "izq" coincida con "izquierda")
            r"(cabeza.*(izquierd|izq)|(izquierd|izq).*cabe|mira.*(izquierd|izq)|voltea.*(izquierd|izq))": "mira_a_la_izquierda",
            r"(cabeza.*(derech|der)|(derech|der).*cabe|mira.*(derech|der)|voltea.*(derech|der))": "mira_a_la_derecha",
            r"\b(cabeza|mirar|gira.*cabe|volte)\b": "mueve_la_cabeza",
            
            # 🔄 CUERPO (Direcciones específicas antes del genérico)
            r"(cuerpo.*(izquierd|izq)|(izquierd|izq).*cuerpo|gira.*(izquierd|izq)|voltea.*(izquierd|izq))": "gira_a_la_izquierda",
            r"(cuerpo.*(derech|der)|(derech|der).*cuerpo|gira.*(derech|der)|voltea.*(derech|der))": "gira_a_la_derecha",
            r"\b(cuerpo|gira\s*(el\s*)?cuerpo|date\s*la\s*vuelta|volt[eé]ate)\b": "gira_el_cuerpo",
            
            #  ACTITUD
            r"\b(confus|no entiendo|qué|eh|loco)\b": "hazte_el_loco",
            
            #  RESET
            r"\b(posici[oó]n\s*(original|inicial)|vuelve\s*a\s*(estar\s*)?quieto|regresa)\b": "vuelve_a_la_posicion_original",
        }
        
        for pattern, movement_type in patterns.items():
            if re.search(pattern, text, re.IGNORECASE):
                return {
                    "movement_detected": True,
                    "movement_type": movement_type,
                    "method": "regex_fallback"
                }
        
        # Si no coincide con ningún patrón de movimiento activo
        return {"movement_detected": False, "movement_type": "ninguno", "method": "regex_fallback"}

    def detect_movement_intent(self, state: MovementState):
        """Nodo del grafo: detecta intención de movimiento con LLM + fallback seguro"""
        user_message = state["messages"][-1].content if state["messages"] else ""
        
        detection_prompt = f"""Analiza si el usuario quiere movimiento físico del robot.
Mensaje: "{user_message}"

REGLAS IMPORTANTES:
1. Si el usuario expresa tristeza, dolor, miedo o pérdida (ej: "estoy triste", "me duele", "se rompió"), NO muevas el robot. Responde con movement_detected: false.
2. Solo detecta movimiento si hay una orden clara o un gesto de interacción activa (saludar, señalar, girar).

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

        # Seguridad adicional: Verificar con regex por si el LLM pasó algo extraño
        # Pero respetamos la regla de empatía del regex primero
        if not result.get("movement_detected"):
            # Si el LLM dijo que no, verificamos si el regex ve algo obvio (como "levanta brazo")
            # pero ignoramos si el regex dice "no" porque ya estamos en no.
            regex_check = self._regex_fallback(user_message)
            if regex_check["movement_detected"]:
                 # El regex vio una orden clara que el LLM quizás perdió
                 result = regex_check
                 result["method"] = "llm_missed_regex_found"
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
        # Si movement_detected es False, aseguramos que joints esté vacío
        if not state.get("movement_detected"):
            joints_to_move = {}

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