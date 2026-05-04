from langgraph.graph import StateGraph, END
from react_state import ChatState
from movement_agent import MovementDetectionAgent
from langchain_core.messages import AIMessage
from langchain_groq import ChatGroq
from langgraph.checkpoint.memory import MemorySaver
from config import CONFIGURATIONS

class StateGraphLLM:
    def __init__(self, node):
        self.node = node

        self.memory = MemorySaver()
        self.llm = ChatGroq(model=CONFIGURATIONS['model'], streaming=True, 
                            max_tokens=CONFIGURATIONS['max_completion_tokens'], temperature=CONFIGURATIONS['temperature'],
                            model_kwargs={"top_p": CONFIGURATIONS['top_p']})
        
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
        """Procesa la información del usuario y ejecuta el grafo de estados"""
        self.node.get_logger().info("Procesando información del usuario...")

        result = self.app.invoke(state, config=config_id)

        return result
    
    def check_movement_intent(self, state: ChatState):
        user_text = state["messages"][-1].content if state["messages"] else ""
        self.node.get_logger().info(f"🔍 Input: '{user_text}'")
        
        movement_result = self.movement_agent.process_movement_intent(state["messages"])
        
        # Logging detallado para ver qué método se usó
        self.node.get_logger().info(
            f"🤖 Detection: {movement_result.get('movement_detected')} | "
            f"Type: {movement_result.get('movement_type')} | "
            f"Method: {movement_result.get('detection_method', 'unknown')}"
        )
        
        if movement_result.get("joints_to_move"):
            self.node.get_logger().info(f"🔧 Joints: {movement_result['joints_to_move']}")

        return {
            "messages": state["messages"],
            "movement_detected": movement_result["movement_detected"],
            "movement_intent": movement_result,
            "robot_action_required": movement_result["movement_detected"]
        }

    def go_to_next_state(self, state: ChatState):
        """Determina si se debe ir al siguiente estado basado en la intención de movimiento"""
        self.node.get_logger().info(f"Checking if movement is required: {state['robot_action_required']}")

        if state["robot_action_required"]:
            self.node.get_logger().info("Movement detected, proceeding to execute movement")
            return "execute_movement"
        else:
            self.node.get_logger().info("No movement detected, proceeding to chatbot")
            return "chatbot"
        
    def chatbot(self, state: ChatState):
        return {
            "messages": [self.llm.invoke(state["messages"])]
        }
    
    def execute_movement(self, state: ChatState):
        """Ejecuta el movimiento del robot basado en la intención detectada"""        
        self.node.get_logger().info(f"Movement detected: {state['movement_intent']['movement_type']}")

        mensaje_ai_predefinido = AIMessage(
            content=f"¡He realizado el movimiento de {state['movement_intent']['movement_type']}! ¿Hay algo más en lo que pueda ayudarte?"
        )

        state["messages"].append(mensaje_ai_predefinido)

        return {
            "messages": state["messages"]
        }