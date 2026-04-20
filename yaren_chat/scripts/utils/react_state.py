from typing import TypedDict, Annotated, Optional
from langgraph.graph import add_messages

class ChatState(TypedDict): 
    messages: Annotated[list, add_messages]
    movement_intent: Optional[dict]
    robot_action_required: Optional[bool]

class MovementState(TypedDict):
    messages: Annotated[list, "add_messages"]
    movement_detected: bool
    movement_type: str
    joints_to_move: dict

class SimpleChatState(TypedDict): 
    messages: Annotated[list, add_messages]