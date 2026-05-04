from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    # El primer argumento es el nombre interno del robot (como vimos en tu XML)
    moveit_config = (
        MoveItConfigsBuilder("yaren_description", package_name="yaren_moveit2_config")
        # Le decimos el nombre exacto de tu XACRO en la carpeta config
        .robot_description(file_path="config/yaren_description.urdf.xacro")
        # Le decimos el nombre exacto de tu SRDF en la carpeta config
        .robot_description_semantic(file_path="config/yaren_description_moveit.srdf")
        .to_moveit_configs()
    )
    
    return generate_demo_launch(moveit_config)