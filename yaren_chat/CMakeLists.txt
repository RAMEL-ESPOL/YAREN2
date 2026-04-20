cmake_minimum_required(VERSION 3.8)
project(yaren_chat)

# ========== C++ Configuration ==========
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaren_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(lifecycle_msgs REQUIRED)

add_executable(control_manager_node src/control_manager_node.cpp)
ament_target_dependencies(control_manager_node 
  rclcpp
  rclcpp_lifecycle
  std_msgs
  lifecycle_msgs
)

include_directories(
  include
  /usr/local/include 
)

# add_executable(llm_lifecycle_node src/llm_lifecycle_node.cpp)
# ament_target_dependencies(llm_lifecycle_node 
#   rclcpp 
#   sensor_msgs 
#   yaren_interfaces 
#   std_msgs
#   rclcpp_lifecycle
#   rclcpp_action
# )

# # Include directories for llama and ggml
# link_directories(/usr/local/lib)

# # Link libraries for llama and ggml
# target_link_libraries(llm_lifecycle_node
#   /usr/local/lib/libllama.so 
#   /usr/local/lib/libggml.so
# )

# ========== Python Configuration ==========
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

install(PROGRAMS
  scripts/stt_lifecycle_node.py
  scripts/tts_lifecycle_node.py
  scripts/llm_lifecycle_node.py
  scripts/llm_lifecycle_node_sin_agentes.py
  scripts/utils/text_processor.py
  scripts/utils/react_state.py
  scripts/utils/movement_agent.py
  scripts/utils/state_graph.py
  scripts/utils/config.py
  DESTINATION lib/${PROJECT_NAME}
)

# ========== Comun Installation ==========
install(DIRECTORY
  launch
  models
  scripts
  src
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  control_manager_node
  # llm_lifecycle_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rclpy
  sensor_msgs
  std_msgs
  yaren_interfaces
  rclcpp_lifecycle
  rclcpp_action
  lifecycle_msgs
)

ament_export_include_directories(include)
ament_export_libraries(llm_lifecycle_node)
ament_package()