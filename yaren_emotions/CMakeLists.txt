cmake_minimum_required(VERSION 3.8)
project(yaren_emotions)

# ========== Configuración común ==========
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ========== Configuración para Python ==========
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)

install(PROGRAMS
  scripts/detect_emotion.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/detect_emotion_tflite.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/csi_cam_pub.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
# ========== Instalación común ==========
install(DIRECTORY
  models scripts utils
  DESTINATION share/${PROJECT_NAME}
)

# ========== Exportación de dependencias ==========
ament_export_dependencies(
  rclpy
  sensor_msgs
  cv_bridge
  std_msgs
)

ament_package()
