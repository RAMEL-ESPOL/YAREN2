cmake_minimum_required(VERSION 3.8)
project(yaren_chatbot)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/chat_node.py
  scripts/kb.py
  scripts/dialogue.py
  scripts/stt_node.py
  scripts/tts_node.py
  scripts/voice_utils.py
  scripts/audio_check.py
  scripts/descargar_modelos.sh
  scripts/build_kb.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch resource data
  DESTINATION share/${PROJECT_NAME})

ament_package()
