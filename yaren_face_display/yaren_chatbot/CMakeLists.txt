cmake_minimum_required(VERSION 3.8)
project(yaren_chatbot)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/chat_node.py
  scripts/kb.py
  scripts/build_kb.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch resource data
  DESTINATION share/${PROJECT_NAME})

ament_package()
