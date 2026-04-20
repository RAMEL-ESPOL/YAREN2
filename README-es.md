# YAREN2 - Plataforma de Robot Social

YAREN2 es una plataforma inteligente de robot social construida sobre **ROS 2 Humble**, diseñada para la interacción natural humano-robot mediante comandos de voz, Modelos de Lenguaje Grande (LLMs vía Groq) y movimientos expresivos.

## Prerrequisitos

- **Sistema Operativo:** Ubuntu 22.04 LTS
- **Distribución ROS:** ROS 2 Humble Hawksbill
- **Hardware:** PC o NVIDIA Jetson Orin Nano (recomendado para mejor rendimiento)
- **Python:** 3.10+

## ️ Instalación

### 1. Dependencias del Sistema

Instala las librerías necesarias para audio, comunicación serial y control en ROS 2:

```bash
sudo apt update
sudo apt install -y \
    python3-pip \
    python3-venv \
    portaudio19-dev \
    python3-pyaudio \
    libusb-1.0-0-dev \
    git \
    curl \
    build-essential \
    cmake \
    ros-humble-control-msgs \
    ros-humble-trajectory-msgs \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers
```
### 2. Dependencias de Python

Se recomienda utilizar un entorno virtual para evitar conflictos. Navega a tu workspace e instala los paquetes necesarios:
```
cd ~/robotis_ws
source venv_yaren/bin/activate # If you already have the env created
pip install --upgrade pip
pip install \
    langchain \
    langgraph \
    langsmith \
    groq \
    langchain-groq \
    vosk \
    piper-tts \
    pyaudio \
    dynamixel-sdk \
    opencv-python \
    numpy \
    onnxruntime \
    pillow
```
### 3. Modelos de IA (STT y TTS)

Debes descargar manualmente los modelos de Reconocimiento de Voz (STT) y Síntesis de Voz (TTS) y colocarlos en la estructura correcta. 
Estructura de carpetas requerida:
```
src/YAREN2/yaren_chat/models/
├── STT/
│   └── vosk-model-es-0.42/  <-- Unzip content here (must contain final.mdl)
└── TTS/
    ├── es_ES-sharvard-medium.onnx
    └── es_ES-sharvard-medium.onnx.json
STT Model: Download vosk-model-small-es-0.42
    and extract it into models/STT/.
TTS Model: Download a Spanish Piper model (e.g., es_ES-sharvard-medium) from Piper Releases
    and place the .onnx and .json files in models/TTS/.
```
### 4. Clave API de Groq

YAREN usa Groq para inferencia rápida de LLM. Obtén tu clave gratuita en Groq Console.
Añádela a tu .bashrc:
```echo 'export GROQ_API_KEY="gsk_YOUR_API_KEY_HERE"' >> ~/.bashrc
source ~/.bashrc
```
### 5. Compilar el Workspace
```
cd ~/robotis_ws
colcon build --symlink-install
source install/setup.bash
```
### 6. Inicio Rápido con Alias Para simplificar el inicio del robot, añade el siguiente alias a tu archivo ~/.bashrc. Esto cargará ROS 2, activará el entorno virtual y lanzará YAREN.
```
#Abre tu configuración de bash:
nano ~/.bashrc
#2.Añade estas líneas al final del archivo:
alias iniciar_yaren='sudo chmod 666 /dev/ttyUSB0 && sudo setserial /dev/ttyUSB0 low_latency && source /opt/ros/humble/setup.bash && cd ~/robotis_ws && source venv_yaren/bin/activate && source install/setup.bash'
#3.Guarda y sal (Ctrl+O, Enter, Ctrl+X), luego recarga:
source ~/.bashrc
#4.Inicia YAREN en cualquier momento escribiendo:
iniciar_yaren
```
### 7. Prender los motores
```
ros2 launch yaren_u2d2 yaren_robot.launch.py
```

### 8. Prender los filtros de la camara
```
    # Para poner los filtros de accesorios:
    ros2 launch yaren_filters yaren_accesorios.launch.py

    # Para poner los filtros de animales:
    ros2 launch yaren_filters yaren_animales.launch.py
```
### 9. Prender la deteccion de emociones de la camara

 ```   
 # Para poner la deteccion de emociones:
ros2 launch yaren_emotions yaren_emotions.launch.py
```
### 10. Prender el juego Yaren Dice

  ```  
# Para poner el juego Yaren Dice:
ros2 launch yaren_dice yaren_dice.launch.py
```
### 11. Prender el Yaren ChatBot
```
    # Para poner Yaren ChatBot con agentes inteligentes:
    ros2 launch yaren_chat yaren_chat.launch.py

    # Para poner Yaren ChatBot sin agentes inteligentes:
    ros2 launch yaren_chat yaren_chat_sin_agentes.launch.py
```
### 12. Prender el Yaren Arm Mimic: 
```Todavia no vale :(```


