# YAREN2 - Social Robot Platform

YAREN2 is an intelligent social robot platform built on ROS 2 Humble, designed for natural human-robot interaction using voice commands, LLMs (Groq), and expressive movements.

## Prerequisites

- **OS:** Ubuntu 22.04 LTS
- **ROS Distribution:** ROS 2 Humble Hawksbill
- **Hardware:** PC or NVIDIA Jetson Orin Nano (recommended for better performance)
- **Python:** 3.10+

## ️ Installation

### 1. System Dependencies

Install the required system libraries for audio, serial communication, and ROS 2 control:

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
### 2. Python Dependencies

It is recommended to use a virtual environment. Navigate to your workspace and install the Python packages:
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
### 3. AI Models (STT & TTS)

You need to download the Voice Recognition (STT) and Text-to-Speech (TTS) models manually.
Structure:
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
### 4. Groq API Key

YAREN uses Groq for fast LLM inference. Get your free API key at Groq Console.
Add it to your .bashrc:
```
echo 'export GROQ_API_KEY="gsk_YOUR_API_KEY_HERE"' >> ~/.bashrc
source ~/.bashrc
```

### 5. Build the Workspace
```
cd ~/robotis_ws
colcon build --symlink-install
source install/setup.bash
```
### 6. Quick Start with Alias To simplify starting the robot, add the following alias to your ~/.bashrc file. This will source ROS 2, activate the virtual environment, and launch YAREN.
```
#1.Open your bash configuration:
nano ~/.bashrc
#2.Add these lines at the end of the file:
alias iniciar_yaren='source /opt/ros/humble/setup.bash && cd ~/robotis_ws && source venv_yaren/bin/activate && source install/setup.bash
#3.Save and exit (Ctrl+O, Enter, Ctrl+X), then reload:
source ~/.bashrc
#4.Start YAREN anytime by typing:
iniciar_yaren
```

### 7.Powering the Motors To initialize communication with the motors via U2D2
```
ros2 launch yaren_u2d2 yaren_robot.launch.py
```
### 8. Camera Filters YAREN2 includes augmented reality filters. Choose your preferred mode
```
    Accessory Filters:
    ros2 launch yaren_filters yaren_accesorios.launch.py

    Animal Filters:
    ros2 launch yaren_filters yaren_animales.launch.py
```
### 9.Emotion Detection To enable the robot to recognize user moods through computer vision

   ``` 
   ros2 launch yaren_emotions yaren_emotions.launch.py
   ```

### 10. Yaren Dice Game To launch the interactive game mode

    ```
    ros2 launch yaren_dice yaren_dice.launch.py
    ```

### 11. Yaren ChatBot The core of the social interaction. Choose between two modes

    ```
With Intelligent Agents (Recommended): Uses advanced logic and decision-making.
    ros2 launch yaren_chat yaren_chat.launch.py
    
    Without Agents: A more direct and simple chat interface.
   ros2 launch yaren_chat yaren_chat_sin_agentes.launch.py
```

### 12. Yaren Arm Mimic: 
```
Its not working yet :c
```

