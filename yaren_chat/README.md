
# Yaren Chat

Funcionalidad que permite tener conversaciones en tiempo real con Yaren usando agentes inteligentes que son capaces de estructurar correctamente oraciones y efectuar acciones como mover alguna parte del cuerpo del robot.

## Dependencias
`pip install langchain langgraph langsmith groq langchain-groq vosk piper-tts pyaudio`

### Modelos TTS y STT
1. Descargar los modelos TTS y STT desde el Google Drive
2. En la raiz de yaren_chat crear una carpeta llamada models
3. Dentro de models crear dos carpetas: TTS y STT
4. Colocar los modelos en sus carpetas correspondientes

### API KEY GROQ
1. Crear una API Key en GROQ. https://console.groq.com/keys
2. Colocar la API Key en la bashrc
- gedit ~/.bashrc
- export GROQ_API_KEY="TU_API_KEY"
- source ~/.bashrc


## Instalación paso a paso

### 1. Crear el workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 2. Clonar el repositorio

```bash
cd src
git clone https://github.com/RAMEL-ESPOL/coco_chat.git
cd ..
```

### 3. Compilar el proyecto

```bash
colcon build
source install/setup.bash
```

## Ejecutar
**Simulación**

```bash
ros2 launch yaren_gazebo_sim yaren_gazebo.launch.py
```

**Yaren Chat con Agentes para mover extremidades**

```bash
ros2 launch yaren_chat yaren_chat.launch.py
```

**Yaren Chat sin Agentes inteligentes**

```bash
ros2 launch yaren_chat yaren_chat_sin_agentes.launch.py
```
