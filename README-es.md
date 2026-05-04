# YAREN2 - Plataforma de Robot Social

YAREN2 es una plataforma de robot social inteligente construida sobre ROS 2 Humble, diseñada para la interacción humano-robot natural mediante comandos de voz, LLMs (Groq) y movimientos expresivos.

---

## Requisitos Previos

- **SO:** Ubuntu 22.04 LTS
- **Distribución ROS 2:** ROS 2 Humble Hawksbill
- **Hardware:** PC con GPU NVIDIA (recomendado) o NVIDIA Jetson Orin Nano
- **Python:** 3.10+

---

## Instalación

### 1. ROS 2 Humble

Si no tienes ROS 2 Humble instalado, sigue la [guía oficial de instalación](https://docs.ros.org/en/humble/Installation.html).

Verifica tu instalación:
```bash
printenv ROS_DISTRO  # Debe mostrar: humble
```

### 2. Clonar el Repositorio

```bash
mkdir -p ~/robotis_ws/src
cd ~/robotis_ws/src
git clone https://github.com/RAMEL-ESPOL/YAREN2.git
```

### 3. Dependencias del Sistema

Ahora que tienes el repo, instala todos los paquetes del sistema desde el archivo incluido:

```bash
cd ~/robotis_ws/src/YAREN2
sudo apt update
sudo apt install -y $(grep -v '^#' system_dependencies.txt | tr '\n' ' ')
```

> Esto instala paquetes de ROS 2, librerías de audio, herramientas USB/serial, ffmpeg y herramientas de compilación.
> Lista completa disponible en `system_dependencies.txt`.

### 4. Crear el Entorno Virtual

> ⚠️ `--system-site-packages` es obligatorio para que el entorno virtual pueda acceder a los paquetes de ROS 2 instalados en el sistema.

```bash
cd ~/robotis_ws
python3 -m venv venv_yaren --system-site-packages
source venv_yaren/bin/activate
pip install --upgrade pip
```

### 5. Instalar Dependencias Python

```bash
pip install -r src/YAREN2/venv_requirements.txt
```

### 6. Modelos de IA (STT y TTS)

Descarga los modelos de reconocimiento de voz y síntesis de texto manualmente y colócalos aquí:

```
src/YAREN2/yaren_chat/models/
├── STT/
│   └── vosk-model-es-0.42/       ← Descomprime el contenido aquí (debe contener final.mdl)
└── TTS/
    ├── es_ES-sharvard-medium.onnx
    └── es_ES-sharvard-medium.onnx.json
```

- **STT:** Descarga [vosk-model-small-es-0.42](https://alphacephei.com/vosk/models) y extráelo en `models/STT/`
- **TTS:** Descarga un modelo Piper en español desde [Piper Releases](https://github.com/rhasspy/piper/releases) y coloca los archivos `.onnx` y `.json` en `models/TTS/`

### 7. API Key de Groq

Obtén tu API key gratuita en [Groq Console](https://console.groq.com) y agrégala a tu `.bashrc`:

```bash
echo 'export GROQ_API_KEY="gsk_TU_API_KEY_AQUI"' >> ~/.bashrc
source ~/.bashrc
```

### 8. Compilar el Workspace

```bash
source /opt/ros/humble/setup.bash
source ~/robotis_ws/venv_yaren/bin/activate
cd ~/robotis_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Inicio Rápido (Alias)

Agrega este alias a tu `~/.bashrc` para sourcer todo en un solo comando:

```bash
nano ~/.bashrc
```

Agrega al final:
```bash
alias iniciar_yaren='source /opt/ros/humble/setup.bash && cd ~/robotis_ws && source venv_yaren/bin/activate && source install/setup.bash'
```

Recarga y úsalo:
```bash
source ~/.bashrc
iniciar_yaren
```

---

## Uso

> Siempre ejecuta `iniciar_yaren` antes de cualquier comando de lanzamiento.

### Encender los Motores

Inicializa la comunicación con los motores vía U2D2:
```bash
ros2 launch yaren_u2d2 yaren_robot.launch.py
```

### Filtros de Cámara

```bash
# Filtros de accesorios
ros2 launch yaren_filters yaren_accesorios.launch.py

# Filtros de animales
ros2 launch yaren_filters yaren_animales.launch.py
```

### Detección de Emociones

```bash
ros2 launch yaren_emotions yaren_emotions.launch.py
```

### Juego de Dados

```bash
ros2 launch yaren_dice yaren_dice.launch.py
```

### ChatBot

```bash
# Con agentes inteligentes (recomendado)
ros2 launch yaren_chat yaren_chat.launch.py

# Sin agentes (modo directo y simple)
ros2 launch yaren_chat yaren_chat_sin_agentes.launch.py
```

### Brazo Mimético

> ⚠️ Aún no funciona — en desarrollo.

---

## Estructura del Proyecto

```
robotis_ws/
├── src/
│   └── YAREN2/
│       ├── ia_face_software/
│       ├── usb_cam/
│       ├── yaren_arm_mimic/
│       ├── yaren_chat/
│       ├── yaren_description/
│       ├── yaren_dice/
│       ├── yaren_emotions/
│       ├── yaren_face_display/
│       ├── yaren_filters/
│       ├── yaren_gazebo_sim/
│       ├── yaren_interfaces/
│       ├── yaren_moveit2_config/
│       ├── yaren_movements/
│       ├── yaren_u2d2/
│       ├── system_dependencies.txt   ← paquetes apt
│       ├── venv_requirements.txt     ← paquetes pip
│       └── requirements.txt          ← lista completa de referencia
├── venv_yaren/     ← entorno virtual (no se sube a git)
├── build/          ← generado por colcon (no se sube a git)
├── install/        ← generado por colcon (no se sube a git)
└── log/            ← generado por colcon (no se sube a git)
```

---

## .gitignore

Asegúrate de tener esto en la raíz de `YAREN2/`:

```
venv_yaren/
build/
install/
log/
__pycache__/
*.pyc
*.egg-info/
.env
```

# Busca todos los archivos .py en tu src y dales permiso de ejecución
find ~/robotis_ws/src/YAREN2 -name "*.py" -exec chmod +x {} +

# Busca todos los modelos 3d en tu src y dales permiso de ejecución al gazebo

export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:~/robotis_ws/src/YAREN2
---
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  'video/x-raw,format=RG10,width=1920,height=1080,framerate=30/1' ! \
  nvvidconv ! \
  'video/x-raw,format=BGRx' ! \
  videoconvert ! fakesink -v
RAMEL2026
RAMEL2026