# Coco Emotions

Detectar las emociones de la persona en tiempo real usando un modelo IA, detecta 7 emociones: enojado, disguto, miedo, felicidad, tristeza, sorpresa, neutro. Existen dos modelos, el que contiene la extension .h5 es el mas pesado porque es producto directo desde el entrenamiento, mientras que el modelo .tflite es una optimizacion del modelo en .h5 usando Tensorflow Lite.

## Dependencias
`pip install tensorflow`

### Modelos IA
1. Descargar el modelo llamado model_mbn_1.h5 desde Google Drive y colocarlo en la carpeta models
2. Descargar el modelo llamado model_mbn_1.tflite desde Google Drive y colocarlo en la carpeta models

## Instalación paso a paso

### 1. Crear el workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 2. Clonar el repositorio

```bash
cd src
git clone https://github.com/RAMEL-ESPOL/coco_emotions.git
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
ros2 launch coco_gazebo_sim coco_robot.launch.py
```

**Coco Emotion (2 opciones)**

```bash
ros2 run coco_emotions detect_emotion.py # (modelo mas pesado)
```

```bash
ros2 run coco_emotions detect_emotion_tflite.py # (modelo mas liviano)
```
