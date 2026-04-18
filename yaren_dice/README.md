# Coco Games

Repositorio que contiene todos los juegos que podra ejecutar el robot, incluye Coco Dice que es muy similar al famoso juego Simon Dice. El niño tendrá que realizar las posiciones que diga el robot, el robot va a ir asignando puntajes como recompensa para el niño.

## Dependencias
`pip install ultralytics piper-tts playsound`

## Instalación paso a paso

### 1. Crear el workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 2. Clonar el repositorio

```bash
cd src
git clone https://github.com/RAMEL-ESPOL/coco_games.git
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

**Coco Dice**

```bash
ros2 launch coco_dice coco_say.launch.py
```
