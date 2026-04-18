# Coco Filters

Funcionalidad para colocar filtros al rostro de las personas, existen dos clases de filtros. Un filtro mas general para editar diferentes cuestiones interesantes como nariz, boca, ojos. Mientras que, tambien existe otro tipo de filtro para colocar mascara al rostro de la persona.

## Dependencias
`pip install mediapipe`

## Instalación paso a paso

### 1. Crear el workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 2. Clonar el repositorio

```bash
cd src
git clone https://github.com/RAMEL-ESPOL/coco_filters.git
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

**Coco Filters - Detector de landmarks faciales**

```bash
ros2 run coco_filters face_landmark_detector.py
```

**Coco Filters (2 opciones)**

```bash
ros2 run coco_filters animal_filter_mask
```

```bash
ros2 run coco_filters face_filter_node
```
