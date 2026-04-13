# Coco Face Screen

Crear animaciones faciales cada que se esta reproduciendo algo por la salida del microfono. Esta funcionalidad brindara la apariencia como el robot estuviese hablando de forma natural.

- No se tiene que inicializar por ningun motivo porque internamente Coco llama a este script para que este escuchando en todo momento si el parlante esta funcionando para poder animar el rostro como si estuvise hablando.
- En todo momento se brindara una animacion para fingir como si el robot pestañea. Esto es independiente si se reproduce algo por el parlante.
- Toda la animacion se la realiza en C++, por lo que esta optimizado para su rendimiento teniendo en cuenta la eficiencia computacional.

## Instalación paso a paso

### 1. Crear el workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 2. Clonar el repositorio

```bash
cd src
git clone https://github.com/RAMEL-ESPOL/coco_face_screen.git
cd ..
```


### 3. Compilar el proyecto

```bash
colcon build
source install/setup.bash
```
