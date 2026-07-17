# Documentación Técnica: Paquete `yaren_dice` y Cámara CSI
**Paquetes ROS2:** `yaren_dice`, `camara_usb_csi`  
**Plataforma:** Jetson Orin Nano · ROS2 Humble · Ubuntu 22.04  
**Dependencias principales:** YOLOv8 (Ultralytics), Piper TTS, OpenCV, GStreamer, NumPy

---

## Índice

1. [Visión general](#1-visión-general)
2. [Arquitectura del sistema](#2-arquitectura-del-sistema)
3. [Cámara CSI — `csi_cam_pub.py`](#3-cámara-csi--csi_cam_pubpy)
4. [Detección corporal — `body_landmarks.py`](#4-detección-corporal--body_landmarkspy)
5. [Detector de poses — `pose_detector.cpp`](#5-detector-de-poses--pose_detectorcpp)
6. [Gestor del juego — `game_manager.cpp`](#6-gestor-del-juego--game_managercpp)
7. [Altavoz — `speaker_node.py`](#7-altavoz--speaker_nodepy)
8. [Tópicos ROS2 del paquete](#8-tópicos-ros2-del-paquete)
9. [Flujo completo de una ronda de juego](#9-flujo-completo-de-una-ronda-de-juego)
10. [Sistema de niveles y desafíos](#10-sistema-de-niveles-y-desafíos)
11. [Notas de concurrencia y diseño](#11-notas-de-concurrencia-y-diseño)

---

## 1. Visión general

`yaren_dice` implementa el juego interactivo **"Yaren Dice"**, una adaptación del clásico "Simón dice" para niños hospitalizados. El juego utiliza la cámara CSI de la Jetson para detectar poses corporales del niño en tiempo real usando YOLOv8 Pose, y proporciona retroalimentación verbal a través de síntesis de voz Piper TTS.

El juego progresa en tres niveles de dificultad:
- **Básico** (puntuación 0–6): desafíos de pose única.
- **Intermedio** (puntuación 7–14): secuencias de hasta 3 poses.
- **Avanzado** (puntuación ≥15): secuencias de hasta 5 poses.

Cada desafío tiene un timeout de 20 segundos y el niño dispone de 3 vidas. La detección requiere que la pose se mantenga durante al menos 0.5 segundos para considerarse válida, evitando detecciones accidentales.

---

## 2. Arquitectura del sistema

```
┌─────────────────────────────────────────────────────────────────────┐
│                          yaren_dice                                 │
│                                                                     │
│  ┌──────────────────┐   /csi_camera/image_raw                      │
│  │  csi_cam_pub.py  │ ────────────────────────────────┐            │
│  │  (Cámara CSI/USB)│                                 ▼            │
│  └──────────────────┘                    ┌─────────────────────┐   │
│                                          │  body_landmarks.py  │   │
│                                          │  (YOLOv8 Pose)      │   │
│                                          └──────────┬──────────┘   │
│                                                     │              │
│                                          /pose_landmarks           │
│                                                     ▼              │
│  ┌───────────────────┐  /current_challenge ┌───────────────────┐  │
│  │   game_manager    │ ──────────────────▶ │  pose_detector    │  │
│  │   (C++)           │ ◀────────────────── │  (C++)            │  │
│  └────────┬──────────┘  /pose_result       └───────────────────┘  │
│           │                                                         │
│           │ /game_feedback                                          │
│           ▼                                                         │
│  ┌───────────────────┐                                             │
│  │  speaker_node.py  │ ──▶ /audio_playing, /yaren/tts_text        │
│  │  (Piper TTS)      │                                             │
│  └───────────────────┘                                             │
└─────────────────────────────────────────────────────────────────────┘

Tópicos externos:
  /yaren/is_english  ← face_screen.cpp (idioma actual)
  /audio_playing     → face_screen.cpp (pausa música de menú)
  /yaren/tts_text    → face_screen.cpp (lip sync)
```

### Nodos del paquete

| Nodo | Archivo | Tipo | Función |
|---|---|---|---|
| `camara` | `csi_cam_pub.py` | Node | Captura y publica frames de cámara |
| `body_points_detector_node` | `body_landmarks.py` | LifecycleNode | Detección de keypoints corporales con YOLOv8 |
| `yaren_pose_detector` | `pose_detector.cpp` | Node | Evalúa poses contra desafíos activos |
| `yaren_game_manager` | `game_manager.cpp` | Node | Orquesta el flujo del juego |
| `yaren_speaker_node` | `speaker_node.py` | LifecycleNode | Síntesis y reproducción de voz |

---

## 3. Cámara CSI — `csi_cam_pub.py`

Nodo publicador de imágenes que soporta tanto cámara CSI IMX219 (vía GStreamer + nvargus) como cámara USB (vía V4L2).

### Pipeline GStreamer (CSI)

```python
def gstreamer_pipeline(sensor_id, capture_width, capture_height,
                        display_width, display_height, framerate, flip_method):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=%d, height=%d, framerate=%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=%d, height=%d, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink"
    )
```

El pipeline usa memoria NVMM (NVIDIA Memory Management) para la decodificación acelerada por hardware, luego convierte a BGR mediante `nvvidconv` y `videoconvert` antes de entregarlo a OpenCV.

### Parámetros declarados

| Parámetro | Default | Descripción |
|---|---|---|
| `sensor_id` | 0 | ID del sensor CSI (0 o 1) |
| `capture_width` | 1920 | Resolución de captura horizontal |
| `capture_height` | 1080 | Resolución de captura vertical |
| `display_width` | 1920 | Resolución de salida horizontal |
| `display_height` | 1080 | Resolución de salida vertical |
| `framerate` | 30 | FPS de captura y publicación |
| `flip_method` | 0 | Rotación del frame (0=sin rotar, 2=180°) |

### Modo activo (USB vs CSI)

```python
# CSI (hardware NVIDIA):
self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

# USB (fallback activo en el código):
self.cap = cv2.VideoCapture(0)
```

El nodo está configurado actualmente para USB (`cv2.VideoCapture(0)`). Para cambiar a CSI se debe descomentar la línea correspondiente y comentar la de USB.

### Publicación

- **Tópico:** `/csi_camera/image_raw` (`sensor_msgs/Image`, encoding `bgr8`)
- **Timer:** `1.0 / framerate` segundos → 30 Hz por defecto
- El header incluye timestamp `get_clock().now()` y `frame_id = 'csi_camera'`.

### Limpieza

`destroy_node()` llama a `self.cap.release()` antes del `super().destroy_node()` para liberar el descriptor del dispositivo correctamente.

---

## 4. Detección corporal — `body_landmarks.py`

Nodo lifecycle que detecta keypoints corporales de personas usando **YOLOv8s-pose** e implementa un patrón productor-consumidor para separar la captura de frames de la inferencia.

### Modelo

- **Archivo:** `yolov8s-pose.pt` en `<pkg_share>/models/`
- **Resolución de inferencia:** 320×320 px (`INFER_SIZE = 320`) — balance entre velocidad y precisión en la Jetson.
- **Keypoints publicados:** 13 puntos (nariz, ojos, orejas, hombros, codos, muñecas, caderas).

### Estados lifecycle

| Transición | Acción |
|---|---|
| `on_configure` | Carga modelo YOLO; crea publisher de `pose_landmarks` |
| `on_activate` | Suscribe `/csi_camera/image_raw`; lanza thread de inferencia |
| `on_deactivate` | `_active = False`; join del thread (timeout 2s); destruye suscripción |
| `on_cleanup` | `self.model = None`; destruye publisher — libera RAM |
| `on_shutdown` | `_active = False` |

El modelo **solo se carga en `on_configure`**, no en el constructor, siguiendo el patrón lifecycle correcto para nodos pesados. Permanece en RAM entre activaciones/desactivaciones, evitando el costo de recarga (~2–5s).

### Patrón productor-consumidor

```
image_callback() [hilo ROS executor]     _infer_loop() [hilo dedicado]
        │                                         │
        │  cada _INFER_EVERY=2 frames:            │
        │  frame → _latest_frame (bajo _lock)     │
        │                                         │  loop while _active:
        │                                         │    with _lock:
        │                                         │      frame = _latest_frame
        │                                         │      _latest_frame = None
        │                                         │    model(frame, imgsz=320)
        │                                         │    publicar Landmarks
```

- **`_INFER_EVERY = 2`:** solo se procesa 1 de cada 2 frames para reducir carga de CPU/GPU.
- **`_lock`:** mutex que protege `_latest_frame` entre el callback de imagen y el thread de inferencia.
- Si no hay frame nuevo, el thread espera `5 ms` antes de reintentar.

### Rotación del frame

```python
frame = cv2.rotate(frame, cv2.ROTATE_180)
```

La cámara CSI está montada invertida en YAREN2; la rotación se aplica en el callback antes de almacenar el frame, no en la inferencia.

### Mensaje publicado

`yaren_interfaces/Landmarks`:
```
Header header
geometry_msgs/Point[] landmarks   # x, y en píxeles; z=0.0
```

Los 13 keypoints se publican en orden fijo: nariz (0), ojo_izq (1), ojo_der (2), oreja_izq (3), oreja_der (4), hombro_izq (5), hombro_der (6), codo_izq (7), codo_der (8), muñeca_izq (9), muñeca_der (10), cadera_izq (11), cadera_der (12).

---

## 5. Detector de poses — `pose_detector.cpp`

Nodo C++ que recibe keypoints corporales y evalúa si el jugador está realizando el desafío activo, publicando el resultado en cada frame.

### Índices de keypoints COCO

```cpp
NOSE=0, LEFT_EYE=1, RIGHT_EYE=2, LEFT_EAR=3, RIGHT_EAR=4,
LEFT_SHOULDER=5, RIGHT_SHOULDER=6,
LEFT_ELBOW=7, RIGHT_ELBOW=8,
LEFT_WRIST=9, RIGHT_WRIST=10,
LEFT_HIP=11, RIGHT_HIP=12
```

### Funciones de comparación geométrica

Todas trabajan con pares `(x, y)` en coordenadas de píxel (origen arriba-izquierda, Y crece hacia abajo):

| Función | Condición | Uso típico |
|---|---|---|
| `is_above(p1, p2, th)` | `p2.y - p1.y > th` | Muñeca sobre hombro |
| `is_below(p1, p2, th)` | `\|p1.y - p2.y\| ≤ th` | Brazo a nivel horizontal |
| `is_at_same_height(p1, p2, th)` | `\|p1.y - p2.y\| < th` | Brazos extendidos |
| `is_in_horizontal_range(p1, p2, th)` | `\|p1.x - p2.x\| < th` | Brazo al frente |
| `is_right_of(p1, p2, th)` | `p2.x - p1.x > th` | Cruce de brazos |
| `is_left_of(p1, p2, th)` | `p1.x - p2.x > th` | Cruce de brazos |
| `is_near(p1, p2, th)` | distancia euclidiana `< th` | Tocar nariz, oreja, codo |

### Umbrales dinámicos

Los umbrales se calculan en cada frame a partir de la geometría de la persona, haciéndolos independientes de la distancia a la cámara:

```cpp
// Basado en distancia entre hombros (~anchura de torso)
int threshold_bt_shoulders = (LEFT_SHOULDER.x - RIGHT_SHOULDER.x) / 3;

// Basado en distancia nariz-ojo (~tamaño de cabeza)
int threshold_vertical   = (NOSE.y - LEFT_EYE.y) * 2;
int threshold_touch_eye  = (LEFT_EYE.x - RIGHT_EYE.x) / 1;
int threshold_touch_elbow = (NOSE.y - LEFT_EYE.y) * 4;
```

### Catálogo de poses detectadas (IDs 1–23)

| ID | Descripción | Keypoints evaluados |
|---|---|---|
| 1 | Brazo derecho arriba | RIGHT_WRIST y RIGHT_ELBOW sobre RIGHT_SHOULDER |
| 2 | Brazo izquierdo arriba | LEFT_WRIST y LEFT_ELBOW sobre LEFT_SHOULDER |
| 3 | Ambos brazos arriba | Combinación de casos 1 y 2 |
| 4 | Brazo derecho extendido horizontal | RIGHT_WRIST a misma altura y distancia horizontal de RIGHT_SHOULDER |
| 5 | Brazo izquierdo extendido horizontal | LEFT_WRIST a misma altura y distancia horizontal de LEFT_SHOULDER |
| 6 | Ambos brazos extendidos | Combinación de casos 4 y 5 |
| 7 | Brazos cruzados arriba | Muñecas sobre hombros pero al lado opuesto |
| 8 | Mano derecha en nariz | RIGHT_WRIST cerca de NOSE |
| 9 | Mano izquierda en nariz | LEFT_WRIST cerca de NOSE |
| 10 | Mano izquierda en ojo izquierdo | LEFT_WRIST cerca de LEFT_EYE |
| 11 | Mano derecha en ojo izquierdo | RIGHT_WRIST cerca de LEFT_EYE |
| 12 | Mano derecha en ojo derecho | RIGHT_WRIST cerca de RIGHT_EYE |
| 13 | Mano izquierda en ojo derecho | LEFT_WRIST cerca de RIGHT_EYE |
| 14 | Mano derecha en oreja derecha | RIGHT_WRIST cerca de RIGHT_EAR |
| 15 | Mano izquierda en oreja derecha | LEFT_WRIST cerca de RIGHT_EAR |
| 16 | Ambas manos en oreja derecha | Combinación de casos 14 y 15 |
| 17 | Mano izquierda en oreja izquierda | LEFT_WRIST cerca de LEFT_EAR |
| 18 | Mano derecha en oreja izquierda | RIGHT_WRIST cerca de LEFT_EAR |
| 19 | Ambas manos en oreja izquierda | Combinación de casos 17 y 18 |
| 20 | Mano derecha en hombro izquierdo | RIGHT_WRIST cerca de LEFT_SHOULDER |
| 21 | Mano izquierda en hombro derecho | LEFT_WRIST cerca de RIGHT_SHOULDER |
| 22 | Mano derecha en codo izquierdo | RIGHT_WRIST cerca de LEFT_ELBOW |
| 23 | Mano izquierda en codo derecho | LEFT_WRIST cerca de RIGHT_ELBOW |

### Publicación de resultado

En cada mensaje de `pose_landmarks` recibido:

```cpp
void detect_poses(Landmarks msg) {
    // Convertir landmarks a vector<pair<float,float>>
    bool result = detect_pose_actions(keypoints);
    
    PoseResult out;
    out.challenge      = current_challenge_;
    out.detected_poses = result;
    out.timestamp      = this->now();
    pose_result_publisher_->publish(out);
}
```

El campo `challenge` en la respuesta permite al `game_manager` verificar que el resultado corresponde al desafío que pidió, descartando resultados rezagados.

---

## 6. Gestor del juego — `game_manager.cpp`

Nodo C++ que orquesta el flujo completo del juego: selección de desafíos, evaluación de resultados, progresión de nivel y gestión de vidas/puntuación.

### Estado del juego

```cpp
int     current_challenge_;        // ID del desafío actual
int     score_;                    // Puntuación acumulada
int     lives_;                    // Vidas restantes (inicia en 3)
bool    audio_playing_;            // TTS activo (bloquea detección)
bool    detection_ongoing_;        // Fase de detección activa
bool    waiting_for_pose_;         // Esperando resultado del detector
double  challenge_timeout_;        // Timestamp UNIX de expiración
double  correct_pose_start_time_;  // Cuándo se detectó la pose correcta
double  correct_pose_duration_;    // Duración mínima de pose (0.5s)
GameLevel current_level_;          // BASIC / INTERMEDIATE / ADVANCED
std::vector<int> current_sequence_; // Secuencia de IDs para niveles avanzados
int     current_sequence_step_;    // Paso actual dentro de la secuencia
int     expected_sequence_length_; // Total de pasos de la secuencia
bool    game_initialized_;         // FIX: evita arrancar antes de recibir idioma
```

### Arranque diferido por idioma

```cpp
// Constructor: game_initialized_ = false → NO arrancar todavía
// handle_language_change():
if (!game_initialized_) {
    game_initialized_ = true;
    select_challenge();
    start_detection();
}
```

El juego **no arranca** hasta recibir el mensaje en `/yaren/is_english`. Esto garantiza que el primer desafío se anuncia en el idioma correcto. El flag `game_initialized_` previene dobles arranques por el QoS `transient_local` que entrega el último mensaje a nuevos suscriptores.

### Protección contra cambio de idioma transitorio

```cpp
auto elapsed = duration_cast<seconds>(now - game_start_time_).count();
if (elapsed < 3.0) {
    // Ignorar — probablemente el mensaje transient_local del arranque
    return;
}
```

Si el cambio llega en los primeros 3 segundos, se ignora. Si llega después:
- Sin progreso (`score_==0 && lives_==3`) → re-seleccionar desafío.
- Con progreso → solo actualizar textos, no resetear el juego.

### Carga de desafíos desde YAML

```cpp
load_challenges_from_yaml()             // config/challenges.yaml
load_intermediate_challenges_from_yaml() // config/intermediate_challenges.yaml
load_advanced_challenges_from_yaml()    // config/advanced_challenges.yaml
```

Cada archivo YAML contiene nodos con:
- `id`: ID numérico de la pose.
- `text` / `text_en`: lista de frases en español/inglés (nivel básico) o frase única (avanzado).
- `poses`: lista de IDs de pose (niveles intermedio/avanzado).
- `sequence_length`: número de poses en la secuencia.

### `select_challenge()`

```
1. Calcular nuevo nivel según score_
2. Si cambió de nivel → announce_level_up() + sleep(3s)
3. Según nivel actual, seleccionar pool de desafíos
4. Elegir desafío aleatorio con rand()
5. Para BÁSICO:
     - current_sequence_ = [challenge_id]
     - Elegir texto aleatorio de la lista
6. Para INTERMEDIO/AVANZADO:
     - current_sequence_ = lista de IDs del YAML
     - current_challenge_ = current_sequence_[0]
7. Publicar current_challenge_ en /current_challenge
8. Publicar texto en /game_feedback (→ TTS lo lee)
```

### `handle_pose_result()` — Evaluación con histéresis temporal

```cpp
if (detected_poses) {
    if (correct_pose_start_time_ == 0.0) {
        correct_pose_start_time_ = get_current_time();  // iniciar timer
    } else if (now - correct_pose_start_time_ >= 0.5) {
        current_sequence_step_++;
        correct_pose_start_time_ = 0.0;
        
        if (current_sequence_step_ >= expected_sequence_length_) {
            handle_successful_challenge();
        } else {
            // Avanzar al siguiente paso de la secuencia
            current_challenge_ = current_sequence_[current_sequence_step_];
            publicar nuevo /current_challenge
            publicar "¡Bien! Ahora la siguiente pose..."
            challenge_timeout_ = now + 20.0
        }
    }
} else {
    correct_pose_start_time_ = 0.0;  // resetear si pierde la pose
}
```

La histéresis de 0.5s evita detecciones accidentales por movimientos de transición. Si el niño pierde la pose antes de los 0.5s, el contador se resetea.

### `handle_successful_challenge()`

```cpp
score_++;
// Texto de victoria aleatorio (6 opciones) + score
feedback_publisher_->publish(victory_text + score_);
sleep(3s);  // esperar a que el TTS lea el mensaje
select_challenge();  // siguiente desafío
```

### `handle_failed_challenge()`

```cpp
lives_--;
if (lives_ <= 0) {
    // Game over: publicar puntuación final + rclcpp::shutdown()
}
// Texto de derrota + vidas restantes con concordancia (intento/intentos)
feedback_publisher_->publish(defeat_text + lives_);
sleep(2s);
select_challenge();
```

La concordancia gramatical se maneja explícitamente:
```cpp
if (lives_ == 1) texto += is_english_ ? " attempt left." : " intento.";
else             texto += is_english_ ? " attempts left." : " intentos.";
```

### Timer de timeout

```cpp
challenge_timer_ = create_wall_timer(500ms, check_challenge_timeout);

void check_challenge_timeout() {
    std::lock_guard<std::mutex> lock(language_mutex_);
    if (!waiting_for_pose_ || challenge_timeout_ == 0.0) return;
    if (get_current_time() > challenge_timeout_)
        handle_failed_challenge(texto_derrota_aleatorio);
}
```

El timer corre cada 500 ms pero solo actúa cuando `waiting_for_pose_` es `true` y el tiempo expiró.

### Sincronización con TTS (`audio_playing_`)

```cpp
// En handle_pose_result():
if (!waiting_for_pose_ || audio_playing_) return;

// En handle_audio_status():
audio_playing_ = msg->data;
if (!audio_playing_ && !detection_ongoing_) start_detection();
```

La detección **no evalúa resultados** mientras el TTS está hablando (`audio_playing_ == true`). La fase de detección se inicia automáticamente cuando el TTS termina de reproducir el enunciado del desafío.

### Niveles de progresión

| Nivel | Condición | Pool de desafíos | Longitud de secuencia |
|---|---|---|---|
| `BASIC` | `score_ < 7` | `challenges.yaml` | 1 pose |
| `INTERMEDIATE` | `7 ≤ score_ < 15` | `intermediate_challenges.yaml` | Hasta 3 poses |
| `ADVANCED` | `score_ ≥ 15` | `advanced_challenges.yaml` | Hasta 5 poses |

---

## 7. Altavoz — `speaker_node.py`

Nodo lifecycle de síntesis de voz para el juego. Es una versión simplificada del TTS de `yaren_chat`, sin el pipeline productor-consumidor ya que los mensajes del juego son frases cortas y completas.

### Diferencias respecto a `tts_lifecycle_node.py` (yaren_chat)

| Aspecto | `speaker_node.py` | `tts_lifecycle_node.py` |
|---|---|---|
| Entrada | `/game_feedback` (`String`) | `/response_person` (`PersonResponse`) |
| Voces | Carga **ambas** (ES + EN) en configure | Carga una sola, cambia en caliente |
| Pipeline | Directo (sin cola) | Productor-consumidor con `queue.Queue` |
| LLM | No — texto ya formado | Sí — espera respuesta del LLM |
| CUDA | `use_cuda=False` | `use_cuda=True` |
| Reproducción | `subprocess.run(['aplay', ...])` | `playsound()` |
| Lip sync | `_compute_lipsync()` con NumPy, ventanas 80 ms | Ventanas 50 ms con `struct.unpack` |

### Estados lifecycle

| Transición | Acción |
|---|---|
| `on_configure` | Carga **ambas** voces (ES y EN); crea publishers; suscribe idioma |
| `on_activate` | `_active = True`; suscribe `/game_feedback` |
| `on_deactivate` | `_active = False`; destruye suscripción de feedback |
| `on_cleanup` | `self.voices = {}`; destruye publishers y suscripción de idioma |
| `on_shutdown` | `_active = False`; `self.voices = {}` |

Cargar ambas voces en `on_configure` (en lugar de una sola con cambio en caliente) es un trade-off: mayor uso de RAM (~600 MB por voz), pero cambio de idioma instantáneo sin latencia de carga.

### `speak_text(text)`

```python
def speak_text(self, text):
    lang  = 'en' if self.is_english else 'es'
    voice = self.voices.get(lang)

    with self.speaking_lock:         # serializar reproducciones
        _publish_audio_status(True)
        try:
            # 1. Sintetizar WAV a archivo temporal
            voice.synthesize_wav(text, wav_file, syn_config)
            
            # 2. Calcular visemas ANTES de reproducir
            lipsync_str = _compute_lipsync(wav_path, sample_rate)
            lipsync_publisher.publish(lipsync_str)
            
            # 3. Reproducir con aplay (bloqueante)
            subprocess.run(['aplay', wav_path], ...)
        finally:
            os.remove(wav_path)
            _publish_audio_status(False)
```

El orden importa: los visemas se publican **antes** de `aplay` para que `face_screen.cpp` reciba la secuencia antes de que empiece el audio.

### `_compute_lipsync(wav_path, sample_rate)`

Versión mejorada respecto al TTS del chat, usa **NumPy** para el cálculo vectorizado:

```python
FRAME_MS   = 80        # ventana más amplia que el chat (50ms)
frame_size = int(sample_rate * 80 / 1000)

for chunk en ventanas_de_80ms:
    rms = sqrt(mean(chunk²))    # RMS normalizado [0.0, 1.0]
    
    # Mapeo RMS → sprite (thresholds en escala normalizada)
    if   rms < 0.01: idx = 0   # silencio
    elif rms < 0.03: idx = 1
    elif rms < 0.06: idx = 2
    elif rms < 0.10: idx = 3
    elif rms < 0.15: idx = 4
    elif rms < 0.20: idx = 5
    elif rms < 0.27: idx = 6
    elif rms < 0.35: idx = 7
    else:            idx = 8   # máxima apertura
```

Diferencias con la versión del chat:
- Usa NumPy (`np.frombuffer`, `np.sqrt(np.mean(chunk**2))`) en lugar de `struct.unpack` + bucle Python.
- Soporta audio estéreo (mezcla a mono antes de calcular RMS).
- Soporta `sampwidth` de 1, 2 y 4 bytes.
- Usa umbrales normalizados [0.0–1.0] en lugar de valores enteros de int16.
- Ventana de 80 ms (vs 50 ms) — más suave para frases del juego.

---

## 8. Tópicos ROS2 del paquete

### Publicados por `yaren_dice`

| Tópico | Tipo | Publicado por | Descripción |
|---|---|---|---|
| `/csi_camera/image_raw` | `sensor_msgs/Image` | `csi_cam_pub` | Frame BGR a 30 Hz |
| `/pose_landmarks` | `yaren_interfaces/Landmarks` | `body_landmarks` | 13 keypoints corporales por frame |
| `/pose_result` | `yaren_interfaces/PoseResult` | `pose_detector` | Resultado de detección de pose |
| `/current_challenge` | `std_msgs/Int16` | `game_manager` | ID del desafío activo |
| `/game_feedback` | `std_msgs/String` | `game_manager` | Texto para TTS (victoria/derrota/desafío) |
| `/audio_playing` | `std_msgs/Bool` | `speaker_node` | TTS activo |
| `/yaren/tts_text` | `std_msgs/String` | `speaker_node` | Secuencia de visemas para lip sync |

### Consumidos por `yaren_dice`

| Tópico | Tipo | Consumido por | Publicado por |
|---|---|---|---|
| `/csi_camera/image_raw` | `sensor_msgs/Image` | `body_landmarks` | `csi_cam_pub` |
| `/pose_landmarks` | `yaren_interfaces/Landmarks` | `pose_detector` | `body_landmarks` |
| `/current_challenge` | `std_msgs/Int16` | `pose_detector` | `game_manager` |
| `/pose_result` | `yaren_interfaces/PoseResult` | `game_manager` | `pose_detector` |
| `/audio_playing` | `std_msgs/Bool` | `game_manager` | `speaker_node` |
| `/game_feedback` | `std_msgs/String` | `speaker_node` | `game_manager` |
| `/yaren/is_english` | `std_msgs/Bool` | `game_manager`, `speaker_node` | `face_screen.cpp` |

### Interfaces personalizadas (`yaren_interfaces`)

**`Landmarks.msg`**
```
Header header
geometry_msgs/Point[] landmarks   # 13 keypoints; z siempre 0.0
```

**`PoseResult.msg`**
```
int16    challenge        # ID del desafío evaluado
bool     detected_poses   # true si el jugador realiza la pose correcta
builtin_interfaces/Time timestamp
```

---

## 9. Flujo completo de una ronda de juego

```
[Arranque]
game_manager: wait 2s → load_challenges × 3 YAML
game_manager: wait /yaren/is_english  (QoS transient_local)
game_manager: select_challenge()
  → elige desafío aleatorio del pool BASIC
  → publica /current_challenge = 1
  → publica /game_feedback = "¡Levanta el brazo derecho!"

[TTS habla]
speaker_node: recibe /game_feedback
  → synthesize_wav("¡Levanta el brazo derecho!")
  → _compute_lipsync() → /yaren/tts_text "80:0,1,3,5,7,5,3,1,0..."
  → /audio_playing: True
  → aplay() (bloqueante ~2s)
  → /audio_playing: False

[Detección activa]
game_manager: /audio_playing=False → start_detection()
  → waiting_for_pose_ = True
  → challenge_timeout_ = now + 20.0

csi_cam_pub: timer_callback() → /csi_camera/image_raw (30 Hz)

body_landmarks: image_callback() [cada 2 frames]
  → _infer_loop(): YOLO(frame, imgsz=320)
  → keypoints.xy → /pose_landmarks [13 puntos]

pose_detector: detect_poses()
  → detect_pose_actions(keypoints)
     case 1: RIGHT_WRIST > RIGHT_SHOULDER && RIGHT_ELBOW > RIGHT_SHOULDER?
  → /pose_result { challenge:1, detected_poses:true/false }

[Niño levanta el brazo]
game_manager: handle_pose_result()
  → detected_poses=true → correct_pose_start_time_ = now
  ... 0.5s después ...
  → current_sequence_step_++ → 1 >= 1 → handle_successful_challenge()
  → score_++ = 1
  → /game_feedback = "¡Muy bien! Tu puntuación es 1."
  → sleep(3s)
  → select_challenge()  [siguiente desafío]

[Si el niño no hace la pose en 20s]
game_manager: check_challenge_timeout()  [timer 500ms]
  → handle_failed_challenge("¡Oh no! ...")
  → lives_-- = 2
  → /game_feedback = "¡Oh no! No te preocupes. Tienes 2 intentos."
  → sleep(2s)
  → select_challenge()

[Game over: lives_ == 0]
game_manager: handle_failed_challenge()
  → /game_feedback = "Ha sido muy divertido... Tu puntuación final es X."
  → rclcpp::shutdown()
```

---

## 10. Sistema de niveles y desafíos

### Progresión automática

```cpp
GameLevel get_current_level() {
    if (score_ >= 15) return GameLevel::ADVANCED;
    if (score_ >= 7)  return GameLevel::INTERMEDIATE;
    return              GameLevel::BASIC;
}
```

Al subir de nivel, `announce_level_up()` publica un mensaje explicativo en `/game_feedback` y espera 3 segundos para que el TTS lo lea antes de seleccionar el primer desafío del nuevo nivel.

### Estructura YAML — Nivel básico (`challenges.yaml`)

```yaml
challenges:
  - id: 1
    text:
      - "¡Levanta el brazo derecho!"
      - "¡Sube tu brazo derecho!"
    text_en:
      - "Raise your right arm!"
      - "Lift up your right arm!"

  - id: 3
    text:
      - "¡Levanta los dos brazos!"
    text_en:
      - "Raise both arms!"
```

Los textos son listas para añadir variedad (se elige uno aleatoriamente con `rand()`), evitando que el juego suene repetitivo.

### Estructura YAML — Niveles intermedio/avanzado

```yaml
intermediate_challenges:
  - poses: [1, 2]
    sequence_length: 2
    text: "¡Primero levanta el brazo derecho, luego el izquierdo!"
    text_en: "First raise your right arm, then your left arm!"

advanced_challenges:
  - poses: [1, 3, 8, 2, 9]
    sequence_length: 5
    text: "¡Sigue esta secuencia de 5 movimientos!"
    text_en: "Follow this sequence of 5 movements!"
```

### Secuencias multi-paso

Para niveles intermedio y avanzado, el flujo de evaluación es:

```
Desafío: poses=[1,3,8], sequence_length=3

Paso 0: current_challenge_=1 → niño levanta brazo derecho
        → /pose_result detected=true por 0.5s
        → current_sequence_step_=1, current_challenge_=3
        → /game_feedback "¡Bien! Ahora la siguiente pose..."

Paso 1: current_challenge_=3 → niño levanta ambos brazos
        → detected=true por 0.5s
        → current_sequence_step_=2, current_challenge_=8

Paso 2: current_challenge_=8 → mano derecha en nariz
        → detected=true por 0.5s
        → current_sequence_step_=3 >= 3 → handle_successful_challenge()
```

Cada paso tiene su propio timeout de 20 segundos (`challenge_timeout_` se resetea en cada avance).

---

## 11. Notas de concurrencia y diseño

### `body_landmarks.py`
- `_lock` serializa el acceso a `_latest_frame` entre el callback de imagen (hilo del executor) y `_infer_loop` (hilo dedicado). Es un `threading.Lock` estándar, no un RLock.
- Al desactivarse, `_active = False` hace que el loop salga en la próxima iteración. El `join(timeout=2.0)` garantiza que el hilo terminó antes de destruir el nodo.
- El modelo YOLO **no tiene lock** porque solo lo accede el hilo de inferencia.

### `game_manager.cpp`
- `language_mutex_` protege tanto `check_challenge_timeout()` como `handle_pose_result()` y `handle_language_change()`, ya que todos modifican el estado del juego.
- `audio_playing_` se lee sin lock en `handle_pose_result()` porque es `bool` (escritura atómica en x86/ARM).
- `std::this_thread::sleep_for()` en `select_challenge()` y los handlers de éxito/fallo es bloqueante en el hilo del executor — aceptable aquí porque el juego tiene un único flujo secuencial y el nodo usa `rclcpp::spin()` con un único thread.

### `speaker_node.py`
- `speaking_lock` (`threading.Lock`) serializa llamadas a `speak_text()`, evitando que dos mensajes de feedback se sinteticen simultáneamente.
- Las llamadas a `speak_text()` se lanzan en threads daemon (`threading.Thread(..., daemon=True)`), por lo que terminan automáticamente si el nodo se destruye.
- `voice_lock` no existe en esta versión: como ambas voces están pre-cargadas y no se cambian en caliente, no es necesario.

### Interacción con `face_screen.cpp`
El nodo `speaker_node` publica en `/audio_playing` (pausa música de menú en `face_screen`) y `/yaren/tts_text` (lip sync). Esto hace que durante el juego la cara de YAREN2 sincronice los labios con las frases del juego exactamente igual que durante el chat conversacional.

### Separación de lanzamiento de `pose_detector`
```cpp
// En el constructor de game_manager:
std::thread([]() {
    std::system("ros2 run yaren_dice pose_detector &");
}).detach();
```
El `game_manager` lanza `pose_detector` como subproceso independiente en lugar de incluirlo en el mismo ejecutable. Esto permite que `pose_detector` se reinicie independientemente si falla, y mantiene la separación de responsabilidades.