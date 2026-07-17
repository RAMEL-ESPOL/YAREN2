# Documentación Técnica: `face_screen.cpp`
**Nodo ROS2:** `face_screen`  
**Paquete:** `yaren_face_display`  
**Plataforma:** Jetson Orin Nano · ROS2 Humble · Ubuntu 22.04  
**Dependencias principales:** OpenCV, SDL2/SDL_mixer, cv_bridge, rclcpp, lifecycle_msgs

---

## Índice

1. [Visión general](#1-visión-general)
2. [Arquitectura de clases](#2-arquitectura-de-clases)
3. [Funciones utilitarias globales](#3-funciones-utilitarias-globales)
4. [Clase `SettingsMenu`](#4-clase-settingsmenu)
5. [Clase `RadioApp`](#5-clase-radioapp)
6. [Clase `RoutinesApp`](#6-clase-routinesapp)
7. [Clase `OnScreenKeyboard`](#7-clase-onscreenkeyboard)
8. [Clase `WifiSetupScreen`](#8-clase-wifisetupscreen)
9. [Nodo principal: `VideoSynchronizer`](#9-nodo-principal-videosynchronizer)
   - [Suscripciones y publicadores](#91-suscripciones-y-publicadores)
   - [Sistema de navegación de menús](#92-sistema-de-navegación-de-menús)
   - [Ciclo de render](#93-ciclo-de-render)
   - [Pantalla de carga (boot)](#94-pantalla-de-carga-boot)
   - [Screensaver de inactividad](#95-screensaver-de-inactividad)
   - [Lip sync](#96-lip-sync)
   - [Gestión de audio](#97-gestión-de-audio)
   - [Lifecycle nodes](#98-lifecycle-nodes)
10. [Tópicos ROS2](#10-tópicos-ros2)
11. [Flujo de inicio completo](#11-flujo-de-inicio-completo)
12. [Notas de concurrencia y fixes aplicados](#12-notas-de-concurrencia-y-fixes-aplicados)

---

## 1. Visión general

`face_screen.cpp` es el nodo central de interfaz visual de YAREN2. Cumple tres roles simultáneos:

- **Pantalla de cara animada:** muestra ojos y boca sincronizados con TTS via lip sync por visemas.
- **Menú táctil interactivo:** navegación por modos del robot (Chat, Emociones, Filtros, Mimic, Radio, Rutinas, etc.) mediante mouse/pantalla táctil.
- **Orquestador de modos:** activa/desactiva nodos lifecycle de ROS2 y lanza subprocesos Python al cambiar de modo.

El nodo publica el frame renderizado como `sensor_msgs/Image` en `/face_screen` a 30 FPS, y gestiona la lógica de audio (música de menú, música de boot, radio infantil) a través de SDL2_mixer.

---

## 2. Arquitectura de clases

```
VideoSynchronizer (rclcpp::Node)
├── SettingsMenu          — configuración de audio, fecha, idioma, WiFi
├── RadioApp              — radio musical infantil con animación facial
├── RoutinesApp           — gestor de rutinas de movimiento personalizadas
├── OnScreenKeyboard      — teclado QWERTY táctil para entrada de texto
└── WifiSetupScreen       — escaneo y conexión a redes WiFi via nmcli
```

Todas las clases renderizan sobre un `cv::Mat` de 800×480 px (resolución de la pantalla de YAREN2) y reciben eventos de mouse forwarded desde el callback único `mouseCallbackStatic`.

---

## 3. Funciones utilitarias globales

### `parsePulseDevices(bool isSink)`
```cpp
static std::vector<AlsaDevice> parsePulseDevices(bool isSink);
```
Ejecuta `pactl list sinks` o `pactl list sources` y parsea la salida para obtener dispositivos de audio PulseAudio disponibles. Filtra automáticamente:
- **Sinks:** incluye HDMI, S/PDIF, Analog, Headphone; excluye virtuales y monitores.
- **Sources:** excluye Nvidia, virtuales y monitores.

**Retorna:** vector de `AlsaDevice { hwId, label }`.

---

### `getPulseDefault(bool isSink)`
```cpp
static std::string getPulseDefault(bool isSink);
```
Obtiene el sink/source por defecto actual via `pactl get-default-sink/source`.

---

### `drawCenteredText(frame, txt, totalW, y, font, scale, color, thick)`
Dibuja texto centrado horizontalmente en el frame en la coordenada `y` indicada.

---

### `drawTextInRect(frame, txt, rect, font, scale, color, thick)`
Dibuja texto centrado dentro de un `cv::Rect`, centrado tanto horizontal como verticalmente.

---

### `drawArrow(frame, center, up, color, size)`
Dibuja un triángulo sólido apuntando hacia arriba o abajo, usado en los botones de scroll de listas.

---

## 4. Clase `SettingsMenu`

Panel de configuración accesible desde el botón de engranaje. Ocupa toda la pantalla cuando está activo.

### Responsabilidades
- Selección de micrófono y parlante via PulseAudio.
- Control de volumen de música con slider draggable y botón mute.
- Ajuste de fecha/hora del sistema (`timedatectl`).
- Toggle de idioma Español/Inglés.
- Botón de acceso a WiFi.
- Botón GUARDAR TODO / REFRESCAR / VOLVER.

### Campos clave

| Campo | Tipo | Descripción |
|---|---|---|
| `volumeLevel` | `int` | Nivel de volumen SDL [0–128] |
| `isMuted` | `bool` | Estado mute |
| `selectedMicId` / `selectedSpkId` | `string` | IDs PulseAudio activos |
| `isEnglish` | `bool*` | Puntero al flag de idioma del nodo principal |
| `onBack` / `onWifi` / `onMicSelected` / `onSpkSelected` / `onLanguageChanged` | `std::function` | Callbacks al nodo principal |
| `audioMutex_` | `std::mutex*` | Mutex compartido para operaciones SDL_mixer |

### Métodos principales

#### `refresh()`
Repopula las listas de micrófonos y parlantes llamando a `parsePulseDevices()` y `getPulseDefault()`. También reinicia los valores de fecha/hora al momento actual. Debe llamarse al abrir el panel.

#### `render(cv::Mat& frame)`
Renderiza el panel completo sobre el frame. Internamente llama a:
- `renderAudioPanel()` — dos veces: una para micrófono, otra para parlante.
- `renderDatePanel()` — spinners de día y mes.
- `renderVolumePanel()` — slider de volumen con botón mute.
- `renderBottomBar()` — botones de acción y mensaje de feedback.

#### `handleMouse(int event, int x, int y)`
Dispatcher de eventos de mouse. Gestiona:
- Drag del slider de volumen (`isDraggingVolume`).
- Clicks en botones mute, scroll de listas, spinners de fecha, botones de acción.
- Llama a `Mix_VolumeMusic()` protegido por `audioMutex_`.

#### `saveAudioConfig()`
Aplica la configuración de audio seleccionada:
```bash
pactl set-default-sink '<id>'
pactl set-default-source '<id>'
# Mueve streams activos al nuevo dispositivo
```
Escribe los IDs seleccionados en `/tmp/yaren_mic_device.txt` y `/tmp/yaren_spk_device.txt` para que otros nodos los lean.

#### `applyDateTime()`
Aplica fecha y hora con:
```bash
sudo timedatectl set-time 'YYYY-MM-DD HH:MM:00'
```
Requiere que `timedatectl` esté configurado con `NOPASSWD` en sudoers.

#### `setAudioMutex(std::mutex* m)`
Inyecta el mutex de audio compartido con `RadioApp` y el nodo principal, evitando condiciones de carrera en SDL_mixer.

---

## 5. Clase `RadioApp`

Radio musical infantil con animación facial sincronizada al ritmo de la canción.

### Estados
```cpp
enum class RadioState { SELECTING, PLAYING };
```
- `SELECTING`: cuadrícula de tarjetas de canciones paginada (2 columnas, 2 filas por página).
- `PLAYING`: pantalla "Now Playing" con cara animada, ecualizador y anillos de pulso.

### Estructura de canción
```cpp
struct SongInfo {
    std::string id, title, subtitle, audioFile;
    cv::Scalar  color;   // color temático de la canción
    double      beatHz;  // frecuencia de beat para animaciones
};
```

### Canciones precargadas
| ID | Título | Artista |
|---|---|---|
| `aramsamsam` | ARA RAM SAM SAM | Luli Pampín |
| `gorila` | BAILE DE GORILA | CantaJuego |
| `barney` | BARNEY ES UN DINOSAURIO | Barney |
| `chipichapa` | CHOPI CHOPI | Christell |
| `libresoy` | LIBRE SOY | Martina Stoessel |
| `sasa` | SA SA | Luli Pampín |
| `sitienesganas` | SI TIENES GANAS | Luli Pampín |

Los archivos `.mp3` se buscan en `<pkg_share>/audios/`.

### Métodos principales

#### `init(pkgDir, logger)`
Inicializa SDL2_mixer (`Mix_OpenAudio` a 44100 Hz, estéreo, buffer 2048), carga imágenes de cara (`eyes_pairs/7.png`, `mouths/13.png`, etc.) y caras alternativas desde `faces/`.

#### `playSong(int idx)`
1. Llama a `killAudio()` para detener lo que suene.
2. Carga el `.mp3` con `Mix_LoadMUS()`.
3. Lanza `Mix_FadeInMusic()` con fade de 500 ms.
4. Lanza `yaren_dance_radio.py` en background para mover el cuerpo del robot.
5. Fallback: si SDL falla, usa `mpg123 -q` en un thread.

#### `killAudio()` / `killAudioImpl()`
Para la música SDL, libera `currentMusic`, y envía un comando ROS2 para devolver brazos a posición de reposo:
```bash
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory ...
```

#### `renderNowPlaying(frame)`
Compone la pantalla de reproducción:
- `drawPulseRings()` — 4 anillos concéntricos pulsantes al `beatHz`.
- `drawAnimatedFace()` — cara con parpadeo, bounce y rotación al ritmo.
- `drawEqualizer()` — 48 barras de ecualizador animadas.
- `drawSongInfoBar()` — cabecera con título y punto de pulso.

#### `drawAnimatedFace()`
Alterna entre la cara compuesta (ojos + boca, con parpadeo aleatorio) y hasta 4 caras alternativas (`money.png`, `open_mouth.png`, etc.) cada `faceSwitchInterval` segundos. Aplica:
- Bounce vertical/horizontal proporcional a `beatHz`.
- Rotación sinusoidal leve.
- Aura de color de la canción.

---

## 6. Clase `RoutinesApp`

Gestor de rutinas de movimiento personalizadas almacenadas como scripts Python en:
```
<workspace>/src/YAREN2/yaren_movements/yaren_movements/*.py
```

### Funcionalidad
- Lista automáticamente los `.py` del directorio excluyendo scripts del sistema (`yaren_dance_radio.py`, `yaren_movement.py`, etc.).
- Muestra tarjetas paginadas (2×2 por página).
- Al seleccionar una tarjeta ejecuta el script con `python3` en un `std::thread` detached y muestra la pantalla "REPRODUCIENDO RUTINA...".
- Botón **NUEVA RUTINA** lanza `yaren_rutinanueva.py` (grabador de poses).

### Métodos principales

#### `refresh()`
Escanea el directorio de movimientos y popula `routines` con los nombres de archivo encontrados, ordenados en reversa.

#### `launchRoutine(scriptPath, routineName)`
Construye la ruta absoluta y ejecuta `python3 <abs_path>/<routineName>` en un hilo detached. Cuando el proceso termina (con o sin error), vuelve al estado `SELECTING` y llama a `refresh()`.

---

## 7. Clase `OnScreenKeyboard`

Teclado QWERTY táctil renderizado sobre el frame, usado para ingresar contraseñas WiFi.

### Características
- Dos layouts: **alfabético** (QWERTY) y **numérico/símbolos** (`123`), alternables con botón toggle.
- Tecla **SHIFT** para mayúsculas (modo alpha).
- Campo de texto con cursor parpadeante; opción de mostrar asteriscos (`isPassword_`).
- Botones **CONECTAR** (confirma) y **CANCELAR** (descarta).

### Métodos

#### `show(promptText, initial)`
Activa el teclado con el texto prompt y valor inicial del campo.

#### `handleMouse(int ev, int x, int y) → bool`
Retorna `true` únicamente cuando el usuario presiona **CONECTAR** y el campo no está vacío. El caller (`WifiSetupScreen`) usa este retorno para iniciar la conexión.

#### `render(cv::Mat& frame)`
Dibuja el panel semitransparente (760×310 px) con todas las teclas, resaltando la tecla bajo el cursor.

---

## 8. Clase `WifiSetupScreen`

Pantalla de configuración WiFi con escaneo de redes y conexión via `nmcli`.

### Flujo de conexión

```
refresh()
  → nmcli connection show        # redes guardadas
  → nmcli dev wifi list          # redes visibles
  → combina ambas listas

handleMouse → btnConnect clicked
  → red guardada   → connectSaved()    (nmcli connection up)
  → red abierta    → connectOpen()     (nmcli device wifi connect)
  → red nueva/WPA  → keyboard_.show() → connectWithPassword()
```

### `connectWithPassword(ssid, password)`
Ejecuta en un thread:
```bash
nmcli device wifi connect '<ssid>' password '<pwd>' > /tmp/yaren_wifi_err.txt 2>&1
```
Lee el archivo de error para clasificar el fallo (contraseña incorrecta, red no encontrada, etc.) y muestra el mensaje en `statusMsg_` durante 4 segundos.

### `renderMain(frame)`
Lista hasta 7 redes visibles con indicadores:
- `●` verde — red guardada.
- `○` gris — red nueva.
- Icono candado — red WPA/WPA2.
- Badge "saved"/"guardada" o "new"/"nueva".

---

## 9. Nodo principal: `VideoSynchronizer`

Hereda de `rclcpp::Node` con nombre `"face_screen"`. Es el orquestador central.

### 9.1 Suscripciones y publicadores

| Tópico | Tipo | Dirección | Descripción |
|---|---|---|---|
| `/audio_playing` | `std_msgs/Bool` | Sub | TTS activo → activa lip sync y pausa música de menú |
| `/yaren_mode` | `std_msgs/String` | Sub+Pub | Cambio de modo (pub: órdenes; sub: modos externos) |
| `/stt_listening` | `std_msgs/Bool` | Sub | STT escuchando → muestra overlay "Yaren te escucha" |
| `/yaren/tts_text` | `std_msgs/String` | Sub | Secuencia de visemas para lip sync |
| `/yaren/wake_event` | `std_msgs/Bool` | Sub | Wake word detectado → resetea timer de inactividad |
| `/face_screen` | `sensor_msgs/Image` | Pub | Frame renderizado a 30 FPS |
| `/yaren/is_english` | `std_msgs/Bool` | Pub | Idioma activo (QoS transient_local) |
| `/yaren/face_idle` | `std_msgs/Bool` | Pub | Estado idle (QoS transient_local) |
| `/yaren/mic_owner` | `std_msgs/String` | Pub | Propietario del micrófono ("mic_test", "none") |
| `/yaren/mic_test` | `std_msgs/Bool` | Pub | Señal de inicio/fin de test de micrófono |

### 9.2 Sistema de navegación de menús

La navegación usa una **pila de niveles** (`navStack: vector<NavLevel>`):

```cpp
struct NavLevel {
    std::string           title;
    cv::Scalar            accentColor;
    std::vector<MenuItem> items;
    std::string           key;        // clave en subMenuMap
};
```

```cpp
struct MenuItem {
    std::string id, label, sublabel;
    cv::Scalar  color;
    cv::Rect    rect;                          // calculado en render
    std::string cmd;                           // comando bash a ejecutar
    std::string stopCmd;                       // comando de detención
    bool        hasSubMenu;
    std::string subMenuKey;                    // clave en subMenuMap
    std::string iconKey;                       // clave en iconMap
    vector<string> lifecycle_nodes;            // nodos lifecycle a activar
};
```

#### Árbol de menús

```
MENÚ PRINCIPAL
├── MODO PRUEBA (sub_modo_prueba)
│   ├── CÁMARA             → simple_camera.py
│   ├── MICRÓFONO          → executeMicTest()
│   └── MOTORES (sub_motores)
│       ├── POS. ORIGINAL  → ros2 topic pub ...
│       ├── BRAZO IZQ (sub_brazo_izq)  →  arriba / medio / bajo
│       ├── BRAZO DER (sub_brazo_der)  →  arriba / medio / bajo
│       ├── BASE (sub_base)             →  izq / der / orig
│       └── CABEZA (sub_cabeza)         →  izq / der / orig
└── YAREN (sub_yaren)  ←→  (sub_yaren_p2)
    ├── MIMIC              → yaren_mimic.launch.py
    ├── CHAT               → lifecycle: llm + stt + tts
    ├── YAREN DICE         → game_manager + lifecycle: csi + body + speaker
    ├── MOVIMIENTOS (sub_yaren_movements)
    │   ├── RUTINA 1       → yaren_rutina1.py
    │   ├── RUTINA 2       → yaren_fullmovement.py
    │   └── RUTINAS PERSONALES → RoutinesApp (INTERNAL_ROUTINES)
    ├── EMOCIONES          → lifecycle: csi + detector
    └── FILTROS (sub_yaren_filtros)
        ├── ANIMALES       → lifecycle: csi + landmarks + filtro_animales
        ├── ACCESORIOS     → lifecycle: csi + landmarks + face_filter
        └── FONDOS VIRTUAL → lifecycle: csi + virtual_background

    [página 2] YAREN RADIO (sub_yaren_radio)
    ├── MÚSICA             → RadioApp (INTERNAL_RADIO)
    └── VIDEOS (sub_yaren_videos)
        ├── POLLITO PÍO    → pollitopio.py
        ├── GALLINA TURULECA → gallinaturuleca.py
        ├── VACA LOLA      → vacalola.py
        └── SUSANITA       → susanita.py
```

La página 2 de YAREN se navega con flechas laterales (`renderSideNavArrow`). El key de la capa indica la posición: `sub_yaren` muestra flecha derecha "1/2", `sub_yaren_p2` muestra flecha izquierda "2/2".

#### `executeMode(MenuItem item, bool publish_mode)`
Lógica al seleccionar un ítem de acción:
1. Si `cmd == "INTERNAL_RADIO"` → activa `RadioApp`.
2. Si `cmd == "INTERNAL_ROUTINES"` → activa `RoutinesApp`.
3. Si `id == "test_mic"` → ejecuta `executeMicTest()`.
4. Para modos normales:
   - Desactiva lifecycle nodes del modo anterior.
   - Activa lifecycle nodes del nuevo modo.
   - Lanza el comando en `std::thread` detached.
   - Publica el nuevo modo en `/yaren_mode`.

### 9.3 Ciclo de render

`renderLoop()` corre en un thread separado a 30 FPS:

```
while (running) {
    if (configuring)       → renderLoadingScreen()
    else if (showWifiSetup_) → cara + wifiSetup_.render()
    else if (showSettings_)  → settingsMenu.render()
    else if (showRadio_)     → radioApp.render()
    else if (showRoutines_)  → routinesApp.render()
    else {
        getFaceFrame()           // cara animada
        renderFaceOverlay()      // overlays de error/mic
        renderMenu()             // si navStack no está vacío
        renderSettingsButton()   // engranaje esquina superior derecha
        renderPowerButton()      // power esquina superior izquierda
    }
    // screensaver si idle > 60s
    // publicar en /face_screen
}
```

`drawWindow()` corre en el hilo principal (spin loop de main) y hace `cv::imshow()` + `cv::waitKey(1)`.

#### `getFaceFrame()`
Construye el frame de cara:
1. Selecciona imagen de ojos (`eyesOpenImg` o `eyesClosedImg`) según el estado de parpadeo.
2. Parpadeo automático cada ~4 segundos, duración 200 ms.
3. Selecciona boca: si TTS activo, blend entre `mouthSprites_[currentVisemeIdx_]` y `mouthSprites_[nextVisemeIdx_]` con `visemeBlend_`. Si no, `mouthClosedImg`.
4. Llama a `overlayImage()` / `overlayImageAlpha()` para compositar capas RGBA.

### 9.4 Pantalla de carga (boot)

`renderLoadingScreen(frame)` durante `configuring == true`:

Muestra una animación de "Robot Awakening" con:
- **Fondo radial** cálido (gradiente rojo-naranja desde el centro).
- **Ondas de sonar** pulsantes.
- **Imagen de Yaren** (`icons/yaren.png`) que se revela de abajo hacia arriba sincronizada con el progreso (`pct = configProgress / 11`). Efecto de silueta azul/ámbar que transiciona a color real al 85%+.
- **Título "YAREN"** con efecto máquina de escribir letra a letra.
- **Barra de progreso** naranja con partículas en el frente.
- **Texto de estado** estilo terminal con cursor parpadeante.

El hilo de boot hace en secuencia:
```
1. sleep(3s)
2. checkChatAvailable()          → si falla, muestra WifiSetupScreen
3. configure_node() × 11        → ros2 lifecycle set configure
4. configStatus = "Yaren listo!"
5. Para música de boot
6. configuring = false
```

Los 11 nodos configurados son:
`llm_lifecycle_node`, `stt_lifecycle_node`, `tts_lifecycle_node`, `detector`, `face_filter_node`, `body_points_detector_node`, `yaren_speaker_node`, `virtual_background_node`, `filtro_animales`, `face_landmark_publisher`, `csi_cam_node`.

### 9.5 Screensaver de inactividad

Tras 60 segundos sin interacción (`idleTimeoutSecs = 60.0`) y con la cara en estado puro (sin menú, sin modo activo, sin overlays), se activa el screensaver:

```cpp
startIdleScreen():
  → stopMenuMusic()
  → idleVideo.open(randomPath)    // uno de 4 videos: carro, gatito, jake, bart
  → Mix_PlayMusic(idleMusic, -1)  // una de 7 canciones relajantes
```

El video se reproduce en loop (`cv::VideoCapture` con reset de posición al llegar al final). Cualquier clic o wake word detectado llama a `resetIdleTimer()` → `stopIdleScreen()`.

### 9.6 Lip sync

El sistema de lip sync recibe secuencias de visemas desde `/yaren/tts_text` en dos formatos:

**Formato nuevo** (preferido):
```
"dur_ms:viseme_idx|dur_ms:viseme_idx|..."
Ejemplo: "80:7|55:4|100:3|80:0"
```

**Formato legacy:**
```
"dur_ms_base:idx,idx,idx,..."
```

Los visemas [0–8] corresponden a sprites de boca `mouths/0.png` … `mouths/8.png`. En `updateViseme()` se hace **blend suave** entre el visema actual y el siguiente durante 70 ms (`visemeBlend_` de 0.0 a 1.0) para suavizar las transiciones.

La tabla de mapeo fonema→sprite usada por el módulo TTS es:
```
0: silencio/pausa   1: consonantes oclusivas   2: U
3: O                4: I/S/Z                   5: E/F
6: X/fricativas     7: A                       8: abierta
```

### 9.7 Gestión de audio

Todas las operaciones SDL_mixer están protegidas por `audioMutex_` (mutex compartido entre `VideoSynchronizer`, `SettingsMenu` y `RadioApp`).

| Música | Fuente | Loop | Cuándo |
|---|---|---|---|
| Boot music | `bringmetolife.mp3` / `cancionmundial2010.mp3` / `justthewayuare.mp3` | ∞ | Durante la carga del sistema |
| Música de menú | `Intro1/2/3.mp3` | ∞ | Menú principal activo |
| Radio infantil | `.mp3` de canciones | ∞ | `RadioApp` en modo PLAYING |
| Música idle | 7 canciones relajantes | ∞ | Screensaver activo |

La música de menú se pausa automáticamente mientras el TTS está hablando (`Mix_PauseMusic` / `Mix_ResumeMusic` en `audioPlayingCallback`).

### 9.8 Lifecycle nodes

La activación/desactivación de nodos lifecycle se hace via servicios ROS2:

```cpp
void change_lifecycle_state(const std::string& node_name, uint8_t transition_id) {
    // Crea cliente /node_name/change_state si no existe
    // Espera disponibilidad 1000 ms
    // Envía async_send_request con transition_id
}
```

Transiciones usadas:
- `TRANSITION_CONFIGURE` (1) — al boot.
- `TRANSITION_ACTIVATE` (3) — al entrar en un modo.
- `TRANSITION_DEACTIVATE` (4) — al salir de un modo.

Los clientes se cachean en `lifecycle_clients_` para evitar recrearlos en cada transición.

---

## 10. Tópicos ROS2

### Publicados por `face_screen`

| Tópico | Tipo | QoS | Descripción |
|---|---|---|---|
| `/face_screen` | `sensor_msgs/Image` | default | Frame BGR 800×480 a 30 FPS |
| `/yaren_mode` | `std_msgs/String` | default | Modo activo o "idle" |
| `/yaren/is_english` | `std_msgs/Bool` | transient_local | Idioma actual |
| `/yaren/face_idle` | `std_msgs/Bool` | transient_local | `true` si cara en reposo |
| `/yaren/mic_owner` | `std_msgs/String` | transient_local | Propietario del mic |
| `/yaren/mic_test` | `std_msgs/Bool` | default | Señal test de micrófono |

### Consumidos por `face_screen`

| Tópico | Tipo | Publicado por |
|---|---|---|
| `/audio_playing` | `std_msgs/Bool` | `tts_lifecycle_node` |
| `/yaren_mode` | `std_msgs/String` | `yaren_voice_menu.py` / externo |
| `/stt_listening` | `std_msgs/Bool` | `stt_lifecycle_node` |
| `/yaren/tts_text` | `std_msgs/String` | `tts_lifecycle_node` |
| `/yaren/wake_event` | `std_msgs/Bool` | `wake_word_node.py` |

---

## 11. Flujo de inicio completo

```
main()
  → rclcpp::init()
  → VideoSynchronizer()  (constructor)
      → Carga imágenes de cara e iconos
      → Inicializa SDL2_mixer
      → Lanza música de boot (aleatoria)
      → Kill de procesos previos (wake_word, voice_menu, etc.)
      → Lanza subprocesos Python en background:
          wake_word_node, yaren_voice_menu, gestor_idioma,
          llm/stt/tts lifecycle nodes, detect_emotion,
          face_filter_node, animal_filter_mask,
          face_landmark_detector, body_landmarks,
          speaker_node, fondo_virtual, csi_cam_pub
      → Hilo de boot (detached):
          sleep(3s)
          checkChatAvailable()  [hasta 10s, curl https://api.groq.com]
          if (!internet) → showWifiSetup_ = true → espera conexión
          configure_node() × 11  [con reintentos ×3 y sleep 2s entre cada uno]
          configStatus = "Yaren listo!"
          Para música de boot
          configuring = false
  → startRenderThread()   → renderLoop() en thread separado
  → spin loop:
      rclcpp::spin_some()
      drawWindow()   → cv::imshow() + cv::waitKey(1)
```

---

## 12. Notas de concurrencia y fixes aplicados

El código incluye fixes documentados con prefijo `// FIX-X:`:

| Fix | Descripción |
|---|---|
| **FIX-A** | `configTotal = 11` (número exacto de `configure_node()` llamados) |
| **FIX-B** | `configProgress` como `atomic<int>` para acceso seguro cross-thread |
| **FIX-C** | `cv::imshow()` se hace fuera del `frameMutex`: se clona el frame con lock, se muestra sin lock |
| **FIX-D** | `std::system()` bloqueante se lanza en `std::thread` detached para no retener `navMutex` |
| **FIX-E** | Todas las operaciones SDL_mixer (`Mix_VolumeMusic`, `Mix_PlayMusic`, `Mix_FreeMusic`) protegidas por `audioMutex_` compartido |
| **FIX-F** | Verificación `joinable()` antes de `join()` en threads secundarios |
| **FIX-G** | `menuMusic` siempre se detiene y libera antes de cargar nueva pista; hilo de limpieza post-fade |
| **FIX-H** | `applyDateTime()` usa `editYear` real en lugar de año hardcodeado |
| **FIX-I** | Publicaciones ROS2 (`modePublisher->publish()`) se hacen fuera de `navMutex` |
| **FIX-L** | `clampDate()` usa `editYear` para calcular correctamente años bisiestos |
| **FIX-M** | `hoveredSettings` y `hoveredPower` como `std::atomic<bool>` para acceso seguro desde el thread de render |

### Mutexes utilizados

| Mutex | Protege |
|---|---|
| `navMutex` | `navStack`, `hoveredItem`, `activeMode`, `activeStopCmd`, `active_lifecycle_nodes` |
| `modeFlagMutex` | `showSettings_`, `showRadio_`, `showRoutines_`, `showWifiSetup_` |
| `audioMutex_` | Todas las llamadas a SDL_mixer |
| `overlayMutex` | `faceOverlay`, `overlayMessage`, `micCountdownSecs` |
| `frameMutex` | `latestFrame` (compartido entre renderThread y drawWindow) |
| `configStatusMutex` | `configStatus`, `configProgress` (lectura en render, escritura en boot thread) |
| `visemeMutex_` | `visemeQueue_`, `visemeDeadline_`, `visemeBlend_` |
| `idleStateMutex` | `lastIdleState` (evita publicar el mismo estado dos veces) |