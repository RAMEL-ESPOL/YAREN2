# Documentación Técnica: Paquete `yaren_chat`
**Paquete ROS2:** `yaren_chat2`  
**Plataforma:** Jetson Orin Nano · ROS2 Humble · Ubuntu 22.04  
**Dependencias principales:** llama.cpp, Vosk, Piper TTS, LangGraph, LangChain-Groq, PyAudio

---

## Índice

1. [Visión general](#1-visión-general)
2. [Arquitectura del sistema](#2-arquitectura-del-sistema)
3. [Módulo STT — `stt_lifecycle_node.py`](#3-módulo-stt--stt_lifecycle_nodepy)
4. [Módulo LLM (Python) — `llm_lifecycle_node.py`](#4-módulo-llm-python--llm_lifecycle_nodepy)
5. [Módulo LLM (C++) — `llm_lifecycle_node.cpp`](#5-módulo-llm-c--llm_lifecycle_nodecpp)
6. [Módulo TTS — `tts_lifecycle_node.py`](#6-módulo-tts--tts_lifecycle_nodepy)
7. [Agente de movimiento — `state_graph.py` / `MovementDetectionAgent`](#7-agente-de-movimiento--state_graphpy--movementdetectionagent)
8. [Orquestador de lifecycle — `control_manager_node.cpp`](#8-orquestador-de-lifecycle--control_manager_nodecpp)
9. [Configuración — `config.py`](#9-configuración--configpy)
10. [Tópicos ROS2 del paquete](#10-tópicos-ros2-del-paquete)
11. [Flujo completo de una conversación](#11-flujo-completo-de-una-conversación)
12. [Gestión de idioma bilingüe](#12-gestión-de-idioma-bilingüe)
13. [Notas de concurrencia](#13-notas-de-concurrencia)

---

## 1. Visión general

`yaren_chat` implementa el pipeline conversacional completo de YAREN2: desde la captura de voz del niño hasta la respuesta hablada del robot, pasando por el reconocimiento de texto, la generación de lenguaje natural y la síntesis de voz.

El sistema está diseñado para niños de 7–12 años en entornos hospitalarios, por lo que el LLM opera bajo un **system prompt estricto** que prohíbe consejos médicos, mantiene un tono positivo y evita anotaciones de acción en la salida de texto.

El pipeline es **productor-consumidor desacoplado**: el LLM emite frases por feedback de action mientras el TTS las sintetiza y reproduce en paralelo, minimizando la latencia percibida por el usuario.

---

## 2. Arquitectura del sistema

```
┌─────────────────────────────────────────────────────────────┐
│                        yaren_chat                           │
│                                                             │
│  ┌──────────┐    /response_person    ┌──────────────────┐  │
│  │   STT    │ ──────────────────────▶│      TTS         │  │
│  │ (Vosk)   │                        │   (Piper TTS)    │  │
│  └──────────┘                        └────────┬─────────┘  │
│       ▲                                       │            │
│       │ /stt_terminado (False)                │ /response_llama (Action)
│       │                                       ▼            │
│       │                              ┌──────────────────┐  │
│       └──────────────────────────────│      LLM         │  │
│         /stt_terminado (True)        │ (Groq / llama)   │  │
│                                      └────────┬─────────┘  │
│                                               │            │
│                                               ▼            │
│                                      ┌──────────────────┐  │
│                                      │ MovementAgent    │  │
│                                      │ (LangGraph)      │  │
│                                      └──────────────────┘  │
└─────────────────────────────────────────────────────────────┘

Tópicos externos relevantes:
  /yaren/mic_owner        ← face_screen.cpp (propietario del mic)
  /yaren/current_language ← gestor_idioma.py
  /yaren/tts_text         → face_screen.cpp (lip sync)
  /audio_playing          → face_screen.cpp (pausa música menú)
  /joint_trajectory_controller/follow_joint_trajectory → motores
```

### Nodos del paquete

| Nodo | Archivo | Tipo | Función |
|---|---|---|---|
| `stt_lifecycle_node` | `stt_lifecycle_node.py` | LifecycleNode (Python) | Reconocimiento de voz offline con Vosk |
| `llm_lifecycle_node` | `llm_lifecycle_node.py` | LifecycleNode (Python) | LLM via Groq API + LangGraph |
| `llm_lifecycle_node` | `llm_lifecycle_node.cpp` | LifecycleNode (C++) | LLM local con llama.cpp (Llama 3.2 1B GGUF) |
| `tts_lifecycle_node` | `tts_lifecycle_node.py` | LifecycleNode (Python) | Síntesis y reproducción de voz con Piper |
| `lifecycle_nodes_manager` | `control_manager_node.cpp` | Node (C++) | Orquesta transiciones STT↔LLM↔TTS |

> **Nota:** Los nodos `llm_lifecycle_node.py` y `llm_lifecycle_node.cpp` son implementaciones alternativas del mismo nodo LLM — el `.py` usa la API de Groq (online), el `.cpp` usa llama.cpp localmente (offline). Se usa uno u otro según disponibilidad de internet.

---

## 3. Módulo STT — `stt_lifecycle_node.py`

Nodo de reconocimiento de voz offline basado en **Vosk** con soporte bilingüe (español/inglés).

### Modelos de voz

| Idioma | Modelo | Ruta |
|---|---|---|
| Español | `vosk-model-es-0.42` | `<pkg_share>/models/STT/vosk-model-es-0.42` |
| Inglés | `vosk-model-en-us-0.22-lgraph` | `<pkg_share>/models/STT/vosk-model-en-us-0.22-lgraph` |

### Estados lifecycle

| Transición | Acción |
|---|---|
| `on_configure` | Carga el modelo Vosk según `idioma_actual`; suscribe a `/stt_terminado` |
| `on_activate` | Publica `mic_owner = "chat_stt"`; lanza `_recognize_speech()` en thread |
| `on_deactivate` | `is_recognizing = False`; libera evento bloqueante; espera join del thread |

### Mecanismo de control de micrófono

El STT respeta un sistema de ownership del micrófono mediante el tópico `/yaren/mic_owner`:

```python
# Solo captura audio cuando el owner es "chat_stt"
if not self.mic_owner_event.wait(timeout=0.1):
    continue   # otro nodo tiene el mic — ceder ciclo
```

Esto evita conflictos con el test de micrófono de `face_screen.cpp` que también usa el dispositivo de audio.

### Flujo de reconocimiento (`_recognize_speech`)

```
Abrir stream PyAudio (16000 Hz, mono, int16, buffer 8000)
Crear KaldiRecognizer(vosk_model, 16000)

loop while is_recognizing:
    ① Esperar mic_owner_event (cede si otro nodo tiene el mic)
    ② stream.read(4000)
    ③ Si tts_finished_event no está set → descartar audio (reset recognizer)
    ④ recognizer.AcceptWaveform(data)
       → Si hay texto reconocido:
          a. tts_finished_event.clear()          # bloquear próximos ciclos
          b. Publicar PersonResponse en /response_person
          c. Publicar Bool(True) en /stt_terminado
          d. tts_finished_event.wait()           # esperar que Yaren termine
          e. sleep(0.5) + flush buffer de audio  # descartar eco de Yaren
          f. Nuevo KaldiRecognizer               # resetear estado ASR
```

### Prevención de eco (`tts_finished_event`)

El evento `tts_finished_event` es el mecanismo central anti-eco:

- Se **limpia** (`clear()`) justo antes de publicar el texto reconocido.
- Se **libera** (`set()`) cuando llega `Bool(False)` en `/stt_terminado` (el TTS terminó de reproducir).
- Mientras está limpio, el recognizer se descarta en cada ciclo en lugar de procesar audio, evitando que Yaren se escuche a sí mismo.

### Cambio de modelo en caliente (`_switch_model_in_memory`)

```python
# 1. Parar hilo de reconocimiento si está corriendo
# 2. Liberar modelo anterior + gc.collect()
# 3. Cargar nuevo modelo desde ruta correspondiente
# 4. Reiniciar hilo si estaba corriendo
```

Gestiona la memoria explícitamente con `del self.vosk_model` + `gc.collect()` porque los modelos Vosk son pesados (~500 MB) y la Jetson tiene RAM limitada.

### Supresión de errores ALSA

```python
@contextmanager
def noalsaerr():
    asound = cdll.LoadLibrary('libasound.so')
    asound.snd_lib_error_set_handler(c_error_handler)
    yield
    asound.snd_lib_error_set_handler(None)
```

Redirige los mensajes de error de ALSA a una función vacía para evitar spam en consola al abrir el stream de PyAudio.

---

## 4. Módulo LLM (Python) — `llm_lifecycle_node.py`

Implementación del nodo LLM usando **Groq API** (online) con **LangGraph** para orquestación de agentes. Es la implementación activa cuando hay conexión a internet.

### Dependencias clave
- `langchain-groq`: cliente LLM con modelo `llama-3.1-8b-instant`
- `LangGraph`: grafo de estados para detección de movimiento + generación de respuesta
- `StateGraphLLM`: clase interna que combina el agente conversacional con `MovementDetectionAgent`

### Estados lifecycle

| Transición | Acción |
|---|---|
| `on_configure` | Crea el `ActionServer` en `/response_llama` |
| `on_activate` | Log de activación; el servidor ya está listo desde configure |
| `on_deactivate` | Reinicia `StateGraphLLM` y `config` → limpia historial de conversación |
| `on_cleanup` | Destruye el action server |

### Selección dinámica de prompt

```python
if self.idioma_actual == "en":
    base_prompt = SYSTEM_PROMPT_BASE_en
    lang_reinforcement = "RESPOND STRICTLY IN ENGLISH FROM THE FIRST WORD."
else:
    base_prompt = SYSTEM_PROMPT_BASE_es
    lang_reinforcement = "RESPONDE ESTRICAMENTE EN ESPAÑOL DESDE LA PRIMERA PALABRA."

prompt_final = f"{base_prompt}\n\n{lang_reinforcement}"
```

El refuerzo de idioma se añade como segunda línea para darle mayor prioridad frente al system prompt base, evitando que el LLM mezcle idiomas en la salida.

### `execute_response_generation` (action callback)

```
1. Construir messages = [SystemMessage(prompt_final), HumanMessage(user_input)]
2. state_graph_llm.process_info(messages, config)
   → Detectar intención de movimiento (MovementDetectionAgent)
   → Generar respuesta LLM (Groq API streaming)
3. Si robot_action_required → execute_robot_movement(joints_to_move)
4. Iterar ai_response carácter a carácter:
   → Acumular en buffer
   → Al detectar '.', '!' o '?' seguido de espacio:
       - clean_text(sentence)
       - publish_feedback(current_chunk)
       - vaciar buffer
5. publish_feedback(is_last_chunk=True)
6. goal_handle.succeed()
```

### Movimiento de motores (`execute_robot_movement`)

```python
def execute_robot_movement(self, joints_to_move: dict):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ["joint_1", ..., "joint_12"]
    point.positions = [0.0] * 12
    # Mapear joints_to_move al índice correcto
    point.time_from_start = Duration(sec=3)
    joint_action_client.send_goal_async(goal)
```

Envía la trayectoria de manera **asíncrona** (no bloqueante) para que los motores se muevan mientras el TTS está reproduciendo la respuesta verbal.

### Historial de conversación

El historial se mantiene dentro de `StateGraphLLM` via LangGraph con `thread_id=1`. Al desactivarse el nodo, se reinicia con un nuevo `StateGraphLLM()`, efectivamente limpiando el historial para la próxima sesión de chat.

---

## 5. Módulo LLM (C++) — `llm_lifecycle_node.cpp`

Implementación alternativa del nodo LLM usando **llama.cpp** directamente, sin dependencia de internet. Usa el modelo `Llama-3.2-1B-Instruct.Q4_K_M.gguf` cuantizado a 4 bits.

### Configuración del modelo

```cpp
static const std::unordered_map<std::string, double> CONFIGURATIONS_ = {
    {"n_gpu_layers",            0      },  // Sin GPU (Jetson CUDA limitado con llama.cpp)
    {"n_ctx",                   2048   },  // Ventana de contexto
    {"n_batch",                 512    },  // Batch de procesamiento
    {"n_threads",               4      },  // Hilos de CPU
    {"min_p",                   0.05   },  // Filtro min-p
    {"top_k",                   40     },  // Top-K sampling
    {"top_p",                   0.95   },  // Top-P (nucleus) sampling
    {"temp",                    0.7    },  // Temperatura
    {"max_tokens_chat",         300    },  // Máx tokens por respuesta
    {"max_tokens_summary",      150    },  // Máx tokens para resumen de contexto
    {"token_buffer_size",       256    },  // Buffer por token
    {"context_usage_threshold", 0.85   },  // Umbral para comprimir contexto
};
```

### Estados lifecycle

| Transición | Acción |
|---|---|
| `on_configure` | Carga el modelo GGUF, inicializa contexto llama, construye sampler chain, crea ActionServer |
| `on_activate` | Sin acción adicional (modelo ya cargado en configure) |
| `on_deactivate` | Sin acción adicional |
| Destructor | Libera sampler, contexto y modelo (`llama_sampler_free`, `llama_free`, `llama_model_free`) |

### Sampler chain

```cpp
sampler_ = llama_sampler_chain_init(llama_sampler_chain_default_params());
llama_sampler_chain_add(sampler_, llama_sampler_init_min_p(0.05, 1));
llama_sampler_chain_add(sampler_, llama_sampler_init_top_k(40));
llama_sampler_chain_add(sampler_, llama_sampler_init_top_p(0.95, 1));
llama_sampler_chain_add(sampler_, llama_sampler_init_temp(0.7));
llama_sampler_chain_add(sampler_, llama_sampler_init_dist(LLAMA_DEFAULT_SEED));
```

El orden importa: min_p filtra primero, luego top_k, luego top_p, luego temperatura, finalmente muestreo.

### `execute_response_generation`

```
std::lock_guard<std::mutex> lock(llama_mutex_)  // serializar inferencias

1. Construir conversation_history_ con system + user
2. llama_chat_apply_template() → prompt string
3. llama_tokenize() → prompt_tokens
4. manage_context(n_prompt_tokens)               // comprimir si necesario
5. llama_batch_get_one(prompt_tokens)
6. Loop de decodificación token a token:
   a. Verificar espacio en contexto (n_ctx_used + batch.n_tokens > n_ctx → break)
   b. goal_handle->is_canceling() → cancelar limpiamente
   c. llama_decode(ctx_, batch)
   d. llama_sampler_sample(sampler_, ctx_, -1)
   e. llama_vocab_is_eog() → break si fin de generación
   f. llama_token_to_piece() → string del token
   g. Acumular en buffer; al detectar '.!?' → publish_feedback(clean_text(sentence))
7. publish_feedback(is_last_chunk=True)
8. goal_handle->succeed()
```

### Gestión de contexto (`manage_context`)

Cuando el contexto supera el 85% de capacidad:

```cpp
bool manage_context(int tokens_to_add) {
    if ((n_ctx_used + tokens_to_add) >= (n_ctx * 0.85)) {
        // 1. Generar resumen de la conversación
        std::string summary = generate_conversation_summary();
        // 2. Limpiar KV cache completamente
        llama_kv_self_clear(ctx_);
        llama_kv_self_seq_rm(ctx_, -1, -1, -1);
        // 3. Inyectar resumen en el system prompt
        system_prompt_base_ += "\n\nResumen de la conversación hasta ahora:\n" + summary;
        return true;
    }
    return false;
}
```

`generate_conversation_summary()` usa los últimos 4 intercambios (8 mensajes) para generar un resumen compacto con el mismo modelo, luego limpia el KV cache y comienza con el contexto reducido.

### `TextProcessor::clean_text`

```cpp
static std::vector<std::string> clean_text(const std::string& text);
```

Procesamiento en pipeline UTF-8 completo:
1. Conversión `string` → `wstring` con `wstring_convert<codecvt_utf8<wchar_t>>`.
2. Regex wide para eliminar puntuación `[!¡?¿*,.:;()\\[\\]{}]` → espacio.
3. Colapsar espacios múltiples.
4. Reconversión `wstring` → `string`.
5. Segmentación en oraciones por `([.!?])\s+`.

Retorna un vector de oraciones limpias listas para TTS.

---

## 6. Módulo TTS — `tts_lifecycle_node.py`

Nodo de síntesis y reproducción de voz usando **Piper TTS**, con pipeline productor-consumidor para baja latencia y publicación de visemas para lip sync.

### Modelos de voz

| Idioma | Modelo | Características |
|---|---|---|
| Español | `es_MX-claude-high.onnx` | Voz mexicana, alta calidad |
| Inglés | `en_US-lessac-medium.onnx` | Voz americana, calidad media |

Ambos modelos se cargan con `use_cuda=True` para aprovechar la GPU de la Jetson.

### Estados lifecycle

| Transición | Acción |
|---|---|
| `on_activate` | Crea `ActionClient` hacia `/response_llama`; limpia `audio_queue`; lanza `_audio_worker` |
| `on_deactivate` | Log de desactivación (el worker es daemon, termina automáticamente) |

### Pipeline productor-consumidor

```
process_input_person(msg)
  │
  ├──► Thread: _send_goal_and_receive_chunks()   [PRODUCTOR]
  │         Envía goal al LLM
  │         _feedback_callback() → audio_queue.put(chunk)  [por cada frase]
  │         Al terminar → audio_queue.put(None)
  │         Espera tts_done.wait()
  │
  └──► Thread: _audio_worker()                   [CONSUMIDOR]
            while True:
                chunk = audio_queue.get()
                if chunk is None: break
                _play_audio(chunk)
            tts_done.set()                        [desbloquea productor]
            stt_status_publisher.publish(False)   [STT puede escuchar]
```

Este diseño permite que el TTS empiece a sintetizar y reproducir la **primera frase** mientras el LLM aún está generando las siguientes, reduciendo la latencia de primera respuesta.

### `_play_audio(text_to_speak)`

```python
1. Publicar text_to_speak en /yaren/tts_text   (lip sync)
2. Publicar Bool(True) en /audio_playing        (pausa música de menú)
3. SynthesisConfig(length_scale=1.2, noise_scale=0.5, noise_w_scale=0.8)
4. voice.synthesize_wav(text, wav_file, syn_config)  [bajo voice_lock]
5. _publish_visemes_from_wav(tmp_path)
6. playsound(tmp_path)
7. os.unlink(tmp_path)                          (limpiar archivo temporal)
8. Publicar Bool(False) en /audio_playing
```

El `SynthesisConfig` está ajustado para sonar más lento (`length_scale=1.2`) y con menor varianza de ruido, adecuado para niños.

### Sistema de visemas (`_publish_visemes_from_wav`)

Análisis de amplitud del WAV en ventanas de 50 ms para asignar índice de sprite de boca:

```python
chunk_ms   = 50
chunk_size = int(framerate * 50 / 1000)

for cada ventana:
    rms = sqrt(mean(sample²))
    
    if   rms < 200:    idx = 0   # boca cerrada
    elif rms < 800:    idx = 1   # apertura mínima
    elif rms < 2000:   idx = 3   # apertura media-baja
    elif rms < 5000:   idx = 5   # apertura media
    elif rms < 10000:  idx = 6   # apertura alta
    else:              idx = 7   # apertura máxima
```

Publica en `/yaren/tts_text` con el formato legacy:
```
"50:0,1,3,5,3,1,0,3,5,..."
```

El nodo `face_screen.cpp` interpreta este formato y aplica blend suave entre sprites.

### Cambio de voz en caliente (`_switch_voice_in_memory`)

```python
with voice_lock:
    del self.voice; self.voice = None; gc.collect()
    self.voice = PiperVoice.load(nueva_ruta, use_cuda=True)
```

Usa `voice_lock` para no descargar la voz mientras `_play_audio` la está usando.

---

## 7. Agente de movimiento — `MovementDetectionAgent`

Componente de `state_graph.py` que detecta intenciones de movimiento en el texto del usuario y las traduce a posiciones de joints Dynamixel.

### Arquitectura LangGraph

```
StateGraph(MovementState)
    entry: "detect_movement"
    detect_movement → plan_movement → END
```

`MovementState` extiende `TypedDict` con:
- `messages`: historial de mensajes LangChain
- `movement_detected`: bool
- `movement_type`: string del movimiento identificado
- `joints_to_move`: dict joint_name → posición (rad)
- `detection_method`: "regex" | "llm" | "llm_error"

### Estrategia de detección en dos niveles

#### Nivel 1: Regex (≈0 ms, sin API)

```python
def _regex_detect(self, text: str) -> dict:
    # Prioridad máxima: bloque de empatía
    if re.search(r"\b(triste|llorar|dolor|...)\b", t):
        return {"movement_detected": False, "movement_type": "ninguno"}
    
    # Patrones de movimiento
    patterns = [
        (r"(?:levanta|sube|alza).*brazo.*derech", "levanta_el_brazo_derecho"),
        (r"(?:alz|levanta).*brazos", "alza_los_brazos"),
        ...
    ]
    for pattern, movement_type in patterns:
        if re.search(pattern, t, re.IGNORECASE):
            return {"movement_detected": True, "movement_type": movement_type}
    
    return {"movement_detected": False, "movement_type": "ninguno"}
```

**El bloque de empatía tiene prioridad máxima:** palabras como "triste", "dolor", "miedo" inhiben cualquier detección de movimiento, para que Yaren no haga gestos inapropiados al consolar al niño.

#### Nivel 2: LLM Groq (fallback, solo si regex encontró keywords de acción)

```python
if not result["movement_detected"]:
    if re.search(r"\b(mueve|gira|levanta|alza|...)\b", user_message):
        result = self._llm_detect(user_message)
```

El LLM solo se consulta si el regex detectó **palabras de acción** pero no identificó el movimiento específico, evitando llamadas innecesarias a la API durante conversación normal.

El prompt al LLM pide JSON estricto:
```python
prompt = (
    f'Clasifica si hay una orden de movimiento físico en: "{user_message}"\n'
    f'Tipos válidos: {", ".join(sorted(VALID_MOVEMENTS - {"ninguno"}))}\n'
    'Responde SOLO JSON: {"movement_detected": true/false, "movement_type": "TIPO"}'
)
```

### Mapa de movimientos (`JOINT_MAP`)

| Movimiento | Joints modificados | Descripción |
|---|---|---|
| `levanta_el_brazo_derecho` | 5, 6, 7, 8, 12 | Brazo derecho arriba, muñeca neutral |
| `levanta_el_brazo_izquierdo` | 8, 9, 10, 11, 12 | Brazo izquierdo arriba |
| `alza_los_brazos` | 5, 6, 7, 8, 9, 10, 11, 12 | Ambos brazos arriba |
| `estira_los_brazos` | 5, 9 | Brazos extendidos al frente |
| `mira_a_la_izquierda` | 3, 4 | Cabeza girada -0.8 rad |
| `mira_a_la_derecha` | 3, 4 | Cabeza girada +0.8 rad |
| `gira_a_la_izquierda` | 1 | Base -1.5 rad |
| `gira_a_la_derecha` | 1 | Base +1.5 rad |
| `hazte_el_loco` | 5, 6, 7, 8, 9, 10, 11, 12 | Posición expresiva/aleatoria |
| `vuelve_a_la_posicion_original` | 1–12 | Reset total a 0.0 rad |

El joint `8` con valor `0.5` y `joint_12` con `0.5` son los muñecos (wrists) que por defecto se mantienen en posición colgante natural.

### Movimientos válidos detectables

```python
VALID_MOVEMENTS = {
    "levanta_el_brazo_derecho", "levanta_el_brazo_izquierdo",
    "alza_los_brazos", "estira_los_brazos", "mueve_la_cabeza",
    "mira_a_la_izquierda", "mira_a_la_derecha", "gira_el_cuerpo",
    "gira_a_la_izquierda", "gira_a_la_derecha",
    "hazte_el_loco", "vuelve_a_la_posicion_original", "ninguno"
}
```

---

## 8. Orquestador de lifecycle — `control_manager_node.cpp`

Nodo C++ que orquesta las transiciones de estado entre STT, LLM y TTS de forma sincronizada, asegurando que solo un nodo esté activo a la vez.

> **Nota:** Este nodo es una alternativa al orquestador integrado en `face_screen.cpp`. Se usa cuando el chat se ejecuta de forma independiente sin la pantalla de cara.

### Lógica de orquestación

```
Arranque (_initial_configuration):
  STT → configure
  STT → activate          ← Yaren escucha
  LLM → configure
  TTS → configure

Evento /stt_terminado (True):  ← STT reconoció texto
  LLM → activate
  TTS → activate
  STT → deactivate         ← parar escucha mientras Yaren responde

Evento /stt_terminado (False): ← TTS terminó de reproducir
  STT → activate           ← Yaren vuelve a escuchar
  TTS → deactivate
  LLM → deactivate
```

### `change_node_state`

```cpp
bool change_node_state(const std::string& node_name, uint8_t transition_id,
                       std::chrono::seconds timeout = std::chrono::seconds(10))
{
    // 1. Seleccionar cliente según nombre ("stt", "llm", "tts")
    // 2. client->wait_for_service(timeout)
    // 3. async_send_request + future.wait_for(timeout)
    // 4. Retornar true/false
}
```

Usa `future.wait_for()` con timeout real para no bloquear indefinidamente si un nodo no responde, a diferencia de un `spin_until_future_complete` que podría deadlockear.

### Prevención de deadlock en callbacks

```cpp
void stt_status_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_lock_);
    stt_terminated_ = msg->data;
    manage_node_lifecycle();   // lanza thread detached
}

void _manage_lifecycle_thread() {
    bool terminated;
    {
        std::lock_guard<std::mutex> lock(state_lock_);
        terminated = stt_terminated_;
    }  // ← soltar lock ANTES de llamadas bloqueantes
    
    // change_node_state() es bloqueante — debe llamarse SIN el lock
    if (terminated) { ... }
}
```

El patrón correcto: copiar el estado bajo lock, soltar el lock, ejecutar las transiciones bloqueantes fuera del lock.

---

## 9. Configuración — `config.py`

### `CONFIGURATIONS`

```python
CONFIGURATIONS = {
    'temperature':           0.5,
    'max_completion_tokens': 300,
    'top_p':                 0.9,
    'model':                 'llama-3.1-8b-instant'
}
```

Parámetros del LLM Groq. `max_completion_tokens=300` limita la longitud de las respuestas para mantener el ritmo conversacional apropiado para niños.

### `SYSTEM_PROMPT_BASE_es` / `SYSTEM_PROMPT_BASE_en`

Prompts del sistema para cada idioma. Reglas clave:

| Regla | Motivación |
|---|---|
| Respuestas cortas y concisas | Evitar monólogos; mantener diálogo bidireccional |
| Nunca dar consejos médicos | Obligación ética en entorno hospitalario |
| Lenguaje adaptado a niños 7–12 años | Vocabulario y tono apropiado |
| Preguntar nombre/edad al inicio | Personalización de la interacción |
| **🚫 Cero anotaciones de acción** | El texto se lee literal por TTS; asteriscos y emoticonos sonarían extraños |
| Refuerzo de idioma explícito | Evitar mezcla de idiomas en la respuesta |

La regla de **cero anotaciones** (e.g. prohibir `*sonríe*`, `(ríe)`, `se inclina`) es crítica: todo lo que el LLM escriba será leído en voz alta por Piper TTS, por lo que cualquier marcador de acción sonaría literalmente al niño.

---

## 10. Tópicos ROS2 del paquete

### Publicados por `yaren_chat`

| Tópico | Tipo | Publicado por | Descripción |
|---|---|---|---|
| `/response_person` | `yaren_interfaces/PersonResponse` | STT | Texto reconocido del niño |
| `/stt_terminado` | `std_msgs/Bool` | STT (True) / TTS (False) | Señal de sincronización STT↔TTS |
| `/stt_listening` | `std_msgs/Bool` | STT | El STT está escuchando activamente |
| `/audio_playing` | `std_msgs/Bool` | TTS | Yaren está reproduciendo audio |
| `/yaren/tts_text` | `std_msgs/String` | TTS | Texto + visemas para lip sync |
| `/yaren/mic_owner` | `std_msgs/String` | STT | Propietario actual del micrófono |

### Consumidos por `yaren_chat`

| Tópico | Tipo | Consumido por | Publicado por |
|---|---|---|---|
| `/yaren/mic_owner` | `std_msgs/String` | STT (sub) | STT (pub), face_screen |
| `/yaren/current_language` | `std_msgs/String` | STT, LLM, TTS | `gestor_idioma.py` |
| `/response_person` | `yaren_interfaces/PersonResponse` | TTS | STT |
| `/stt_terminado` | `std_msgs/Bool` | STT, control_manager | TTS, control_manager |

### Action interfaces

| Acción | Tipo | Servidor | Cliente |
|---|---|---|---|
| `/response_llama` | `yaren_interfaces/ProcessResponse` | LLM node | TTS node |
| `/joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | ros2_control | LLM node (Python) |

### `yaren_interfaces/ProcessResponse`

```
# Goal
string input_text

# Feedback
string current_chunk    ← frase limpia lista para TTS
bool   is_last_chunk    ← señal de fin de generación

# Result
bool completed
```

---

## 11. Flujo completo de una conversación

```
Niño habla
    │
    ▼
[STT] stream.read(4000) → recognizer.AcceptWaveform()
    │ texto reconocido
    ▼
[STT] Publica /response_person { text: "hola yaren" }
[STT] Publica /stt_terminado { data: True }
[STT] tts_finished_event.clear()  ← bloquear eco
    │
    ├──► [control_manager] STT→deactivate, LLM→activate, TTS→activate
    │
    ▼
[TTS] process_input_person() recibe /response_person
[TTS] Lanza _send_goal_and_receive_chunks() + _audio_worker()
    │
    ▼
[TTS→LLM] send_goal_async("/response_llama", input_text="hola yaren")
    │
    ▼
[LLM] execute_response_generation()
    │   MovementDetectionAgent → _regex_detect("hola yaren")
    │   → ningún movimiento detectado
    │   Groq API → "¡Hola! ¿Cómo te llamas?"
    │
    ├── publish_feedback("¡Hola!")
    ├── publish_feedback("¿Cómo te llamas?")
    └── publish_feedback(is_last_chunk=True)
    │
    ▼
[TTS] _feedback_callback → audio_queue.put("¡Hola!")
                         → audio_queue.put("¿Cómo te llamas?")
    │
    ▼
[TTS._audio_worker]
    chunk = "¡Hola!"
    _play_audio("¡Hola!")
        → /yaren/tts_text: "50:0,1,3,5,3,1,0"   (visemas)
        → /audio_playing: True
        → Piper sintetiza WAV
        → _publish_visemes_from_wav()
        → playsound()
        → /audio_playing: False
    
    chunk = "¿Cómo te llamas?"
    _play_audio("¿Cómo te llamas?")
        → ...
    
    chunk = None → break
    tts_done.set()
    /stt_terminado: False
    │
    ├──► [control_manager] STT→activate, TTS→deactivate, LLM→deactivate
    │
    ▼
[STT] tts_finished_event.set()  ← volver a escuchar
[STT] Flush buffer + nuevo KaldiRecognizer
[face_screen] lip sync activo durante reproducción ✓
[face_screen] música de menú pausada durante audio ✓
```

---

## 12. Gestión de idioma bilingüe

El idioma se propaga desde `gestor_idioma.py` (nodo externo) via `/yaren/current_language` a todos los nodos del paquete:

```
/yaren/current_language  ("es" | "en")
    │
    ├── STT: _switch_model_in_memory()
    │         vosk-model-es → vosk-model-en (o viceversa)
    │
    ├── LLM (Python): cb_cambio_idioma()
    │         SYSTEM_PROMPT_BASE_es → SYSTEM_PROMPT_BASE_en
    │
    └── TTS: _switch_voice_in_memory()
              es_MX-claude-high → en_US-lessac-medium
```

El cambio es **en caliente**: cada nodo libera el modelo anterior, llama a `gc.collect()` y carga el nuevo sin necesidad de reiniciar el nodo lifecycle. En todos los casos se usa `voice_lock` / mutexes para evitar cambiar el modelo mientras está siendo usado activamente.

El LLM (C++) no tiene cambio de idioma en caliente ya que usa un modelo local único; el idioma se maneja añadiendo refuerzo en el system prompt.

---

## 13. Notas de concurrencia

### STT
- `_recognize_speech()` corre en un thread dedicado.
- `tts_finished_event` es un `threading.Event` usado como semáforo binario entre el thread de reconocimiento y el callback de `/stt_terminado`.
- `mic_owner_event` controla el acceso al hardware de audio.

### TTS
- `audio_queue` es un `queue.Queue` thread-safe (FIFO).
- `voice_lock` serializa acceso a `self.voice` entre el thread de reproducción y el callback de cambio de idioma.
- `tts_done` es un `threading.Event` para sincronizar el productor (espera en `tts_done.wait()`) y el consumidor (libera con `tts_done.set()`).

### LLM (C++)
- `llama_mutex_` serializa todas las inferencias; llama.cpp no es thread-safe.
- `goal_handle->is_canceling()` se verifica en cada iteración del loop de decodificación para poder cancelar limpiamente.

### Control Manager
- `state_lock_` protege `stt_terminated_` en el callback.
- Las llamadas `change_node_state()` son bloqueantes y **siempre** se ejecutan fuera del lock en un thread detached para no bloquear el executor de ROS2.