Documentación Técnica: Paquete yaren_emotions y Pipeline de Entrenamiento

Paquetes ROS2: yaren_emotions (y soporte de cámara)

Plataforma: Jetson Orin Nano · ROS2 Humble · Ubuntu 22.04

Dependencias principales: TensorFlow (Keras), MediaPipe FaceMesh, OpenCV, GStreamer, NumPy, scikit-learn, pandas
Índice

    Visión general

    Arquitectura del sistema

    Cámara CSI — csi_cam_pub.py

    Detector de emociones — emotion_detection_node.py

    Configuración y Pipeline de Entrenamiento

    Procesamiento de Datos — DataProcessor

    Visualización y Análisis — Visualizer

    Tópicos ROS2 del paquete

    Notas de concurrencia y diseño

1. Visión general

El paquete yaren_emotions implementa un sistema robusto de reconocimiento de emociones faciales para el robot YAREN2. Está compuesto por dos partes principales:

    Nodo de Inferencia ROS2: Un nodo Lifecycle que captura video de la cámara, detecta rostros usando MediaPipe FaceMesh y clasifica la emoción en tiempo real usando un modelo convolucional entrenado a medida en TensorFlow.

    Pipeline de Machine Learning: Un conjunto de scripts en Python (main.py, explore_dataset.py, DataProcessor, etc.) dedicados a la carga, preprocesamiento, aumento de datos y entrenamiento del modelo clasificador utilizando el dataset FER2013.

El sistema detecta 7 emociones básicas: Angry, Disgust, Fear, Happy, Sad, Surprise, Neutral (con soporte bilingüe EN/ES dinámico).
2. Arquitectura del sistema
Plaintext

┌─────────────────────────────────────────────────────────────────────┐
│                          yaren_emotions                             │
│                                                                     │
│  ┌──────────────────┐   /csi_camera/image_raw                       │
│  │  csi_cam_pub.py  │ ────────────────────────────────┐             │
│  │  (Cámara CSI/USB)│                                 ▼             │
│  └──────────────────┘                    ┌──────────────────────┐   │
│                                          │ EmotionDetectionNode │   │
│  /yaren/is_english                       │ (MediaPipe + TF)     │   │
│  ──────────────────────────────────────▶ │                      │   │
│  (QoS Transient Local)                   └───────┬──────────┬───┘   │
│                                                  │          │       │
│                                         /emotion │          │ /yaren_mode 
│                                          ▼       │          ▼       │
└─────────────────────────────────────────────────────────────────────┘

Nodos del paquete ROS2
Nodo	Archivo	Tipo	Función
camara	csi_cam_pub.py	Node	Captura y publica frames de la cámara (CSI/USB)
detector	emotion_detection_node.py	LifecycleNode	Extracción de rostros y clasificación de emociones
3. Cámara CSI — csi_cam_pub.py

Nodo publicador de imágenes que soporta tanto cámara CSI IMX219 (vía GStreamer + nvargus) como cámara USB (vía V4L2), idéntico en estructura al de otros módulos del robot.
Pipeline GStreamer (CSI)
Python

def gstreamer_pipeline(sensor_id, capture_width, capture_height,
                        display_width, display_height, framerate, flip_method):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! video/x-raw, format=(string)BGR ! appsink"
    )

Nota: Actualmente el código está configurado en fallback para USB (cv2.VideoCapture(0)). Para el despliegue final en la Jetson, se debe reactivar cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER).
4. Detector de emociones — emotion_detection_node.py

Nodo Lifecycle que orquesta la detección en tiempo real. Utiliza MediaPipe para localizar el rostro y extraer una región de interés (ROI), que luego es evaluada por un modelo Keras pre-entrenado.  
Estados lifecycle
Transición	Acción
on_configure	Carga el modelo .h5 de Keras; inicializa MediaPipe FaceMesh; crea publicadores y suscriptor de idioma.
on_activate	Suscribe a /csi_camera/image_raw; configura UI en pantalla completa; lanza el hilo de inferencia daemon.
on_deactivate	Destruye suscripción de cámara; cierra ventanas de OpenCV; espera a que el hilo de inferencia termine (timeout 2s).
on_cleanup	Libera modelo de TF y MediaPipe de la RAM; destruye publishers/subscribers.
on_shutdown	Fuerza cierre de hilos y ventanas.
  

Al igual que en otros nodos pesados, el modelo se carga en on_configure. Se hace un warmup del modelo pasando un tensor vacío (dummy) de 48x48x3 para evitar latencia en la primera predicción en vivo.
Patrón de Inferencia e Hilos
Python

# Hilo principal (ROS Executor)
if self._frame_count % self._INFER_EVERY == 0:
    with self._lock:
        self._latest_frame = frame.copy()

# Hilo dedicado (_infer_loop)
while self._active and rclpy.ok():
    with self._lock:
        # Extrae frame y procesa ROI
        preds = self.model(roi, training=False)

    _INFER_EVERY = 3: Procesa 1 de cada 3 frames para no saturar la CPU/GPU.

    MediaPipe: Localiza los landmarks, calcula un bounding box expandido en 40 px, recorta el rostro, y lo redimensiona a 48x48 para Keras.

Interfaz UI e Integración con el Sistema

La ventana de OpenCV se lanza en pantalla completa usando un truco con xdotool para forzar el foco y colocarla siempre visible (WND_PROP_TOPMOST).

Si el usuario hace clic izquierdo (EVENT_LBUTTONDOWN) en la pantalla:

    Publica 'idle' en /yaren_mode.

    El sistema centralizado (face_screen) intercepta el cambio y automáticamente solicita el on_deactivate del nodo de emociones.

5. Configuración y Pipeline de Entrenamiento

El paquete incluye un entorno completo para entrenar el modelo (fuera del robot) estructurado en módulos orientados a objetos.
Config (Clase central)

Define los hiperparámetros estáticos:

    Modelo: Entrada 48x48, 7 clases.

    Entrenamiento: Batch size 32, Learning Rate 0.0001, 25 Epochs.

    Dataset: FER2013 (archive/fer2013/fer2013/fer2013.csv).

    Callbacks: EarlyStopping (paciencia 11) y ReduceLROnPlateau (factor 0.3, paciencia 7).

Data Augmentation

Configurado mediante ImageDataGenerator para mitigar el sobreajuste:

    Rotación (15°), desplazamientos (15%), cortes (15%), zoom (15%) y volteo horizontal.

6. Procesamiento de Datos — DataProcessor

Encargado de convertir el CSV plano de FER2013 a tensores listos para entrenamiento.

    pixels_to_images(): Lee el string de píxeles separados por espacio, lo convierte a numpy array de 48x48, y lo pasa a formato RGB de 3 canales usando cv2.cvtColor(..., cv2.COLOR_GRAY2RGB).

    encode_labels(): Convierte las etiquetas numéricas a One-Hot Encoding usando LabelEncoder y to_categorical.

    Split y Normalización: Divide en Train/Valid (90/10) y normaliza los tensores dividiendo por 255.0.

7. Visualización y Análisis — Visualizer

Provee herramientas completas para entender el dataset y el rendimiento del modelo (a través de Matplotlib y Seaborn):

    Dataset: plot_emotion_distribution() (barras de frecuencia) y plot_sample_images() (muestra visual del dataset).

    Entrenamiento: plot_training_history() (gráficas de pérdida y precisión).

    Evaluación: plot_confusion_matrix(), generate_classification_report() y plot_prediction_examples() (muestra inferencias correctas vs incorrectas visualmente).

Además, el script independiente explore_dataset.py permite hacer un balance de clases y analizar estadísticas de los píxeles (brillo medio, desviación estándar, etc.) por cada emoción.
8. Tópicos ROS2 del paquete
Publicados
Tópico	Tipo	Nodo	Descripción
/csi_camera/image_raw	sensor_msgs/Image	camara	Frame capturado de la cámara.
/emotion	std_msgs/Int16	detector	ID de la emoción detectada (0-6).
/yaren_mode	std_msgs/String	detector	Publica "idle" para notificar salida del módulo.
Consumidos
Tópico	Tipo	Nodo	Descripción
/csi_camera/image_raw	sensor_msgs/Image	detector	Recepción de imágenes.
/yaren/is_english	std_msgs/Bool	detector	Idioma actual (QoS transient_local) para etiquetado.
9. Notas de concurrencia y diseño
  

    language_callback dinámico: El array de emociones (EMOTIONS_EN o EMOTIONS_ES) se selecciona en vivo durante la inferencia basado en la variable self.is_english. Al tener QoS transient_local, el nodo recibe el estado del idioma tan pronto arranca.

    Liberación de memoria explícita: En on_cleanup, forzar self.model = None y self.face_mesh = None es vital en la Jetson Orin Nano, de lo contrario la memoria VRAM de la GPU no se liberaría correctamente entre activaciones del nodo.

    Gestor de ventanas robusto: El uso de un hilo daemon threading.Thread(target=force_focus, daemon=True).start() con time.sleep(0.3) evita bloqueos si la ventana de OpenCV tarda unos milisegundos en inicializarse en el compositor gráfico de Ubuntu.