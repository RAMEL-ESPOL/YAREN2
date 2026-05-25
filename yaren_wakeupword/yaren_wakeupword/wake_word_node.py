import os
import json
import pyaudio
from vosk import Model, KaldiRecognizer

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy

class YarenWakeWordNode(Node):
    def __init__(self):
        super().__init__('yaren_wake_word_node')
        
        # --- Variables de Estado ---
        self.wake_word = "hola amigo"
        self.is_face_idle = False  # Por defecto no escuchamos hasta que C++ nos de luz verde
        
        # --- Publicadores y Suscriptores ---
        # 1. Suscriptor al estado de la cara (QoS Transient Local para leer el último estado al instante)
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.idle_sub = self.create_subscription(
            Bool, 
            '/yaren/face_idle', 
            self.idle_callback, 
            qos_profile
        )
        
        # 2. Publicador para despertar el menú
        self.wake_pub = self.create_publisher(Bool, '/yaren/wake_event', 10)
        
        # --- Configuración de Vosk ---
        # --- Configuración de Vosk ---
        # Obtenemos el directorio actual (la raíz del workspace) dinámicamente
        workspace_dir = os.getcwd()
        model_path = os.path.join(workspace_dir, 'src', 'YAREN2', 'yaren_chat', 'models', 'STT', 'vosk-model-small-es-0.42')
        
        if not os.path.exists(model_path):
            self.get_logger().error(f"Modelo no encontrado en: {model_path}")
            # Si falla, imprimimos el directorio actual para depurar fácilmente
            self.get_logger().error(f"Asegúrate de ejecutar el nodo desde la raíz del workspace. Directorio actual: {workspace_dir}")
            return

        self.model = Model(model_path)
        self.recognizer = KaldiRecognizer(self.model, 16000)
        # --- Configuración del Micrófono ---
        self.mic = pyaudio.PyAudio()
        self.stream = self.mic.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8000)
        self.stream.start_stream()
        
        self.timer = self.create_timer(0.1, self.audio_loop_callback)
        self.get_logger().info("🤖 Nodo yaren_wake_word en espera del estado de la pantalla...")

    def idle_callback(self, msg):
        """Actualiza el estado cuando entra o sale de un menú."""
        self.is_face_idle = msg.data
        if self.is_face_idle:
            self.get_logger().info("✅ Pantalla libre. Escuchando 'hola amigo'...")
            self.recognizer = KaldiRecognizer(self.model, 16000)
        else:
            self.get_logger().info("🛑 Menú activo. Micrófono pausado.")

    def audio_loop_callback(self):
        """Loop de lectura del micrófono."""
        
        # Si Yaren está en un menú, leemos el buffer para vaciarlo y no procesamos nada
        if not self.is_face_idle:
            try:
                # Vaciar el buffer evita que procese audio viejo al volver al estado Idle
                available = self.stream.get_read_available()
                if available > 0:
                    self.stream.read(available, exception_on_overflow=False)
            except Exception:
                pass
            return

        try:
            data = self.stream.read(4000, exception_on_overflow=False)
            if len(data) == 0:
                return

            if self.recognizer.AcceptWaveform(data):
                result = json.loads(self.recognizer.Result())
                text = result.get("text", "").lower()
                
                if self.wake_word in text:
                    self.get_logger().info(f"✨ ¡Palabra detectada!: {text}")
                    
                    # Avisamos a C++ que abra el menú
                    wake_msg = Bool()
                    wake_msg.data = True
                    self.wake_pub.publish(wake_msg)
                    
                    # Pausamos internamente para no re-disparar mientras C++ procesa
                    self.is_face_idle = False 
                    
        except Exception as e:
            pass # Silenciamos errores menores de desbordamiento de PyAudio

    def destroy_node(self):
        self.stream.stop_stream()
        self.stream.close()
        self.mic.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YarenWakeWordNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()