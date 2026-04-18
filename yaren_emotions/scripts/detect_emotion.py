#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int16
import numpy as np
import tensorflow as tf
import cv2
import mediapipe as mp
import os
from ament_index_python.packages import get_package_share_directory 

class FacialExpressionModel(object):
    def __init__(self, model):
        self.model = model
        self.EMOTIONS_LIST = ["Angry", "Disgust", "Fear", "Happy",
                            "Sad", "Surprise", "Neutral"]

    def predict_emotion(self, img):
        img = np.expand_dims(img, axis=0)
        img = img / 255.0
        img = img.astype(np.float32)

        self.preds = self.model.predict(img, verbose=0)
        return self.EMOTIONS_LIST[np.argmax(self.preds)]

class EmotionDetectionNode(Node):
    def __init__(self):
        super().__init__('emotion_detection_node')
        
        # Cargar modelo
        pkg_path = get_package_share_directory('coco_emotions')
        model_path = os.path.join(pkg_path, 'models', 'model_mbn_1.h5')

        self.model = tf.keras.models.load_model(model_path)
        self.emotion_model = FacialExpressionModel(self.model)
        
        # Configurar MediaPipe
        mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = mp_face_mesh.FaceMesh(max_num_faces=1, refine_landmarks=True, 
                                min_detection_confidence=0.5, 
                                min_tracking_confidence=0.5)
        
        self.bridge = CvBridge()

        # Publicador de la emoción (dato numérico)
        self.publisher = self.create_publisher(Int16, '/emotion', 10)
        
        # Publicador de la imagen visual (opcional pero recomendado)
        self.vis_publisher = self.create_publisher(Image, '/emotion/image_annotated', 10)
        
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        
        self.get_logger().info('Emotion Detection Node initialized')

    def image_callback(self, msg):
        try:
            fr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Copia para visualización
            vis_frame = fr.copy()
            
            rgb_frame = cv2.cvtColor(fr, cv2.COLOR_BGR2RGB)
            results = self.face_mesh.process(rgb_frame)

            if results.multi_face_landmarks:
                for face_landmarks in results.multi_face_landmarks:
                    h, w, _ = fr.shape
                    x_coords = [lm.x * w for lm in face_landmarks.landmark]
                    y_coords = [lm.y * h for lm in face_landmarks.landmark]

                    x_min, x_max = int(min(x_coords)), int(max(x_coords))
                    y_min, y_max = int(min(y_coords)), int(max(y_coords))

                    expand = 45
                    x_min = max(0, x_min - expand)
                    y_min = max(0, y_min - expand)
                    x_max = min(w, x_max + expand)
                    y_max = min(h, y_max + expand)

                    fc = rgb_frame[y_min:y_max, x_min:x_max]
                    roi = cv2.resize(fc, (48, 48))

                    if roi.size > 0:
                        pred = self.emotion_model.predict_emotion(roi)
                        
                        # 1. Publicar el dato numérico
                        self.publisher.publish(Int16(data=self.emotion_model.EMOTIONS_LIST.index(pred)))
                        
                        # 2. Dibujar en la imagen
                        cv2.rectangle(vis_frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                        cv2.putText(vis_frame, pred, (x_min, y_min - 10), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            # Publicar la imagen anotada al tópico (para ver en RQT si quieres)
            out_msg = self.bridge.cv2_to_imgmsg(vis_frame, encoding='bgr8')
            self.vis_publisher.publish(out_msg)

            # --- AQUI ESTA EL CAMBIO PARA ABRIR VENTANA AUTOMATICA ---
            cv2.imshow("Emotion Detector", vis_frame)
            # waitKey(1) es CRITICO. Permite a OpenCV refrescar la ventana. 
            # Si lo quitas, la ventana se colgará o no aparecerá.
            cv2.waitKey(1) 

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    emotion_detection_node = EmotionDetectionNode()
    
    try:
        rclpy.spin(emotion_detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Asegurarse de cerrar la ventana al terminar con Ctrl+C
        cv2.destroyAllWindows()
        emotion_detection_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
