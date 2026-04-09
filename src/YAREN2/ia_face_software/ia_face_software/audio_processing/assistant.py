import io
import os
import json
#import speech_recognition as sr
import sounddevice as sd
import whisper
import time
from datetime import datetime, timedelta
from queue import Queue
from tempfile import NamedTemporaryFile
import openai
import gtts
import pygame
import warnings
import numpy as np

# Ignorar advertencias
warnings.filterwarnings("ignore")

# Clave de OpenAI
#openai.api_key = os.getenv("OPENAI_API_KEY")
openai.api_key = ""  # Coloca tu clave de OpenAI aquí
print(openai.api_key )

class Assistant:
    def __init__(self, model, record_timeout, phrase_timeout, energy_threshold, wake_word, comands, prompt,basics_responses):
        self.transcription = ['']
        self.audio_model = whisper.load_model(model)
        self.record_timeout = record_timeout
        self.wake_word = [word.lower() for word in wake_word]
        self.comands = comands
        self.prompt = prompt
        self.basics_responses = basics_responses
        self.listening = True  # Nueva bandera para controlar la escucha


    def pause_listening(self):
        """Pausa la escucha del asistente."""
        self.listening = False
        print("Transcripción pausada.")

    def resume_listening(self):
        """Reanuda la escucha del asistente."""
        self.listening = True
    print("Transcripción reanudada.")


    def call_gpt(self, texto):
        print("Comunicando con OpenAI:", texto)
        try:
            respuesta = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",  # Usa el modelo más reciente disponible
                messages=[{
                            "role": "system",
                            "content": [
                                            {
                                                "type": "text",
                                                "text": self.prompt
                                            },{
                                                "type": "text",
                                                "text": json.dumps(self.basics_responses, ensure_ascii=False, indent=4)

                                            },
                                        ],
                            },{"role": "user", "content": texto}],
                temperature=0.5,
                max_tokens=150,
                top_p=1,
                frequency_penalty=0,
                presence_penalty=0.6
            )
            return respuesta["choices"][0]["message"]["content"]
        except Exception as e:
            print(f"Error en la llamada a OpenAI: {e}")
            return "Lo siento, no puedo procesar tu solicitud en este momento."

    def tts(self, texto):
        tts = gtts.gTTS(texto, lang='es')
        audio_path = '/tmp/audio.mp3'  # Seguro y rápido para ROS2
        tts.save(audio_path)
        return audio_path
    
    def play_audio_pygame(self, filename):
        try:
            # Una validación extra de seguridad
            if filename is None:
                print("Error interno: La ruta del audio está vacía.")
                return
                
            pygame.mixer.init()
            pygame.mixer.music.load(filename)
            pygame.mixer.music.play()
            
            # Esperar a que termine de hablar antes de seguir escuchando
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)
                
        except Exception as e:
            print(f"Error al reproducir el audio: {e}")

    def listen_movement_comand(self):
        samplerate = 16000  # Tasa de muestreo compatible con Whisper
        channels = 1  # Audio mono

        print("Iniciando escucha...")
        while True:
            try:
                # Capturar audio con sounddevice
                print("Grabando...")
                audio_data = sd.rec(int(self.record_timeout * samplerate), samplerate=samplerate, channels=channels, dtype='float32')
                sd.wait()  # Esperar a que termine la grabación
                print("Grabación completada.")

                # Convertir audio a un array NumPy compatible con Whisper
                audio_data = np.squeeze(audio_data)  # Reducir dimensiones si es necesario
                audio_data = audio_data.astype(np.float32)  # Asegurarse de que es float32

                # Transcribir el audio con Whisper
                result = self.audio_model.transcribe(audio_data, fp16=False, language='es')
                text = result['text'].strip()
                print(f"Texto transcrito: {text}")

                # Extraer las claves del JSON
                keys = list(self.basics_responses.keys())

                # Comprobar si "JULIO" fue mencionado
                # Procesar si el wake] _word está en el texto
                for word in self.wake_word:
                    if word in text.lower():
                        # Solo procesar la transcripción si JULIO fue mencionado
                        self.transcription.append(text)
                        print(text)  # Imprime la transcripción en tiempo real

                        # Guardar en el archivo inmediatamente
                        with open("transcript.txt", "a") as f:
                            f.write(text + "\n")

                        
                        mensaje = text.lower().replace(word, "").strip()  # Elimina el wake word
                        #mensaje = "saluda"
                        # Verificar si el mensaje está en la lista de comandos
                        mensaje_limpio = mensaje.replace(",", "").strip().replace(".", "").strip()
                        if mensaje:  # Asegúrate de que hay un mensaje después del wake word
                            mensaje = "Humano: " + mensaje
                            respuesta = self.call_gpt(mensaje)
                            print(respuesta)
                            ruta_audio = self.tts(respuesta)
                            self.play_audio_pygame(ruta_audio)
                            if mensaje_limpio in self.comands:
                                # Obtener e imprimir la posición del mensaje en la lista
                                indice = self.comands.index(mensaje_limpio)
                                print(f"El comando '{mensaje_limpio}' está en la lista de comandos en la posición {indice}.")
                                return indice, respuesta
                            else:
                                return -1,respuesta

                
                if any(key.lower() in text.lower() for key in keys):
                    # Si alguna clave del JSON está en el texto, extraer el mensaje relacionado con la clave
                    matching_key = next(key for key in keys if key.lower() in text.lower())
                    mensaje = matching_key  # El mensaje será la clave en este caso
                    mensaje = "Humano: " + mensaje
                    respuesta = self.basics_responses[matching_key]
                    print(respuesta)
                    ruta_audio = self.tts(respuesta)
                    self.play_audio_pygame(ruta_audio)
                    if matching_key in self.comands:
                        # Obtener e imprimir la posición del mensaje en la lista
                        indice = self.comands.index(matching_key)
                        print(f"El comando '{matching_key}' está en la lista de comandos en la posición {indice}.")
                        return indice, respuesta
                    else:
                        return -1,respuesta
                        

                # Si "JULIO" no se mencionó, no se procesa nada
                self.last_sample = bytes()  # Reiniciar last_sample para la próxima entrada

            except KeyboardInterrupt:
                break


# Aquí puedes incluir tu lógica para ejecutar el asistente
if __name__ == "__main__":
    # Instanciar el asistente con todos los parámetros necesarios
    assistant = Assistant(
        model='large',                # Modelo Whisper
        record_timeout=5,             # Tiempo máximo de grabación por frase
        phrase_timeout=2,             # Tiempo de espera para considerar una frase como finalizada
        energy_threshold=300,         # Umbral de energía para detección de sonido
        wake_word=['julio'] ,            # Palabra de activación
        comands= [] ,  # comands
        prompt= """Eres un asistente amigable diseñado para ayudar con tareas relacionadas con el robot YAREN.
        Responde de forma clara y útil a las solicitudes, y proporciona ayuda técnica cuando sea necesario.
        Responde en maximo 50 palabras""",          # Descripción del asistente (prompt)
        basics_responses = {}
    )
    assistant.listen()