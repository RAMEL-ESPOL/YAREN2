# from gtts import gTTS
from moviepy.editor import ImageClip, concatenate_videoclips, AudioFileClip, CompositeVideoClip
import os
from pydub import AudioSegment, silence
import numpy as np
import sys
import time
import os
from gtts import gTTS
from concurrent.futures import ThreadPoolExecutor
from pydub import AudioSegment
from ament_index_python.packages import get_package_share_directory
# Añade el directorio src a sys.path
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
from united_morph_transition import morph_transition
from transition import save_frames_to_folder
import openai
from PIL import Image

import time



def ajustar_pitch(audio_path, pitch_semitonos, velocidad):
    """
    Ajusta el pitch sin modificar la velocidad del audio.
    Usa un archivo temporal para evitar errores de sobrescritura.
    """
    temp_audio_path = audio_path + ".temp.mp3"
    cambio_pitch = 2 ** (pitch_semitonos / 12.0)  # Calcula el cambio de tono
    comando = f"ffmpeg -y -i {audio_path} -filter:a rubberband=pitch={cambio_pitch}:tempo={velocidad} {temp_audio_path}"
    os.system(comando)

    if os.path.exists(audio_path):
        os.remove(audio_path)

    os.rename(temp_audio_path, audio_path)  # Renombra el archivo temporal al archivo original


def generar_audio(texto, audio_path, pitch_semitonos=5, velocidad=1.2):
   # Generar el audio usando Google TTS (acento de España 'es' o México 'com.mx')
    tts = gTTS(text=texto, lang='es', tld='com.mx') 
    tts.save(audio_path)
    
    # Mantenemos tu función de ajuste de pitch y velocidad intacta
    ajustar_pitch(audio_path, pitch_semitonos, velocidad)
    print(f"Audio guardado en: {audio_path}")

def eliminar_silencios(audio_path, trimmed_audio_path):
    audio = AudioSegment.from_file(audio_path)
    trimmed_audio = silence.detect_nonsilent(audio, min_silence_len=200, silence_thresh=-40)

    if trimmed_audio:
        start_trim, end_trim = trimmed_audio[0][0], trimmed_audio[-1][1]
        audio_trimmed = audio[start_trim:end_trim]
        audio_trimmed.export(trimmed_audio_path, format="mp3")
        return len(audio_trimmed) / 1000.0  # Duración en segundos
    else:
        raise ValueError("No se detectó contenido hablado en el audio.")

def mapear_texto_a_imagenes(total_duration, respuesta, image_folder, points_folder, phoneme_to_image, phoneme_to_points, shut_mouth, shut_mouth_points, mouth_emotion = "default"):
    unit_duration = total_duration / len(respuesta)
    duration_per_vocal = 0
    sequence, sequence_points, durations = [], [], []

    for char in respuesta:
        duration_per_vocal += unit_duration
        if char in phoneme_to_image or char in [" ", ".", "!", "?", "(", ")", ",", "¿", "¡"]:
            image_file = os.path.join(image_folder, phoneme_to_image.get(char, shut_mouth[mouth_emotion]))
            points_file = os.path.join(points_folder, phoneme_to_points.get(char, shut_mouth_points[mouth_emotion]))

            if os.path.exists(image_file):
                sequence.append(image_file)
                durations.append(duration_per_vocal)
                duration_per_vocal = 0
            if os.path.exists(points_file):
                sequence_points.append(points_file)

    return sequence, sequence_points, durations

def load_images(folder):
        images = []
        for file_name in sorted(os.listdir(folder)):
            if file_name.endswith((".png", ".jpg")):
                image_path = os.path.join(folder, file_name)
                img = Image.open(image_path).convert("RGBA")  # Asegúrate de que sea RGBA
                images.append(np.array(img))
        return images

def crear_video_boca(shut_mouth, sequence, sequence_points, durations, image_folder, transiciones_existentes, morph_transition, unit_duration, mouth_emotion = "default"):
    clips = []
    duracion_transicion = unit_duration / 6
    num_images = 2

    for i in range(len(sequence)):
        image = sequence[i]
        duration = durations[i]
        clip = ImageClip(image, duration=duration - duracion_transicion)
        clips.append(clip)

        if i < len(sequence) - 1 and sequence[i]  != os.path.join(image_folder, shut_mouth[mouth_emotion]) and sequence[i+1] != os.path.join(image_folder, shut_mouth[mouth_emotion]):
            
            number_1 = sequence[i].split('/')[-1].split('.')[0]
            #print(number_1)

            number_2 = sequence[i+1].split('/')[-1].split('.')[0]
            #print(number_2)

            if number_1 == number_2:
                clip = ImageClip(image, duration= duracion_transicion * num_images)
                clips.append(clip)
                #print("igual")
            else:

                if os.path.exists(transiciones_existentes + "/transition_frames_" + number_1 + " a "  + number_2) or os.path.exists(transiciones_existentes + "/transition_frames_" + number_2 + " a "  + number_1) :
                    if os.path.exists(transiciones_existentes + "/transition_frames_" + number_2 + " a "  + number_1):
                        frames = load_images(transiciones_existentes + "/transition_frames_" + number_2 + " a "  + number_1)
                        # Invertir el orden de los elementos de deformed_images_2
                        frames = frames[::-1]
                    else:
                        frames = load_images(transiciones_existentes + "/transition_frames_" + number_1 + " a "  + number_2)

                else:
                    frames = morph_transition(sequence[i], sequence[i+1], sequence_points[i], sequence_points[i+1], num_images)
                    save_frames_to_folder(frames, transiciones_existentes + "/transition_frames_" + number_1 + " a "  + number_2)
                
                clips.extend([ImageClip(np.array(frame), duration=duracion_transicion) for frame in frames])

    return concatenate_videoclips(clips, method="compose")

def crear_bucle_parpadeo(eyes_folder, open_eyes, total_duration, fps=24, eye_emotion = "open"):
    eye_states = ["closed", eye_emotion]
    state_durations = [0.1, 2]
    total_states_duration = sum(state_durations)
    loops_needed = int(np.ceil(total_duration / total_states_duration))

    eye_clips = []
    for _ in range(loops_needed):
        for state, duration in zip(eye_states, state_durations):
            eye_image = os.path.join(eyes_folder, open_eyes[state])
            if os.path.exists(eye_image):
                eye_clips.append(ImageClip(eye_image, duration=duration))

    return concatenate_videoclips(eye_clips, method="compose").subclip(0, total_duration)


def combinar_video_y_audio_en_archivo(mouth_video, blinking_eyes, audio_path, output_video_with_audio):
    final_video = CompositeVideoClip([
        mouth_video.set_position(("center", "bottom")),
        blinking_eyes.set_position(("center", "top"))
    ])

    audio_clip = AudioFileClip(audio_path)
    final_video = final_video.set_audio(audio_clip)
    final_video.write_videofile(output_video_with_audio, codec="libx264", audio_codec="aac", fps=24)


def combinar_video_y_audio(mouth_video, blinking_eyes, audio_path):
    # Combina los clips visuales
    final_video = CompositeVideoClip([
        mouth_video.set_position(("center", "bottom")),
        blinking_eyes.set_position(("center", "top"))
    ])

    # Añade el audio al video
    audio_clip = AudioFileClip(audio_path)
    final_video = final_video.set_audio(audio_clip)

    return final_video  # Retorna el objeto final combinado sin escribir en disco


def detectar_emocion_y_expresion(texto, emocion_a_expresion):
    # Detectar la emoción usando OpenAI
    respuesta = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",
        messages=[
            {
                "role": "system",
                "content": """Eres un asistente que identifica emociones en textos y las clasifica en las siguientes categorías: 
                                neutro, feliz, enojado, desagrado, tierno, silly, desinteres. Responde solo con una de estas palabras."""
            },
            {
                "role": "user",
                "content": f"Identifica la emoción predominante en este texto: \"{texto}\"."
            },
        ],
        max_tokens=5,
        temperature=0.3,
    )
    emocion = respuesta["choices"][0]["message"]["content"].strip().lower()

    # Validar la emoción
    emociones_validas = set(emocion_a_expresion.keys())
    if emocion not in emociones_validas:
        print(f"Emoción no reconocida: {emocion}. Usando expresiones predeterminadas.")
        emocion = "neutro"

    # Seleccionar las expresiones correspondientes
    expresion = emocion_a_expresion[emocion]
    shut_mouth_expression = expresion["shut_mouth"]
    open_eyes_expression = expresion["open_eyes"]

    return emocion, shut_mouth_expression, open_eyes_expression

def total_video_generation(respuesta):
        # Configuración de la API de OpenAI
    #openai.api_key = os.getenv("OPENAI_API_KEY")
    openai.api_key = ""  # Coloca tu clave de OpenAI aquí
    #print(openai.api_key )

    path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'faces')) + "/"# ── RUTAS CORREGIDAS PARA ROS2 ──
    try:
        package_share_path = get_package_share_directory('ia_face_software')
    except Exception as e:
        print(f"Error cargando el path de ROS2: {e}")
        package_share_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))

    # Esta ruta apunta a share/ia_face_software/faces/
    path = os.path.join(package_share_path, 'faces') + "/"
    
    # ── Redireccionar los audios temporales a /tmp/ para evitar errores de permisos
    audio_path = "/tmp/audio.mp3"
    trimmed_audio_path = "/tmp/trimmed_audio.mp3"
    output_video_with_audio = "/tmp/output_video_with_audio.mp4"
    # Configuración inicial
    #respuesta = "Estoy emocionado por la fiesta sorpresa."
    audio_path, trimmed_audio_path = path + "audio.mp3", path + "trimmed_audio.mp3"
    image_folder = path + "separate_parts_without_background/mouths"
    points_folder = path + "separate_parts_without_background/mouths_points"
    output_video_with_audio = path + "output_video_with_audio.mp4"
    eyes_folder = path + "separate_parts_without_background/eyes_pairs"
    transiciones_existentes = path + "transitions"

    phoneme_to_image = {
        'i': '1.png', 'u': '2.png', 'o': '3.png', 'j': '3.png',
        'f': '4.png', 'v': '4.png',
        'c': '5.png', 'd': '5.png', 'g': '5.png', 'k': '5.png',
        'n': '5.png', 's': '5.png', 't': '5.png', 'x': '5.png',
        'y': '1.png', 'z': '5.png', 'l': '5.png', 'r': '5.png',
        'a': '6.png', 'e': '6.png', 'q': '6.png',
        'b': '0.png', 'm': '0.png', 'p': '0.png',
        'h': '0.png'  # Asociar espacios con la imagen 13
    }
    phoneme_to_points = {k: v.replace(".png", "_points.txt") for k, v in phoneme_to_image.items()}
    shut_mouth = {"default": "13.png", "groan" : "9.png",
                "pout": "14.png", "meh": "15.png",
                "cat": "16.png", "smile":"1.png" }
    shut_mouth_points = {k: v.replace(".png", "_points.txt") for k, v in shut_mouth.items()}

    open_eyes = {"meh": "1.png", "mad": "2.png",
                "closed": "3.png", "sparkle closed": "4.png",
                "emotional": "5.png", "excited": "6.png",
                "open": "7.png", "dead": "8.png", "cool": "10.png"}
    
    # Mapear emociones a expresiones de ojos y bocas
    emocion_a_expresion = {
    "neutro": {"shut_mouth": "default", "open_eyes": "open"},
    "feliz": {"shut_mouth": "smile", "open_eyes": "excited"},
    "enojado": {"shut_mouth": "groan", "open_eyes": "mad"},
    "desagrado": {"shut_mouth": "meh", "open_eyes": "meh"},
    "desinteres": {"shut_mouth": "meh", "open_eyes": "meh"},
    "tierno": {"shut_mouth": "default", "open_eyes": "emotional"},
    "silly": {"shut_mouth": "cat", "open_eyes": "excited"},

    }


    generar_audio(respuesta, audio_path,velocidad=0.8)
    emocion, shut_mouth_expression, open_eyes_expression = detectar_emocion_y_expresion(respuesta, emocion_a_expresion)
    total_duration = eliminar_silencios(audio_path, trimmed_audio_path)
    #print(emocion, shut_mouth_expression, open_eyes_expression)
    #mouth_emotion = "pout"
    #eye_emotion = "meh"
    sequence, sequence_points, durations = mapear_texto_a_imagenes(total_duration, respuesta, image_folder, points_folder, phoneme_to_image, phoneme_to_points, shut_mouth, shut_mouth_points, mouth_emotion = shut_mouth_expression)
    mouth_video = crear_video_boca(shut_mouth, sequence, sequence_points, durations, image_folder, transiciones_existentes, morph_transition, total_duration / len(respuesta), mouth_emotion = shut_mouth_expression)
    blinking_eyes = crear_bucle_parpadeo(eyes_folder, open_eyes, mouth_video.duration, eye_emotion = open_eyes_expression)
    #print(f"Tiempo de ejecución, antes de video: {time.time() - start_time:.2f} segundos")
    combinar_video_y_audio_en_archivo(mouth_video, blinking_eyes, audio_path, output_video_with_audio)
    print(f"Video con audio y ojos parpadeando: {output_video_with_audio}")

    return combinar_video_y_audio(mouth_video, blinking_eyes, audio_path)

  


if __name__ == "__main__":

    # Inicia el temporizador
    start_time = time.time()

    # Configuración inicial
    respuesta = "si estoy muy bien gracias por preguntar, dime en que puedo ayudarte hoy!"
    total_video_generation(respuesta)

    # Termina el temporizador y calcula la duración
    elapsed_time = time.time() - start_time
    print(f"Tiempo de ejecución: {elapsed_time:.2f} segundos")