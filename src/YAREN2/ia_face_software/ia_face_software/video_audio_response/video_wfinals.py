import os
import sys
from moviepy.editor import CompositeVideoClip, AudioFileClip

# Añade el directorio src a sys.path
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
from video_generation import total_video_generation


def guardar_video(final_video, output_dir="video_evidencias", filename="final_video.mp4"):
    # Crear el directorio si no existe
    os.makedirs(output_dir, exist_ok=True)
    
    # Ruta completa del archivo de salida
    output_path = os.path.join(output_dir, filename)
    
    # Guardar el video en formato MP4
    final_video.write_videofile(output_path, codec="libx264", audio_codec="aac", fps=24)

    return output_path  # Retorna la ruta del archivo guardado



respuesta ="entiendo que estes tristes, yo te voy a ayudar"

path = os.path.abspath(os.path.join(os.path.dirname(__file__)))

final_video = total_video_generation(respuesta)
ruta_guardada = guardar_video(final_video,output_dir=path, filename=respuesta+".mp4")
print(f"Video guardado en: {ruta_guardada}")