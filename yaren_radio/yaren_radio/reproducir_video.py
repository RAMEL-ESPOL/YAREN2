#!/usr/bin/env python3
import os
import sys
vlc = None
try:
    import vlc
except ImportError:
    pass
import tkinter as tk

def play_video(ruta_video):
    if vlc is None:
        print("Error: La librería python-vlc no está instalada.")
        sys.exit(1)

    # 1. Crear ventana principal a pantalla completa
    root = tk.Tk()
    root.attributes('-fullscreen', True)
    root.configure(background='black')
    root.config(cursor="none")  # Ocultar el cursor para no distraer

    # 2. Función unificada para cerrar todo de forma segura
    def close_app(event=None):
        print("\nEntrada detectada (Touch/Clic/Tecla). Cerrando video...")
        try:
            player.stop()
        except Exception:
            pass
        root.destroy()
        sys.exit(0)

    # 3. Vincular el clic izquierdo/toque (<Button-1>) y cualquier tecla para cerrar
    root.bind('<Button-1>', close_app)
    root.bind('<Any-KeyPress>', close_app)

    # 4. Crear el frame (marco) contenedor donde se proyectará el video
    video_frame = tk.Frame(root, bg='black')
    video_frame.pack(fill=tk.BOTH, expand=True)
    video_frame.bind('<Button-1>', close_app)

    # 5. Configurar VLC
    # --no-mouse-events y --no-keyboard-events evitan que VLC robe el evento táctil
    vlc_instance = vlc.Instance("--no-xlib", "--no-mouse-events", "--no-keyboard-events", "--quiet")
    
    inner_player = vlc_instance.media_player_new()
    player = vlc_instance.media_list_player_new()
    player.set_media_player(inner_player)

    # 6. Validar y cargar el video recibido por argumento
    if not os.path.exists(ruta_video):
        print(f"Error crítico: No se encontró el archivo de video en: {ruta_video}")
        sys.exit(1)

    media_list = vlc_instance.media_list_new([ruta_video])
    player.set_media_list(media_list)
    player.set_playback_mode(vlc.PlaybackMode.loop)

    # 7. Empotrar VLC en la ventana de Tkinter (Requerido para Linux/Ubuntu)
    window_id = video_frame.winfo_id()
    inner_player.set_xwindow(window_id)

    # 8. Reproducir e iniciar el bucle de la interfaz gráfica
    player.play()
    print(f"Reproduciendo video: {ruta_video}. Toca la pantalla o haz clic para salir.")
    
    try:
        root.mainloop() 
    except KeyboardInterrupt:
        close_app()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Uso correcto: python3 reproducir_video.py <ruta_absoluta_o_relativa_al_video>")
        sys.exit(1)
    
    play_video(sys.argv[1])