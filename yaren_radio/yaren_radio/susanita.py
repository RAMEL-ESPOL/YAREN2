#!/usr/bin/env python3
import vlc
import sys
import tkinter as tk
import os

def play_video():
    # 1. Crear ventana principal a pantalla completa
    root = tk.Tk()
    root.attributes('-fullscreen', True)
    root.configure(background='black')
    root.config(cursor="none")  # Ocultar el cursor para no distraer

    # 2. Función unificada para cerrar todo
    def close_app(event=None):
        print("\nEntrada detectada (Touch/Clic/Tecla). Cerrando video...")
        player.stop()
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
    # --no-mouse-events y --no-keyboard-events hacen que VLC no se "robe" el touch,
    # permitiendo que pase a la ventana de Tkinter que lo cerrará.
    vlc_instance = vlc.Instance("--no-xlib", "--no-mouse-events", "--no-keyboard-events", "--quiet")
    
    inner_player = vlc_instance.media_player_new()
    player = vlc_instance.media_list_player_new()
    player.set_media_player(inner_player)

    # 6. Cargar el video
    workspace_dir = os.getcwd()

    ruta_video = os.path.join(
        workspace_dir, 
        'src', 'YAREN2', 'yaren_radio', 'videos', 
        'vidssave.com Susanita _ La Granja de Zenón 6 720P.mp4'
    )
    media_list = vlc_instance.media_list_new([ruta_video])
    player.set_media_list(media_list)
    player.set_playback_mode(vlc.PlaybackMode.loop)

    # 7. Empotrar VLC en la ventana de Tkinter (Requerido para Linux/Ubuntu)
    # winfo_id() obtiene el ID de la ventana para que VLC dibuje directamente ahí
    window_id = video_frame.winfo_id()
    inner_player.set_xwindow(window_id)

    # 8. Reproducir e iniciar el bucle de la interfaz gráfica
    player.play()
    print("Reproduciendo video. Toca la pantalla o haz clic para salir.")
    
    try:
        # mainloop() reemplaza tu 'while not cerrar_programa'
        root.mainloop() 
    except KeyboardInterrupt:
        close_app()

if __name__ == '__main__':
    play_video()