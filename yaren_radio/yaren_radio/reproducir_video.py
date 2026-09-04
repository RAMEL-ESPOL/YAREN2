#!/usr/bin/env python3
import os
import sys
import time
import signal  # <-- NUEVO: Para interceptar el "kill" del menú C++
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
    root.config(cursor="none")

    # 2. Función de limpieza (ahora acepta argumentos por si viene de 'signal')
    def close_app(*args):
        print("\nCerrando video y liberando memoria gráfica de la Jetson...")
        try:
            player.stop()
            inner_player.release()
            player.release()
            vlc_instance.release()
        except Exception:
            pass
        try:
            root.destroy()
        except Exception:
            pass
        sys.exit(0)

    # 3. INTERCEPTAR EL COMANDO KILL DE C++
    # Cuando C++ haga 'kill -15', Python ejecutará close_app() en lugar de morir de golpe
    signal.signal(signal.SIGTERM, close_app)
    signal.signal(signal.SIGINT, close_app)

    # 4. Vincular el clic izquierdo/toque (<Button-1>) y cualquier tecla para cerrar
    root.bind('<Button-1>', close_app)
    root.bind('<Any-KeyPress>', close_app)

    video_frame = tk.Frame(root, bg='black')
    video_frame.pack(fill=tk.BOTH, expand=True)
    video_frame.bind('<Button-1>', close_app)

    # --- FIX RENDERING X11 ---
    # Fuerza a Tkinter a crear los gráficos de la ventana ANTES de incrustar VLC.
    # Evita el pantallazo negro al abrir rápido.
    root.update()
    # -------------------------

    # 5. Configurar VLC
    vlc_instance = vlc.Instance(
        "--no-xlib", 
        "--no-mouse-events", 
        "--no-keyboard-events", 
        "--quiet", 
        "--aout=pulse",
        "--vout=x11",         # Fuerza la salida gráfica estándar segura
        "--avcodec-hw=none"   # Apaga la decodificación por hardware para evitar fugas de VRAM
    )
    
    inner_player = vlc_instance.media_player_new()
    player = vlc_instance.media_list_player_new()
    player.set_media_player(inner_player)

    # 6. Validar y cargar el video
    if not os.path.exists(ruta_video):
        sys.exit(1)

    media_list = vlc_instance.media_list_new([ruta_video])
    player.set_media_list(media_list)
    player.set_playback_mode(vlc.PlaybackMode.loop)

    # 7. Empotrar VLC en la ventana de Tkinter
    window_id = video_frame.winfo_id()
    inner_player.set_xwindow(window_id)

    # 8. Reproducir
    player.play()
    
    time.sleep(0.1) 
    inner_player.audio_set_mute(False)
    inner_player.audio_set_volume(100)
    
    try:
        root.mainloop() 
    except Exception:
        close_app()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(1)
    
    play_video(sys.argv[1])
