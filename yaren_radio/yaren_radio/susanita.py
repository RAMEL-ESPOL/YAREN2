#!/usr/bin/env python3
import time
import vlc
import sys
from pynput import mouse

cerrar_programa = False

def on_click(x, y, button, pressed):
    """Función que detecta clics globales del ratón."""
    global cerrar_programa
    # Filtrar solo cuando se presiona el botón izquierdo
    if pressed and button == mouse.Button.left:
        print("\nClic izquierdo detectado. Cerrando video y terminando proceso...")
        cerrar_programa = True
        return False  # Detiene el listener de pynput para no consumir recursos

def play_video():
    global cerrar_programa
    try:
        # 1. Definir los argumentos para pantalla completa y sin distracciones
        vlc_args = [
            "--fullscreen",                   
            "--no-video-title-show", 
            "--mouse-hide-timeout=0",
            "--video-on-top",
            "--no-mouse-events"           
        ]
        
        # 2. Inicializar VLC
        vlc_instance = vlc.Instance(*vlc_args)
        player = vlc_instance.media_list_player_new()
 
         # CREAMOS PRIMERO EL REPRODUCTOR INDIVIDUAL (Control directo de ventana)
        inner_player = vlc_instance.media_player_new()
        inner_player.set_fullscreen(True) # Forzado inmediato antes de cargar archivos

        # CREAMOS EL REPRODUCTOR DE LISTA Y LE VINCULAMOS EL REPRODUCTOR BASE
        player = vlc_instance.media_list_player_new()
        player.set_media_player(inner_player) # <--- Enlace crucial
        
        # 3. Cargar el video
        ruta_video = "/home/roberto/robotis_ws/src/YAREN2/yaren_radio/videos/vidssave.com Susanita _ La Granja de Zenón 6 720P.mp4"
        media_list = vlc_instance.media_list_new([ruta_video])
        player.set_media_list(media_list)
        
        # 4. Configurar bucle infinito
        player.set_playback_mode(vlc.PlaybackMode.loop)
        
        # 5. Forzar el Fullscreen
        inner_player = player.get_media_player()
        inner_player.set_fullscreen(True)
        
        # --- INICIAR EL DETECTOR DE CLICS EN SEGUNDO PLANO ---
        listener = mouse.Listener(on_click=on_click)
        listener.start()
        # -----------------------------------------------------

        # Reproducir
        player.play()
        
        print(" Reproduciendo video. Haz clic izquierdo en la pantalla para salir.")
    
        # 6. Mantener el programa abierto hasta que se haga clic
        while not cerrar_programa:
            time.sleep(0.2)    
            
    except KeyboardInterrupt:
        print("\nDeteniendo el video por terminal...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Detener la reproducción y limpiar instancias de memoria
        if 'player' in locals() and player is not None:
            player.stop()
        print("Proceso completamente terminado.")
        sys.exit(0)

if __name__ == '__main__':
     play_video()