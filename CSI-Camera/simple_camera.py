# MIT License
# Copyright (c) 2019-2022 JetsonHacks
# Using a CSI camera (such as the Raspberry Pi Version 2) connected to a
# NVIDIA Jetson Nano Developer Kit using OpenCV
# Drivers for the camera and OpenCV are included in the base image
import cv2

# Variable global para registrar si se hizo clic en la pantalla
window_clicked = False

# Función que captura los eventos del ratón
def mouse_event_callback(event, x, y, flags, param):
    global window_clicked
    # Si detecta un clic izquierdo del ratón, actualiza la variable
    if event == cv2.EVENT_LBUTTONDOWN:
        window_clicked = True


"""
gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
Flip the image by setting the flip_method (most common values: 0 and 2)
display_width and display_height determine the size of each camera pane in the window on the screen
Default 1920x1080 displayd in a 1/4 size window
"""
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=2,  # <-- MODIFICADO: 2 rota 180 grados (pone la imagen al revés)
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


def open_camera(usb_device_id=0, capture_width=1920, capture_height=1080, framerate=30):
    """Intenta abrir primero la camara USB, y si no existe/no entrega frames,
    cae a la camara CSI (nvarguscamerasrc)."""

    # 1) Intentar USB
    print(f"Buscando camara USB en /dev/video{usb_device_id} ...")
    cap = cv2.VideoCapture(usb_device_id, cv2.CAP_V4L2)
    if cap.isOpened():
        # MJPG suele dar muchos mas FPS que el formato crudo por defecto en USB
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  capture_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, capture_height)
        cap.set(cv2.CAP_PROP_FPS,          framerate)
        # Reducir el buffer interno ayuda a evitar lag/baja tasa percibida
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        ok, _ = cap.read()
        if ok:
            print("Camara USB detectada y en uso.")
            return cap, "USB Camera"
        cap.release()

    # 2) Caer a CSI
    print("Camara USB no disponible, probando CSI (nvarguscamerasrc)...")
    pipeline = gstreamer_pipeline(flip_method=2)
    print(pipeline)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if cap.isOpened():
        print("Camara CSI detectada y en uso.")
        return cap, "CSI Camera"

    return cap, None


def show_camera():
    global window_clicked

    video_capture, window_title = open_camera()

    if video_capture.isOpened():
        try:
            # --- MODIFICACIÓN PARA PANTALLA COMPLETA ---
            # Primero se crea la ventana permitiendo que cambie de tamaño (WINDOW_NORMAL)
            cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
            # Luego se fuerza a que use la propiedad de pantalla completa
            cv2.setWindowProperty(window_title, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            # --------------------------------------------

            # --- MODIFICACIÓN PARA CERRAR CON CLIC ---
            # Vinculamos la función de eventos del ratón a nuestra ventana
            cv2.setMouseCallback(window_title, mouse_event_callback)
            # -----------------------------------------

            while True:
                ret_val, frame = video_capture.read()
                if not ret_val:
                    print("Error: no se pudo leer el frame de la camara.")
                    break

                if window_title == "USB Camera":
                    # Voltear horizontalmente (efecto espejo) y rotar 180 grados
                    frame = cv2.flip(frame, 0)  # -1 = flip horizontal + vertical (180°)
                else:
                    # La CSI ya viene rotada por flip_method=2 en el pipeline;
                    # aqui solo agregamos el efecto espejo horizontal
                    pass
                # Check to see if the user closed the window
                if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                    cv2.imshow(window_title, frame)
                else:
                    break

                keyCode = cv2.waitKey(10) & 0xFF
                # Stop the program on the ESC key, 'q', o si hubo un CLIC
                if keyCode == 27 or keyCode == ord('q') or window_clicked:
                    break
        finally:
            video_capture.release()
            cv2.destroyAllWindows()
            # Reiniciamos la variable por si la función se vuelve a llamar
            window_clicked = False
    else:
        print("Error: Unable to open camera (ni USB ni CSI disponibles)")


if __name__ == "__main__":
    show_camera()