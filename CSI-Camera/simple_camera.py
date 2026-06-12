# MIT License
# Copyright (c) 2019-2022 JetsonHacks
# Using a CSI camera (such as the Raspberry Pi Version 2) connected to a
# NVIDIA Jetson Nano Developer Kit using OpenCV
# Drivers for the camera and OpenCV are included in the base image
import os
import cv2

# Variable global para registrar si se hizo clic en la pantalla
window_clicked = False


def mouse_event_callback(event, x, y, flags, param):
    global window_clicked
    if event == cv2.EVENT_LBUTTONDOWN:
        window_clicked = True


def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=6,
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


def is_usb_camera(device_id: int) -> bool:
    """Verifica via sysfs si /dev/video{device_id} corresponde a hardware USB real."""
    device_path = f"/dev/video{device_id}"
    if not os.path.exists(device_path):
        return False
    sys_path = f"/sys/class/video4linux/video{device_id}/device/driver"
    try:
        driver = os.path.realpath(sys_path)
        return "usb" in driver.lower()
    except Exception:
        return False


def open_camera(usb_device_id=0, capture_width=1920, capture_height=1080, framerate=30):
    """Intenta abrir primero la camara USB (verificada via sysfs), y si no existe
    o no entrega frames, cae a la camara CSI (nvarguscamerasrc)."""

    # 1) Verificar si hay hardware USB real antes de intentar abrir
    if is_usb_camera(usb_device_id):
        print(f"Hardware USB detectado en /dev/video{usb_device_id}. Intentando abrir...")
        cap = cv2.VideoCapture(usb_device_id, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  capture_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, capture_height)
            cap.set(cv2.CAP_PROP_FPS,          framerate)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            ok, _ = cap.read()
            if ok:
                print("Camara USB detectada y en uso.")
                return cap, "USB Camera"
            cap.release()
            print("VideoCapture USB abre pero no entrega frames. Descartando.")
    else:
        print(f"No se detectó hardware USB en /dev/video{usb_device_id}.")

    # 2) Caer a CSI
    print("Probando CSI (nvarguscamerasrc)...")
    pipeline = gstreamer_pipeline(flip_method=6)
    print(pipeline)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if cap.isOpened():
        print("Camara CSI detectada y en uso.")
        return cap, "CSI Camera"

    return cap, None


def show_camera():
    global window_clicked

    video_capture, window_title = open_camera()

    if video_capture is not None and video_capture.isOpened():
        try:
            cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
            cv2.setWindowProperty(window_title, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            cv2.setMouseCallback(window_title, mouse_event_callback)

            while True:
                ret_val, frame = video_capture.read()
                if not ret_val:
                    print("Error: no se pudo leer el frame de la camara.")
                    break

                if window_title == "USB Camera":
                    frame = cv2.flip(frame, 0)
                # CSI ya viene rotada por flip_method=6, no hace falta flip adicional

                if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                    cv2.imshow(window_title, frame)
                else:
                    break

                keyCode = cv2.waitKey(10) & 0xFF
                if keyCode == 27 or keyCode == ord('q') or window_clicked:
                    break
        finally:
            video_capture.release()
            cv2.destroyAllWindows()
            window_clicked = False
    else:
        print("Error: Unable to open camera (ni USB ni CSI disponibles)")


if __name__ == "__main__":
    show_camera()
