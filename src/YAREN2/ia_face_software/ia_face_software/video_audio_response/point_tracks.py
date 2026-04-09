import cv2
import numpy as np

# Lista del orden de los puntos que se deben seleccionar (1, 13, 7, 19, ...)
points_order = [1, 13, 7, 19, 4, 10, 16, 22, 2, 3, 5, 6, 8, 9, 11, 12, 14, 15, 17, 18, 20, 21, 23, 24]

# Variable global para almacenar los puntos
points = {}

# Umbral de proximidad para eliminar un punto (en píxeles)
proximity_threshold = 20

# Función de callback para capturar los puntos al hacer clic en la imagen
def click_event(event, x, y, flags, param):
    global points
    if event == cv2.EVENT_LBUTTONDOWN:  # Clic izquierdo, agregar punto
        if len(points) < len(points_order):
            point_number = points_order[len(points)]  # Obtener el número del siguiente punto
            points[point_number] = (x, y)  # Añadir el punto con sus coordenadas
            cv2.circle(image, (x, y), 5, (0, 0, 255), -1)
            cv2.putText(image, str(point_number), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow("Image", image)
            print(f"Punto {point_number} añadido en ({x}, {y})")
        else:
            print("Se han añadido todos los puntos.")


    elif event == cv2.EVENT_RBUTTONDOWN:  # Clic derecho, eliminar punto cercano
        for point in list(points.values()):
            if np.sqrt((x - point[0])**2 + (y - point[1])**2) < proximity_threshold:
                # Eliminar el primer punto cercano
                points = {key: val for key, val in points.items() if val != point}
                break
        # Redibujar la imagen y los puntos restantes
        image_copy = image.copy()
        for point_number, point in points.items():
            cv2.circle(image_copy, point, 5, (0, 0, 255), -1)
            cv2.putText(image_copy, str(point_number), (point[0], point[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("Image", image_copy)

# Cargar la imagen
image_path = 'caritas/partes separadas sin fondo/bocas/0.png'  # Cambia la ruta a tu imagen
image = cv2.imread(image_path)

# Mostrar la imagen y esperar clics
cv2.imshow("Image", image)
cv2.setMouseCallback("Image", click_event)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Asegurarse de que todos los puntos estén en el diccionario, incluso los que no se han tomado
for point_number in points_order:
    if point_number not in points:
        points[point_number] = (0, 0)  # Si no se ha tomado el punto, asignar (0, 0)

# Ordenar los puntos en el orden correcto (de 1 a 24)
ordered_points = [points[i] for i in range(1, 25)]

# Mostrar los puntos seleccionados y ordenados
print("Puntos seleccionados (en el orden correcto):", ordered_points)

# Generar el path (una lista de puntos que pueden usarse como una ruta)
path = ordered_points  # Ya tenemos los puntos ordenados

# Guardar los puntos en un archivo txt
txt_file = image_path.split('/')[-1].split('.')[0] + "_points.txt"
with open(txt_file, 'w') as f:
    for point in path:
        f.write(f"{point[0]},{point[1]}\n")

print(f"Puntos guardados en: {txt_file}")
