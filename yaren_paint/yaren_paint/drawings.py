"""Libreria de dibujos de YAREN PAINT.

Cada forma es (tipo, parametros, nombre) en un espacio de diseno de 800x600:
  circle  (cx, cy, r)
  ellipse (cx, cy, rx, ry)
  poly    [(x, y), ...]   cerrado, se puede colorear
  line    [(x, y), ...]   abierto, solo contorno
El orden de la lista es el orden de capas y de los pasos guiados.
"""
import math

DESIGN_W, DESIGN_H = 800, 600


def _rays(cx, cy, r1, r2, n):
    out = []
    for i in range(n):
        a = 2 * math.pi * i / n
        out.append(("line", [(cx + r1 * math.cos(a), cy + r1 * math.sin(a)),
                             (cx + r2 * math.cos(a), cy + r2 * math.sin(a))],
                    "un rayo del sol"))
    return out


def _arc(cx, cy, r, a0, a1, n=12):
    return [(cx + r * math.cos(a0 + (a1 - a0) * i / n),
             cy + r * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]


def _petals(cx, cy, dist, r, n):
    out = []
    for i in range(n):
        a = 2 * math.pi * i / n
        out.append(("circle", (cx + dist * math.cos(a), cy + dist * math.sin(a), r),
                    "un pétalo"))
    return out


DRAWINGS = [
    {"id": "casa", "label": "Casa", "title": "una casa", "shapes": [
        ("poly", [(250, 300), (550, 300), (550, 500), (250, 500)], "las paredes"),
        ("poly", [(220, 300), (400, 160), (580, 300)], "el techo"),
        ("poly", [(360, 500), (360, 390), (440, 390), (440, 500)], "la puerta"),
        ("poly", [(285, 335), (345, 335), (345, 395), (285, 395)], "una ventana"),
        ("poly", [(455, 335), (515, 335), (515, 395), (455, 395)], "la otra ventana"),
        ("circle", (680, 110, 50), "el sol"),
        ("line", [(100, 500), (700, 500)], "el pasto"),
    ]},
    {"id": "sol", "label": "Sol", "title": "un sol feliz",
     "shapes": ([("circle", (400, 300, 100), "la cara del sol")]
                + _rays(400, 300, 118, 185, 8)
                + [("circle", (365, 275, 10), "un ojo"),
                   ("circle", (435, 275, 10), "el otro ojo"),
                   ("line", _arc(400, 315, 50, math.pi / 9, 8 * math.pi / 9), "la sonrisa")])},
    {"id": "flor", "label": "Flor", "title": "una flor",
     "shapes": ([("poly", [(392, 320), (408, 320), (408, 545), (392, 545)], "el tallo"),
                 ("poly", [(408, 470), (495, 410), (525, 450), (460, 492)], "una hoja")]
                + _petals(400, 230, 85, 55, 6)
                + [("circle", (400, 230, 42), "el centro de la flor")])},
    {"id": "pez", "label": "Pez", "title": "un pez", "shapes": [
        ("poly", [(540, 300), (670, 210), (670, 390)], "la cola"),
        ("poly", [(330, 205), (390, 130), (450, 215)], "la aleta"),
        ("ellipse", (380, 300, 170, 100), "el cuerpo"),
        ("circle", (290, 270, 14), "el ojo"),
        ("line", [(215, 315), (255, 330)], "la boca"),
        ("circle", (150, 180, 16), "una burbuja"),
        ("circle", (120, 120, 11), "otra burbuja"),
        ("circle", (165, 90, 8), "una burbuja pequeña"),
    ]},
    {"id": "arbol", "label": "Árbol", "title": "un árbol", "shapes": [
        ("poly", [(365, 360), (435, 360), (435, 545), (365, 545)], "el tronco"),
        ("circle", (310, 300, 85), "la copa de la izquierda"),
        ("circle", (490, 300, 85), "la copa de la derecha"),
        ("circle", (400, 215, 110), "la copa de arriba"),
        ("circle", (350, 250, 14), "una manzana"),
        ("circle", (450, 290, 14), "otra manzana"),
        ("circle", (400, 160, 14), "una manzana más"),
        ("line", [(150, 545), (650, 545)], "el pasto"),
    ]},
    {"id": "gato", "label": "Gato", "title": "un gato", "shapes": [
        ("poly", [(285, 235), (290, 110), (385, 190)], "la oreja izquierda"),
        ("poly", [(515, 235), (510, 110), (415, 190)], "la oreja derecha"),
        ("circle", (400, 310, 130), "la cabeza"),
        ("circle", (350, 285, 16), "un ojo"),
        ("circle", (450, 285, 16), "el otro ojo"),
        ("poly", [(385, 330), (415, 330), (400, 348)], "la nariz"),
        ("line", [(300, 335), (200, 320)], "un bigote"),
        ("line", [(300, 355), (200, 365)], "otro bigote"),
        ("line", [(500, 335), (600, 320)], "un bigote"),
        ("line", [(500, 355), (600, 365)], "otro bigote"),
    ]},
]


# ---------- utilidades geometricas ----------
def viewport(rect):
    """(escala, offset_x, offset_y) para encajar 800x600 dentro de rect."""
    s = min(rect.w / DESIGN_W, rect.h / DESIGN_H)
    return s, rect.x + (rect.w - DESIGN_W * s) / 2, rect.y + (rect.h - DESIGN_H * s) / 2


def _ti(vp, p):
    s, ox, oy = vp
    return int(ox + p[0] * s), int(oy + p[1] * s)


def to_screen(vp, p):
    s, ox, oy = vp
    return ox + p[0] * s, oy + p[1] * s


def to_design(vp, p):
    s, ox, oy = vp
    return (p[0] - ox) / s, (p[1] - oy) / s


def is_fillable(shape):
    return shape[0] != "line"


def draw_shape(surf, shape, vp, color, width):
    """width=0 rellena (solo formas cerradas)."""
    kind, p, _ = shape
    s = vp[0]
    if kind == "circle":
        pygame.draw.circle(surf, color, _ti(vp, (p[0], p[1])), max(1, int(p[2] * s)), width)
    elif kind == "ellipse":
        rect = pygame.Rect(0, 0, max(2, int(2 * p[2] * s)), max(2, int(2 * p[3] * s)))
        rect.center = _ti(vp, (p[0], p[1]))
        pygame.draw.ellipse(surf, color, rect, width)
    elif kind == "poly":
        pygame.draw.polygon(surf, color, [_ti(vp, q) for q in p], width)
    elif kind == "line" and width > 0:
        pygame.draw.lines(surf, color, False, [_ti(vp, q) for q in p], width)


def hit_test(shape, q):
    kind, p, _ = shape
    x, y = q
    if kind == "circle":
        return (x - p[0]) ** 2 + (y - p[1]) ** 2 <= p[2] ** 2
    if kind == "ellipse":
        return ((x - p[0]) / p[2]) ** 2 + ((y - p[1]) / p[3]) ** 2 <= 1
    if kind == "poly":
        inside = False
        j = len(p) - 1
        for i in range(len(p)):
            xi, yi = p[i]
            xj, yj = p[j]
            if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
                inside = not inside
            j = i
        return inside
    return False


def _along(pts, step, closed):
    out = []
    seq = list(pts) + [pts[0]] if closed else list(pts)
    for a, b in zip(seq, seq[1:]):
        n = max(1, int(math.dist(a, b) // step))
        for i in range(n):
            t = i / n
            out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    if not closed:
        out.append(pts[-1])
    return out


def outline_points(shape, step=14):
    """Puntos del contorno (espacio de diseno) que el jugador debe trazar."""
    kind, p, _ = shape
    if kind == "circle":
        n = max(8, int(2 * math.pi * p[2] / step))
        return [(p[0] + p[2] * math.cos(2 * math.pi * i / n),
                 p[1] + p[2] * math.sin(2 * math.pi * i / n)) for i in range(n)]
    if kind == "ellipse":
        n = max(12, int(math.pi * (p[2] + p[3]) / step))
        return [(p[0] + p[2] * math.cos(2 * math.pi * i / n),
                 p[1] + p[3] * math.sin(2 * math.pi * i / n)) for i in range(n)]
    if kind == "poly":
        return _along(p, step, True)
    return _along(p, step, False)
