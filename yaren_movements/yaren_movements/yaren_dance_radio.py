#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
import random  # Importamos random para la elección inicial

TIME_SCALE = 1.0     # >1 = mas lento, <1 = mas rapido
START_DELAY = 1.2    # s para llegar a la 1ra pose desde donde este


def _smooth_points_base(routine, scale=TIME_SCALE, start_delay=START_DELAY, min_dt=0.1):
    """routine = [(pose, t)]; t = tiempo para llegar a esa pose desde la anterior.
    Devuelve (puntos, duracion_total) con velocidades continuas: el controlador
    hace spline cubica y NO frena en cada pose (si el joint invierte direccion
    o la pose se repite, velocidad 0 para evitar rebotes)."""
    import math as _math
    P = [[float(x) for x in pos] for pos, _ in routine]
    n, J = len(P), len(P[0])
    T, acc = [], 0.0
    for i, (_, t) in enumerate(routine):
        dt = max(float(t) * scale, min_dt)
        if i == 0:
            dt = max(dt, start_delay)
        acc += dt
        T.append(acc)
    points = []
    for i in range(n):
        vel = [0.0] * J
        if 0 < i < n - 1:
            for j in range(J):
                s0 = (P[i][j] - P[i - 1][j]) / (T[i] - T[i - 1])
                s1 = (P[i + 1][j] - P[i][j]) / (T[i + 1] - T[i])
                if s0 * s1 > 0:
                    v = (s0 + s1) / 2.0
                    lim = 3.0 * min(abs(s0), abs(s1))
                    vel[j] = _math.copysign(min(abs(v), lim), v)
        pt = JointTrajectoryPoint()
        pt.positions = P[i]
        pt.velocities = vel
        pt.time_from_start = Duration(sec=int(T[i]), nanosec=int((T[i] % 1) * 1e9))
        points.append(pt)
    return points, T[-1]

BASE_POSITION = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5]


def smooth_points(routine, scale=TIME_SCALE, start_delay=START_DELAY, min_dt=0.1, step=0.2):
    """Curva Hermite muestreada cada 'step' s con velocidad analitica en cada punto."""
    base, total = _smooth_points_base(routine, scale, start_delay, min_dt)

    def tt(pt):
        return pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9

    out = [base[0]]
    for a, b in zip(base[:-1], base[1:]):
        t0, t1 = tt(a), tt(b)
        h = max(t1 - t0, 1e-3)
        n = max(1, int(round(h / step)))
        for k in range(1, n):
            u = k / n
            u2, u3 = u * u, u * u * u
            h00 = 2 * u3 - 3 * u2 + 1
            h10 = u3 - 2 * u2 + u
            h01 = -2 * u3 + 3 * u2
            h11 = u3 - u2
            d00 = (6 * u2 - 6 * u) / h
            d10 = 3 * u2 - 4 * u + 1
            d01 = (-6 * u2 + 6 * u) / h
            d11 = 3 * u2 - 2 * u
            pa, pb = list(a.positions), list(b.positions)
            va, vb = list(a.velocities), list(b.velocities)
            pt = JointTrajectoryPoint()
            pt.positions = [h00 * pa[j] + h10 * h * va[j] + h01 * pb[j] + h11 * h * vb[j]
                            for j in range(len(pa))]
            pt.velocities = [d00 * pa[j] + d10 * va[j] + d01 * pb[j] + d11 * vb[j]
                             for j in range(len(pa))]
            t = t0 + u * h
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            out.append(pt)
        out.append(b)
    return out, total


# ======================================================================
#  CATALOGO DE RUTINAS (patch_rutinas)
# ======================================================================
import math as _mth
import os as _os

VEL_MAX = 2.0   # rad/s maximo entre poses (si un tramo es mas rapido, se alarga)

# brazo = (hombro, abduccion, rotacion, codo). Derecho: J5,J6,J7=-rot,J8. Izquierdo: J9=-hombro,J10,J11=+rot,J12
REST = (0.0, 0.0, 0.0, 0.5)
UP = (3.0, 0.0, 3.0, 0.0)
FWD = (1.7, 0.0, 3.0, 0.0)
SIDE = (1.7, 1.0, 3.0, 0.0)
V = (3.0, 1.0, 3.0, 0.0)
MID = (1.5, 0.0, 3.0, 0.0)
HALF = (1.5, 0.0, 0.0, 0.0)
HALFB = (1.5, 0.0, 0.0, 1.0)
T = (1.5, 1.0, 0.0, 0.0)
ARCH = (3.0, 0.5, 3.0, 0.0)
HUG = (1.5, 0.0, 1.5, 1.0)
WING_HI = (2.4, 1.0, 3.0, 0.0)
WING_LO = (1.0, 1.0, 3.0, 0.2)
WING_A = (0.4, 1.0, 0.0, 1.3)
WING_B = (0.4, 1.0, 0.0, 0.3)
GUARD = (1.0, 0.0, 0.0, 1.3)
HIP = (0.4, 1.0, 0.0, 1.3)
PUNCH = (1.7, 0.0, 3.0, 0.0)
DISCO_LO = (0.4, 0.8, 3.0, 0.5)
ROW_BACK = (0.5, 0.0, 0.0, 1.0)


def pose(r=REST, l=REST, j1=0.0, j3=0.0, j4=0.0):
    return [j1, 0.0, j3, j4,
            r[0], r[1], -r[2], r[3],
            -l[0], l[1], l[2], l[3]]


BASE_POSE = pose(REST, REST)


def _desde_gui(items, t_def=1.0):
    K = _mth.pi / 5
    OFF = _mth.pi - 1.75
    out = []
    for it in items:
        pos, t = it if isinstance(it, tuple) else (it, t_def)
        out.append(([round(v * K + (OFF if i in (7, 11) else 0.0), 4)
                     for i, v in enumerate(pos)], t))
    return out


RET_VMAX = 1.0   # rad/s maximo en el regreso a base
RET_MIN_T = 2.5  # s minimos para el ultimo tramo hacia la base


def preparar(rutina, vmax=None):
    """Limita hombros a +-3.0, alarga tramos rapidos y regresa a base en 2 pasos lentos."""
    vmax = vmax or VEL_MAX

    def clamp(pos):
        pos = [float(x) for x in pos]
        for i in (4, 6, 8, 10):
            pos[i] = max(-3.0, min(3.0, pos[i]))
        return pos

    # paso 1: cerrar abduccion y recoger codos, hombro quieto
    ult = clamp(rutina[-1][0])
    inter = list(ult)
    inter[5] = 0.0
    inter[9] = 0.0
    inter[7] = 0.5
    inter[11] = 0.5

    lista = [(clamp(p), float(t), vmax) for p, t in rutina]
    lista.append((inter, 1.5, RET_VMAX))
    lista.append((list(BASE_POSE), RET_MIN_T, RET_VMAX))

    out, prev = [], None
    for pos, t, vm in lista:
        if prev is not None:
            d = max(abs(a - b) for a, b in zip(pos, prev))
            t = max(t, d / vm)
        out.append((pos, t))
        prev = pos
    return out


# ---------------------------- rutinas nuevas ----------------------------
def _ola():
    r = [(pose(REST, REST), 2.0)]
    for _ in range(2):
        r += [(pose(MID, REST, j1=0.3), 1.1), (pose(UP, MID, j1=0.3), 1.1),
              (pose(MID, UP, j1=-0.3), 1.1), (pose(REST, MID, j1=-0.3), 1.1)]
    r += [(pose(V, V), 1.4), (pose(SIDE, SIDE), 1.2), (pose(V, V), 1.2)]
    return r


def _robot():
    r = [(pose(REST, REST), 1.5)]
    pasos = [(FWD, REST, 0.4), (FWD, FWD, -0.4), (SIDE, FWD, 0.4), (SIDE, SIDE, -0.4),
             (UP, SIDE, 0.4), (UP, UP, -0.4), (HALFB, UP, 0.4), (HALFB, HALFB, -0.4)]
    for a, b, cab in pasos:
        r.append((pose(a, b, j3=cab), 1.0))
        r.append((pose(a, b, j3=cab), 0.4))   # pausa "mecanica"
    return r


def _pajaro():
    r = [(pose(SIDE, SIDE), 1.5)]
    for _ in range(4):
        r += [(pose(WING_HI, WING_HI), 0.9), (pose(WING_LO, WING_LO), 0.9)]
    r += [(pose(V, V), 1.4), (pose(V, V, j1=0.4, j3=0.3), 1.5),
          (pose(V, V, j1=-0.4, j3=-0.3), 2.0), (pose(V, V), 1.5)]
    for _ in range(2):
        r += [(pose(WING_HI, WING_HI), 0.9), (pose(WING_LO, WING_LO), 0.9)]
    return r


def _remo():
    r = [(pose(HALFB, HALFB), 1.5)]
    for _ in range(4):
        r += [(pose(FWD, ROW_BACK, j1=0.35), 1.3), (pose(ROW_BACK, FWD, j1=-0.35), 1.3)]
    r.append((pose(HALFB, HALFB), 1.2))
    return r


def _superheroe():
    r = [(pose(REST, HIP), 1.5), (pose(UP, HIP, j1=0.2, j3=-0.3), 1.4),
         (pose(UP, HIP, j1=0.2, j3=-0.3), 0.8), (pose(ARCH, ARCH), 1.4)]
    for _ in range(3):
        r += [(pose(PUNCH, GUARD, j1=0.2), 0.8), (pose(GUARD, PUNCH, j1=-0.2), 0.8)]
    r += [(pose(UP, HIP, j3=0.3), 1.4), (pose(UP, UP, j3=-0.3), 1.2), (pose(UP, UP), 1.2)]
    return r


def _disco():
    r = [(pose(REST, REST), 1.5)]
    for _ in range(3):
        r += [(pose(UP, DISCO_LO, j1=0.35), 1.0), (pose(DISCO_LO, UP, j1=-0.35), 1.0)]
    r += [(pose(V, V), 1.2), (pose(HALF, HALF), 1.0), (pose(V, V), 1.0)]
    return r


def _estiramiento():
    return [(pose(REST, REST), 2.0), (pose(V, V), 3.0), (pose(V, V, j1=0.4), 2.5),
            (pose(V, V, j1=-0.4), 2.5), (pose(SIDE, SIDE), 2.5), (pose(HUG, HUG), 2.5)]


def _pollito():
    r = [(pose(WING_A, WING_A), 1.5)]
    for _ in range(8):
        r += [(pose(WING_B, WING_B), 0.6), (pose(WING_A, WING_A), 0.6)]
    for c in (0.3, -0.3, 0.3, -0.3):
        r.append((pose(WING_A, WING_A, j1=c, j3=c), 0.9))
    r += [(pose(T, T), 1.2), (pose(WING_A, WING_A), 1.2)]
    return r


# ----------------- rutinas largas guardadas (unidades GUI) -----------------
_PERSONALIZADA = _desde_gui([
    ([0.0, 0.0, 0, 0.02, 0.04, 0.1, 0.1, -1.1, -0.18, 0.1, -0.17, -1.1], 2.0),
    ([0.0, 0.0, 0, 0.02, 0.04, 0.1, 0.1, -1.1, -0.18, 0.1, -0.17, -1.1], 2.0),
    ([0.0, 0.0, 0.0, 0.08, 4.64, -0.18, -4.73, -2.51, -0.18, 0.07, -0.17, -1.11], 2.0),
    ([0.0, 0.0, 0.0, 0.08, 4.64, -0.18, -4.73, -2.51, -0.18, 0.07, -0.17, -1.11], 0.5),
    ([0.0, 0.0, 0.0, 0.08, 4.65, 1.07, -4.72, -2.51, -0.18, 0.07, -0.17, -1.11], 0.5),
    ([0.0, 0.0, 0.0, 0.08, 4.65, 1.07, -4.72, -2.51, -0.18, 0.07, -0.17, -1.11], 0.5),
    ([0.0, 0.0, -0.0, 0.08, 4.64, -0.16, -4.74, -2.51, -0.18, 0.07, -0.17, -1.11], 0.5),
    ([0.0, 0.0, -0.0, 0.08, 4.64, -0.16, -4.74, -2.51, -0.18, 0.07, -0.17, -1.11], 1.0),
    ([0.0, 0.0, 0.0, 0.08, 4.65, 1.07, -4.72, -2.51, -0.18, 0.07, -0.17, -1.11], 0.5),
    ([0.0, 0.0, 0.0, 0.08, 4.65, 1.07, -4.72, -2.51, -0.18, 0.07, -0.17, -1.11], 0.5),
    ([0.0, 0.0, -0.0, 0.08, 4.64, -0.16, -4.74, -2.51, -0.18, 0.07, -0.17, -1.11], 0.5),
    ([0.0, 0.0, -0.0, 0.08, 4.64, -0.16, -4.74, -2.51, -0.18, 0.07, -0.17, -1.11], 0.5),
    ([0.0, 0.0, -0.0, 0.08, 2.35, -0.18, -2.44, -2.51, -0.18, 0.07, -0.17, -1.11], 1.0),
    ([0.0, 0.0, -0.0, 0.08, 2.35, -0.18, -2.44, -2.51, -0.18, 0.07, -0.17, -1.11], 1.0),
    ([0.0, 0.0, 0.0, 0.08, 2.50, -0.18, -2.44, -2.51, -0.18, 0.07, -0.17, -1.11], 1.0),
    ([0.0, 0.0, 0.0, 0.08, 2.50, -0.18, -2.44, -2.51, -0.18, 0.07, -0.17, -1.11], 1.0),
    ([0.0, 0.0, -0.0, 0.08, 2.35, -0.17, -2.44, -2.51, -0.18, 0.07, -0.17, -1.11], 2.0),
    ([0.0, 0.0, 0.0, 0.1, 2.66, -0.17, -1.52, 0.2, -2.8, -0.2, 1.65, 0.05], 4.0),
    ([0.01, 0.0, 0.0, 0.2, 1.96, 0.28, -1.72, 0.22, -2.13, -0.24, 1.75, -0.31], 2.0),
    ([0.01, 0.0, 0.0, 0.2, 1.96, 0.28, -1.72, -1.5, -2.11, -0.24, 1.76, -1.86], 2.0),
    ([0.01, 0.0, 0.0, 0.2, 1.96, 0.28, -1.72, 0.22, -2.13, -0.24, 1.75, -0.31], 2.0),
    ([0.01, 0.0, 0.0, 0.2, 1.96, 0.28, -1.72, -1.5, -2.11, -0.24, 1.76, -1.86], 2.0),
    ([0.0, 0.0, 0.0, 0.17, 2.77, 0.66, -2.81, -1.73, -2.81, 1.02, 2.81, -1.31], 2.0),
    ([0.0, 0.0, 0.0, 0.11, 4.63, -0.17, -4.73, -2.51, -4.92, 0.04, 4.98, -2.52], 1.5),
    ([0.0, 0.0, 0.0, 0.08, 4.65, 1.07, -4.72, -2.51, -4.92, 1, 4.98, -2.52], 0.5),
    ([0.0, 0.0, 0.0, 0.11, 4.63, -0.17, -4.73, -2.51, -4.92, 0.04, 4.98, -2.52], 0.5),
    ([0.0, 0.0, 0.0, 0.08, 4.65, 1.07, -4.72, -2.51, -4.92, 1, 4.98, -2.52], 0.5),
    ([0.0, 0.0, 0.0, 0.11, 4.63, -0.17, -4.73, -2.51, -4.92, 0.04, 4.98, -2.52], 0.5),
    ([0.0, 0.0, 0.0, 0.08, 4.65, 1.07, -4.72, -2.51, -4.92, 1, 4.98, -2.52], 0.5),
])

_RUTINA8 = _desde_gui([
    [0.0, 0.0, 0, 0.02, 0.04, 0.1, 0.1, -1.1, -0.18, 0.1, -0.17, -1.1],
    [0.17, 0.0, -0.0, 0.22, 0.46, -0.23, -1.76, 0.31, -0.55, -0.24, 1.76, 0.34],
    [0.24, 0.0, 0.08, 0.12, 1.64, 0.54, 0.5, -0.85, -1.71, 0.69, -0.72, -1.15],
    [0.34, 0.0, -0.01, 0.07, 0.03, -0.13, -0.37, -1.78, 0.01, -0.25, -0.01, -1.98],
    [0.23, 0.0, 0.08, 0.13, 0.21, -0.13, -0.01, -2.49, -4.67, -0.13, 4.99, -2.53],
    [0.23, 0.0, 0.08, 0.13, 0.21, -0.13, -0.01, -2.49, -4.67, 1.05, 4.97, -2.53],
    [0.23, 0.0, 0.08, 0.13, 0.21, -0.13, -0.01, -2.49, -4.67, 0.07, 4.97, -2.53],
    [0.23, 0.0, 0.08, 0.13, 4.85, 0.73, -2.63, -0.42, -4.67, 0.63, 2.28, -0.65],
    [0.24, 0.0, -0.54, 0.16, 4.85, 0.73, -2.63, -0.42, -4.67, 0.63, 2.28, -0.65],
    [0.24, 0.0, 0.54, 0.23, 4.85, 0.73, -2.63, -0.42, -4.67, 0.63, 2.28, -0.65],
    [0.24, 0.0, 0.02, 0.14, 4.85, 0.73, -2.63, -0.42, -4.67, 0.63, 2.28, -0.65],
    [0.24, 0.0, 0.02, 0.14, 2.53, -0.02, 5.0, -0.39, -2.16, -0.07, -0.06, 0.34],
    [0.24, 0.0, 0.02, 0.14, 2.53, -0.06, -3.92, -0.39, -2.16, -0.06, 1.4, 0.34],
    [0.24, 0.0, 0.02, 0.15, 2.53, -0.05, 3.93, -0.39, -2.16, -0.17, -1.01, 0.34],
    [0.24, 0.0, 0.02, 0.14, 1.67, -0.34, -0.38, -1.46, -1.73, -0.54, 0.03, -1.33],
    [-1.21, 0.0, -0.01, 0.12, 0.03, -0.13, -0.37, -1.78, 0.01, -0.25, -0.01, -1.98],
    [-1.21, 0.0, -0.8, -0.06, 0.03, -0.13, -0.37, -1.78, 0.01, -0.25, -0.01, -1.98],
    [-1.21, 0.0, -0.03, 0.64, 0.03, -0.13, -0.37, -1.78, 0.01, -0.25, -0.01, -1.98],
    [0.34, 0.0, -0.01, 0.07, 0.03, -0.13, -0.37, -1.78, 0.01, -0.25, -0.01, -1.98],
], 2.5)

CATALOGO_NUEVAS = [
    ("Ola", _ola), ("Robot", _robot), ("Pajaro", _pajaro), ("Remo", _remo),
    ("Superheroe", _superheroe), ("Disco", _disco),
    ("Estiramiento", _estiramiento), ("Pollito", _pollito),
]
CATALOGO_RUTINAS = [("Rutina 8", _RUTINA8)] + \
                   [(n, f()) for n, f in CATALOGO_NUEVAS]


# ======================================================================
#  COREOGRAFIAS POR CANCION (patch_canciones)
# ======================================================================
import os as _os, json as _json, math as _m

MIN_UNIT = 0.6      # s minimos por paso (si el tiempo es mas corto, se baila a medio tiempo)
VMAX_SONG = 2.0     # rad/s maximo; si un paso es mas rapido se alarga en tiempos enteros
LEAD = 0.6          # s de empalme entre bloques de trayectoria
CHUNK_S = 60.0      # duracion aproximada de cada bloque enviado
SONG_SYNC = 0.0     # s de espera antes del primer paso (sube si el baile se adelanta a la musica)


def _alt(a, b, n, j1=0.0, j3=0.0, beats=1):
    out = []
    for k in range(n):
        sg = 1 if k % 2 == 0 else -1
        r, l = (a, b) if k % 2 == 0 else (b, a)
        out.append((pose(r, l, j1=j1 * sg, j3=j3 * sg), beats))
    return out


def _flap(hi, lo, n, beats=1):
    out = []
    for _ in range(n):
        out += [(pose(hi, hi), beats), (pose(lo, lo), beats)]
    return out


def _bal(arm, n, amp=0.4, j3=0.0, beats=1):
    return [(pose(arm, arm, j1=amp * (1 if k % 2 == 0 else -1),
                  j3=j3 * (1 if k % 2 == 0 else -1)), beats) for k in range(n)]


FRASES = {
    "aletea": lambda: _flap(WING_HI, WING_LO, 2),
    "pollito": lambda: _flap(WING_A, WING_B, 4),
    "ola": lambda: [(pose(MID, REST, j1=0.3), 1), (pose(UP, MID, j1=0.3), 1),
                    (pose(MID, UP, j1=-0.3), 1), (pose(REST, MID, j1=-0.3), 1)],
    "marcha": lambda: _alt(MID, REST, 4, j1=0.25),
    "disco": lambda: _alt(UP, DISCO_LO, 4, j1=0.3),
    "pop": lambda: _alt(UP, HIP, 4, j1=0.2),
    "punos": lambda: _alt(PUNCH, GUARD, 4, j1=0.2),
    "vsi": lambda: [(pose(V, V), 1), (pose(SIDE, SIDE), 1), (pose(V, V), 1), (pose(HALF, HALF), 1)],
    "abraza": lambda: [(pose(HUG, HUG), 2), (pose(SIDE, SIDE), 2)],
    "sway": lambda: _bal(SIDE, 4, 0.4),
    "cadera": lambda: _bal(HIP, 4, 0.4),
    "cuello": lambda: _bal(T, 4, 0.0, j3=0.45),
    "vuela": lambda: [(pose(V, V, j1=0.4, j3=0.3), 2), (pose(V, V, j1=-0.4, j3=-0.3), 2)],
    "gira": lambda: [(pose(SIDE, SIDE, j1=0.9), 2), (pose(SIDE, SIDE, j1=-0.9), 2)],
    "heroe": lambda: [(pose(UP, HIP, j1=0.2, j3=-0.3), 2), (pose(ARCH, ARCH), 2)],
    "saluda": lambda: [(pose(UP, REST), 1), (pose(MID, REST), 1)] * 2,
    "remo": lambda: [(pose(FWD, ROW_BACK, j1=0.35), 2), (pose(ROW_BACK, FWD, j1=-0.35), 2)],
    "lento_sube": lambda: [(pose(UP, UP), 4), (pose(V, V), 2), (pose(SIDE, SIDE), 2)],
    "respira": lambda: [(pose(HALF, HALF), 2), (pose(HUG, HUG), 2)],
    "muu": lambda: [(pose(HALFB, HALFB, j3=0.4), 2), (pose(HALFB, HALFB, j3=-0.4), 2)],
    "peso": lambda: _alt(HUG, MID, 4, j1=0.2),
}

# id de la cancion (igual que en el C++): (BPM estimado, frases en orden)
CANCIONES_COREO = {
    "aramsamsam": (125, ["saluda", "marcha", "pollito", "ola", "disco", "vsi", "sway", "marcha", "aletea", "vuela", "cuello", "pollito"]),
    "gorila": (110, ["peso", "peso", "remo", "sway", "punos", "peso", "cadera", "gira", "peso", "punos", "cuello", "remo"]),
    "libresoy": (137, ["respira", "lento_sube", "ola", "vuela", "lento_sube", "abraza", "heroe", "lento_sube", "gira", "respira"]),
    "bailedelsapo": (120, ["pollito", "cuello", "pollito", "marcha", "aletea", "pollito", "gira", "cuello", "pollito", "vsi"]),
    "gallinaturuleca": (125, ["pollito", "cuello", "aletea", "pollito", "cadera", "pollito", "cuello", "aletea", "gira", "pollito"]),
    "pollitopio": (115, ["pollito", "pollito", "saluda", "pollito", "cuello", "aletea", "pollito", "sway", "pollito"]),
    "vacalola": (105, ["muu", "marcha", "cuello", "muu", "sway", "peso", "muu", "ola", "cuello", "muu"]),
    "hakunamatata": (100, ["sway", "ola", "abraza", "sway", "vsi", "ola", "respira", "sway", "saluda", "vuela"]),
    "amigofiel": (90, ["saluda", "abraza", "marcha", "abraza", "sway", "ola", "saluda", "vsi", "abraza", "respira"]),
    "bruno": (106, ["cuello", "respira", "cuello", "abraza", "cadera", "cuello", "heroe", "respira", "gira", "cuello"]),
    "apt": (149, ["disco", "pop", "disco", "punos", "cadera", "disco", "pop", "vsi", "punos", "cadera", "heroe", "disco"]),
    "wakawaka": (127, ["marcha", "disco", "sway", "aletea", "marcha", "gira", "disco", "ola", "vsi", "marcha", "punos", "heroe"]),
    "vidaloca": (176, ["cadera", "cadera", "pop", "ola", "cadera", "disco", "gira", "cadera", "pop", "heroe"]),
    "yakkomundo": (140, ["marcha", "punos", "disco", "gira", "marcha", "remo", "pop", "aletea", "punos", "vsi", "marcha", "gira"]),
    "lallorona": (72, ["respira", "lento_sube", "cuello", "abraza", "respira", "ola", "lento_sube", "sway", "respira"]),
    "goldenkpop": (120, ["heroe", "punos", "vuela", "heroe", "pop", "punos", "gira", "heroe", "disco", "vuela", "heroe"]),
    "sodapop": (120, ["pop", "disco", "cadera", "pollito", "pop", "vsi", "disco", "gira", "cadera", "pop", "aletea"]),
    "youridol": (125, ["cuello", "punos", "cadera", "pop", "cuello", "disco", "punos", "gira", "pop", "heroe"]),
}


def _bpm_de(sid):
    env = _os.environ.get("BPM", "").strip()
    if env:
        return float(env)
    try:   # medidos con el script de BPM (opcional)
        with open(_os.path.expanduser("~/.yaren/bpm.json")) as f:
            v = _json.load(f).get(sid)
            if v:
                return float(v)
    except Exception:
        pass
    return float(CANCIONES_COREO[sid][0])


def coreografia_cancion(sid):
    """Devuelve (ciclo, unit): ciclo = [(pose, segundos)] alineado a la rejilla de tiempos."""
    bpm = _bpm_de(sid)
    unit = 60.0 / bpm
    while unit < MIN_UNIT:
        unit *= 2.0
    steps = []
    for nombre in CANCIONES_COREO[sid][1]:
        steps += FRASES[nombre]()
    steps.append((pose(REST, REST), 2))   # cierra el ciclo en base
    ciclo, prev = [], list(BASE_POSE)
    for p, b in steps:
        p = [float(x) for x in p]
        for i in (4, 6, 8, 10):
            p[i] = max(-3.0, min(3.0, p[i]))
        d = max(abs(a - c) for a, c in zip(p, prev))
        n = max(int(b), int(_m.ceil(d / (VMAX_SONG * unit) - 1e-9)))
        ciclo.append((p, n * unit))
        prev = p
    return ciclo, unit, bpm


# ======================================================================
#  SINCRONIA CON LA MUSICA (patch_sync)
# ======================================================================
BEATS_FILE = _os.path.expanduser("~/.yaren/beats.json")
RET_V = 1.0         # rad/s maximo en el regreso a base
START_MARGIN = 0.15  # s de margen para publicar antes del siguiente tiempo


RET_T1 = 1.0   # s: cerrar brazos y codos
RET_T2 = 1.8   # s: bajar a base


def _cl(p):
    p = [float(x) for x in p]
    for i in (4, 6, 8, 10):
        p[i] = max(-3.0, min(3.0, p[i]))
    return p


def _grid(sid):
    """Rejilla de pasos: unit (s), t0 (fase, s de cancion), D (duracion de la vuelta)."""
    try:
        with open(BEATS_FILE) as f:
            info = _json.load(f).get(sid)
    except Exception:
        info = None
    if info:
        P, t0, D, acc = info["period"], info["t0"], info["dur"], info.get("acc", [0, 0, 0, 0])
    else:
        P, t0, D, acc = 60.0 / CANCIONES_COREO[sid][0], 0.0, 120.0, [0, 0, 0, 0]
    env = _os.environ.get("BPM", "").strip()
    if env:
        P = 60.0 / float(env)
    D += float(_os.environ.get("LOOP_ADJ", "0") or 0)
    m = 1
    while P * m < MIN_UNIT:
        m *= 2
    m = min(m, 4)
    r = max(range(m), key=lambda r: sum(acc[j] for j in range(4) if j % m == r))
    return {"P": P, "m": m, "unit": P * m, "t0": t0 + r * P, "D": D, "info": bool(info)}


def ciclo_unidades(sid, unit):
    """Coreografia en pasos de n unidades (n entero; se alarga si el movimiento es muy rapido)."""
    steps = []
    for nombre in CANCIONES_COREO[sid][1]:
        steps += FRASES[nombre]()
    steps.append((pose(REST, REST), 2))
    ciclo, prev = [], list(BASE_POSE)
    for p, b in steps:
        p = _cl(p)
        d = max(abs(a - c) for a, c in zip(p, prev))
        n = max(int(b), int(_m.ceil(d / (VMAX_SONG * unit) - 1e-9)))
        ciclo.append((p, n))
        prev = p
    return ciclo


def _retorno(prev, unit):
    """Regreso a base en 2 pasos lentos: cierra abduccion y codos, luego baja."""
    inter = list(prev)
    inter[5] = 0.0
    inter[9] = 0.0
    inter[7] = 0.5
    inter[11] = 0.5
    n1 = max(1, int(_m.ceil(max(abs(a - b) for a, b in zip(prev, inter)) / (RET_V * unit) - 1e-9)))
    n2 = max(2, int(_m.ceil(max(abs(a - b) for a, b in zip(inter, BASE_POSE)) / (RET_V * unit) - 1e-9)))
    return [(inter, n1), (list(BASE_POSE), n2)]


def plan_loop(g, ciclo, k0, k_last):
    """Poses con tiempo de cancion (s) sobre la rejilla. Llega a la 1ra pose en el slot k0,
    la ultima llegada es <= slot k_last y SIEMPRE termina en base. None si no cabe."""
    unit, t0 = g["unit"], g["t0"]
    pts, k, prev, i = [(list(BASE_POSE), k0)], k0, list(BASE_POSE), 0
    while True:
        p, n = ciclo[i % len(ciclo)]
        if k + n + sum(nn for _, nn in _retorno(p, unit)) > k_last:
            break
        k += n
        pts.append((p, k))
        prev = p
        i += 1
    for pp, nn in _retorno(prev, unit):
        k += nn
        pts.append((pp, k))
    if i == 0 or k > k_last:
        return None
    return [(pp, t0 + kk * unit) for pp, kk in pts]


class YarenDanceRadio(Node):
    def __init__(self):
        super().__init__('yaren_dance_radio')
        
        self.publisher = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10)
        
        self.joint_names = [
            "joint_1", "joint_2", "joint_3", "joint_4", 
            "joint_5", "joint_6", "joint_7", "joint_8", 
            "joint_9", "joint_10", "joint_11", "joint_12"
        ]
        self.get_logger().info("¡Yaren comenzó a bailar la música de la radio!")

    def send_movement(self, positions, seconds):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        
        # Convierte segundos a formato ROS
        point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        msg.points.append(point)
        self.publisher.publish(msg)

    def send_routine(self, routine):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        msg.points, total = smooth_points(routine)
        # Publicar hasta que el controlador este enlazado (max 40 s)
        t0 = time.time()
        while rclpy.ok() and self.publisher.get_subscription_count() == 0:
            if time.time() - t0 > 40.0:
                self.get_logger().warn("Controlador sin enlazar tras 40 s, envio igual")
                break
            time.sleep(0.1)
        time.sleep(0.1)                      # margen tras el match
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Trayectoria enviada ({len(msg.points)} puntos, {total:.1f} s), "
            f"suscriptores: {self.publisher.get_subscription_count()}")
        return total

    def wait_interruptible(self, secs):
        end = time.time() + secs
        while rclpy.ok() and time.time() < end:
            time.sleep(0.05)

    def go_base(self, secs=2.5):
        """Vuelve a la posicion base (reemplaza cualquier trayectoria en curso)."""
        self.send_movement(BASE_POSITION, secs)
        time.sleep(secs + 0.3)

    def run_song(self, sid):
        """Baila la cancion pegado a la musica. False si no hay coreografia."""
        if sid not in CANCIONES_COREO:
            return False
        g = _grid(sid)
        unit, t0, D = g["unit"], g["t0"], g["D"]
        ciclo = ciclo_unidades(sid, unit)
        sync = float(_os.environ.get("SYNC", SONG_SYNC))
        start = float(_os.environ.get("SONG_START") or time.time())
        k_first = int(_m.ceil((LEAD - t0) / unit - 1e-9))
        k_last = k_first + int(_m.floor((D - LEAD - 0.1) / unit))
        if not g["info"]:
            self.get_logger().warn("Sin medicion de beats: uso BPM de memoria (corre medir_beats.py)")
        self.get_logger().info(
            f"{sid}: {60.0 / g['P']:.1f} BPM, paso cada {g['m']} tiempo(s) = {unit:.3f} s, "
            f"fase {t0:.2f} s, vuelta {D:.1f} s, ciclo de {len(ciclo)} pasos")

        t_w = time.time()
        while rclpy.ok() and self.publisher.get_subscription_count() == 0:
            if time.time() - t_w > 40.0:
                self.get_logger().warn("Controlador sin enlazar, envio igual")
                break
            time.sleep(0.05)

        def E(L, s_):                      # hora real de la cancion en la vuelta L
            return start + L * D + s_ + sync

        import signal as _signal
        state = {"plan": None, "L": 0, "bye": False}

        def _bye(signum, frame):
            """SIGTERM (cambio de cancion, pausa o stop): regresa suave a base y sale."""
            if state["bye"]:
                return
            state["bye"] = True
            from trajectory_msgs.msg import JointTrajectoryPoint
            from builtin_interfaces.msg import Duration
            pose, plan = list(BASE_POSE), state["plan"]
            if plan:
                t = time.time() - E(state["L"], 0.0)
                for j in range(1, len(plan)):
                    (pa, ta), (pb, tb) = plan[j - 1], plan[j]
                    if ta <= t < tb:
                        f = (t - ta) / (tb - ta)
                        pose = [x + (y - x) * f for x, y in zip(pa, pb)]
                        break
            inter = list(pose)
            inter[5] = inter[9] = 0.0
            inter[7] = inter[11] = 0.5
            msg = JointTrajectory()
            msg.joint_names = self.joint_names
            for pos, sec in ((inter, RET_T1), (list(BASE_POSE), RET_T1 + RET_T2)):
                pt = JointTrajectoryPoint()
                pt.positions = _cl(pos)
                pt.time_from_start = Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9))
                msg.points.append(pt)
            self.publisher.publish(msg)
            time.sleep(RET_T1 + RET_T2 + 0.1)
            _os._exit(0)

        _signal.signal(_signal.SIGTERM, _bye)

        L = 0
        while rclpy.ok():
            s_now = time.time() - E(L, 0.0)
            while s_now >= D:
                L += 1
                s_now -= D
            k0 = max(k_first, int(_m.ceil((s_now + LEAD + START_MARGIN - t0) / unit)))
            plan = plan_loop(g, ciclo, k0, k_last)
            if plan is None:
                if k0 <= k_first:
                    self.get_logger().warn("La cancion es muy corta para la coreografia")
                    return False
                L += 1                     # no cabe en lo que queda: espera la siguiente vuelta
                continue
            rutina = [(plan[0][0], LEAD)] + [
                (plan[j][0], plan[j][1] - plan[j - 1][1]) for j in range(1, len(plan))]
            msg = JointTrajectory()
            msg.joint_names = self.joint_names
            msg.points, _ = smooth_points(rutina, scale=1.0, start_delay=LEAD)
            t_pub = E(L, plan[0][1] - LEAD)
            while rclpy.ok() and time.time() < t_pub:
                time.sleep(min(0.05, max(0.001, t_pub - time.time())))
            self.publisher.publish(msg)
            state["plan"], state["L"] = plan, L
            self.get_logger().info(
                f"Vuelta {L}: {len(plan) - 1} pasos, entra en el segundo {plan[0][1]:.2f} de la cancion")
            L += 1
        return True

    def run_dance(self):
        _song = _os.environ.get("SONG", "").strip().lower()
        if _song and self.run_song(_song):
            return
        rutina_1 = [
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, -3.0, 0.0, -1.5, 0.0, 3.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.5], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, -1.5, 0.0, 3.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.0, -3.0, 0.0, -3.0, 0.0, 3.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 3.0, 1.0, -3.0, 0.0, -3.0, 1.0, 3.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.0, -3.0, 0.0, -3.0, 0.0, 3.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 3.0, 1.0, -3.0, 0.0, -3.0, 1.0, 3.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.0, -3.0, 0.0, -3.0, 0.0, 3.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.5], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, -1.5, 0.0, 0.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 1.0, -1.5, 0.0, 0.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 1.0, -1.5, 0.0, 0.0, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, -1.5, 0.0, 0.0, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, -1.5, 0.0, 0.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 1.0, -1.5, 0.0, 0.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 1.0, -1.5, 0.0, 1.5, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, -1.5, 1.0, -1.5, 0.0, 0.0, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, -1.0, 1.0, -3.0, 0.5, 3.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.5, -3.0, 0.0, -1.5, 0.0, 1.0, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, -1.0, 1.0, -3.0, 0.5, 3.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.5, -3.0, 0.0, -1.5, 0.0, 1.0, 1.0], 2.0)
        ]

        # Rutina 2: Los nuevos movimientos que probaste en la terminal
        rutina_2 = [
            ([0.0, 0.0, 0.0, 0.0, 1.5, 1.0, 0.0, 0.0, -1.5, 1.0, 0.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.0, -1.5, 0.0, -3.0, 0.0, 1.5, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 1.0, 0.0, 0.0, -1.5, 1.0, 0.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.0, -1.5, 0.0, -3.0, 0.0, 1.5, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 1.0, 0.0, 0.0, -1.5, 1.0, 0.0, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 3.0, 0.0, -1.5, 0.0, -3.0, 0.0, 1.5, 0.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.5, 0.0, -1.5, 1.0, -1.5, 0.0, 1.5, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 2.0, 0.0, -1.5, 1.0, -2.0, 0.0, 1.5, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.5, 1.0, -1.0, 0.0, 1.5, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 2.0, 0.0, -1.5, 1.0, -2.0, 0.0, 1.5, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.5, 1.0, -1.0, 0.0, 1.5, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 2.0, 0.0, -1.5, 1.0, -2.0, 0.0, 1.5, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.5, 1.0, -1.0, 0.0, 1.5, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 2.0, 0.0, -1.5, 1.0, -2.0, 0.0, 1.5, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.5, 1.0, -1.0, 0.0, 1.5, 1.0], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.5, -0.2, 0.0, 0.0, 1.3], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 1.3, -0.5, 0.0, 0.0, 0.5], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.5, -0.2, 0.0, 0.0, 1.3], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 1.3, -0.5, 0.0, 0.0, 0.5], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.5, -0.2, 0.0, 0.0, 1.3], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 1.3, -0.5, 0.0, 0.0, 0.5], 2.0)
        ]
        catalogo = [("Original 1", rutina_1), ("Original 2", rutina_2)] + list(CATALOGO_RUTINAS)
        solo = _os.environ.get("RUTINA", "").strip().lower()
        if solo:
            catalogo = [c for c in catalogo if solo in c[0].lower()] or catalogo
        self.get_logger().info("Catalogo: " + ", ".join(n for n, _ in catalogo))

        bolsa, ultima = [], None
        while rclpy.ok():
            if not bolsa:
                bolsa = list(range(len(catalogo)))
                random.shuffle(bolsa)
                if len(bolsa) > 1 and bolsa[-1] == ultima:
                    bolsa[0], bolsa[-1] = bolsa[-1], bolsa[0]
            idx = bolsa.pop()
            nombre, rutina = catalogo[idx]
            self.get_logger().info("Iniciando: " + nombre)
            total = self.send_routine(preparar(rutina))
            self.wait_interruptible(total)
            ultima = idx

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = YarenDanceRadio()
    try:
        node.run_dance()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.go_base()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()