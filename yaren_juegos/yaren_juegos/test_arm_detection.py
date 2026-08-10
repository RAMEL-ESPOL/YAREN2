#!/usr/bin/env python3
"""
test_arm_detection.py
=====================
Script standalone para probar la detección de direcciones con los brazos
usando YOLOv8-pose. Sin ROS2, sin dependencias extra.

Mapeo de direcciones (retorna SET de activas simultáneamente):
  UP            → ambos brazos arriba (muñecas sobre hombros)
  DOWN          → ambos brazos pegados al cuerpo (muñecas cerca de caderas/muslos)
  LEFT          → brazo izquierdo extendido lateral
  RIGHT         → brazo derecho extendido lateral
  LEFT + RIGHT  → T-pose (ambos brazos laterales al mismo tiempo)
  UP + LEFT     → brazo izquierdo arriba, derecho lateral
  UP + RIGHT    → brazo derecho arriba, izquierdo lateral

Ejecutar:
  python3 test_arm_detection.py
  python3 test_arm_detection.py --source /dev/video0
  python3 test_arm_detection.py --source 0
  python3 test_arm_detection.py --model /ruta/al/yolov8s-pose.pt
"""

import argparse
import math
import os
import time

import cv2
import numpy as np

# ─── Keypoint indices (COCO 17-point) ────────────────────────────────────────
NOSE          = 0
LEFT_EYE      = 1; RIGHT_EYE      = 2
LEFT_EAR      = 3; RIGHT_EAR      = 4
LEFT_SHOULDER = 5; RIGHT_SHOULDER = 6
LEFT_ELBOW    = 7; RIGHT_ELBOW    = 8
LEFT_WRIST    = 9; RIGHT_WRIST    = 10
LEFT_HIP      = 11; RIGHT_HIP    = 12
LEFT_KNEE     = 13; RIGHT_KNEE   = 14
LEFT_ANKLE    = 15; RIGHT_ANKLE  = 16

# ─── Colores UI ──────────────────────────────────────────────────────────────
C_WHITE  = (255, 255, 255)
C_BLACK  = (  0,   0,   0)
C_YELLOW = (  0, 220, 255)
C_GREEN  = ( 50, 230,  80)
C_RED    = ( 60,  60, 255)
C_CYAN   = (255, 220,   0)
C_GRAY   = (120, 120, 120)
C_ORANGE = ( 30, 140, 255)

# Colores por combinación de poses
POSE_COLORS = {
    frozenset({"UP"}):           ( 50, 230,  80),   # verde
    frozenset({"DOWN"}):         ( 60,  60, 255),   # rojo
    frozenset({"LEFT"}):         (  0, 220, 255),   # amarillo
    frozenset({"RIGHT"}):        (  0, 180, 255),   # naranja
    frozenset({"LEFT", "RIGHT"}): (200,  80, 255),  # magenta — T-pose
    frozenset({"UP", "LEFT"}):   (255, 200,  50),   # cyan
    frozenset({"UP", "RIGHT"}):  (255, 100, 150),   # rosa
    frozenset():                 (100, 100, 100),   # gris — neutral
}

POSE_NAMES = {
    frozenset({"UP"}):            "ARMS UP",
    frozenset({"DOWN"}):          "ARMS DOWN",
    frozenset({"LEFT"}):          "LEFT ARM",
    frozenset({"RIGHT"}):         "RIGHT ARM",
    frozenset({"LEFT", "RIGHT"}): "T-POSE",
    frozenset({"UP", "LEFT"}):    "LEFT UP",
    frozenset({"UP", "RIGHT"}):   "RIGHT UP",
    frozenset():                  "NEUTRAL",
}

W, H = 800, 480


# ─── Detección de dirección ───────────────────────────────────────────────────

def detect_direction(kpts):
    """
    kpts: array (17, 2) con coordenadas (x, y) en píxeles.
    Retorna: (frozenset de strings activos, dict debug)

    Ejemplo de retorno: (frozenset({"LEFT", "RIGHT"}), {...})

    Lógica por componente:
      UP    → ambas muñecas claramente por encima de ambos hombros
      DOWN  → ambas muñecas a la altura de caderas/muslos y cerca del cuerpo
      LEFT  → muñeca izquierda extendida lateralmente (x menor que hombro izq)
      RIGHT → muñeca derecha extendida lateralmente (x mayor que hombro der)

    Umbrales adaptativos basados en el ancho entre hombros (sw).
    """
    if kpts is None or len(kpts) < 13:
        return frozenset(), {}

    ls = kpts[LEFT_SHOULDER]
    rs = kpts[RIGHT_SHOULDER]
    lw = kpts[LEFT_WRIST]
    rw = kpts[RIGHT_WRIST]
    le = kpts[LEFT_ELBOW]
    re = kpts[RIGHT_ELBOW]
    lh = kpts[LEFT_HIP]
    rh = kpts[RIGHT_HIP]

    def valid(p):
        return p[0] > 5 and p[1] > 5

    sw = abs(ls[0] - rs[0])   # ancho entre hombros — referencia de escala
    if sw < 10:
        return frozenset(), {}

    # ── Umbrales adaptativos ──────────────────────────────────────────────────
    v_thresh  = sw * 0.40    # diferencia vertical para "arriba"
    side_h    = sw * 0.50    # separación horizontal mínima para brazo lateral
                             # (más permisivo que antes: era 0.65)
    side_v    = sw * 0.65    # tolerancia vertical para brazo lateral
                             # (más permisivo: era 0.50)
    down_x    = sw * 0.35    # muñeca no puede estar muy separada lateralmente para DOWN

    mid_shoulder_y = (ls[1] + rs[1]) / 2
    mid_hip_y      = (lh[1] + rh[1]) / 2 if valid(lh) and valid(rh) else mid_shoulder_y + sw * 1.5

    detected = set()
    debug = {
        "sw": sw,
        "v_thresh": v_thresh,
        "side_h": side_h,
        "side_v": side_v,
    }

    # ── UP: ambas muñecas por encima de los hombros ───────────────────────────
    if valid(lw) and valid(rw) and valid(le) and valid(re):
        lw_above = (ls[1] - lw[1]) > v_thresh
        rw_above = (rs[1] - rw[1]) > v_thresh
        le_above = (ls[1] - le[1]) > v_thresh * 0.3
        re_above = (rs[1] - re[1]) > v_thresh * 0.3
        debug["lw_above"] = lw_above
        debug["rw_above"] = rw_above
        if lw_above and rw_above and le_above and re_above:
            detected.add("UP")

    # ── DOWN: ambas muñecas cerca del cuerpo a la altura de caderas/muslos ────
    # La condición es: muñecas por debajo del nivel del hombro Y cerca del eje
    # del cuerpo en X (no extendidas lateralmente).
    if valid(lw) and valid(rw):
        lw_below_shoulder = lw[1] > mid_shoulder_y + sw * 0.3
        rw_below_shoulder = rw[1] > mid_shoulder_y + sw * 0.3
        # No deben estar muy separadas del cuerpo lateralmente
        lw_near_body = abs(lw[0] - ls[0]) < down_x + sw * 0.5
        rw_near_body = abs(rw[0] - rs[0]) < down_x + sw * 0.5
        debug["lw_below_shoulder"] = lw_below_shoulder
        debug["rw_below_shoulder"] = rw_below_shoulder
        debug["lw_near_body"]      = lw_near_body
        debug["rw_near_body"]      = rw_near_body
        if lw_below_shoulder and rw_below_shoulder and lw_near_body and rw_near_body:
            # Solo si UP no está activo (evitar ambigüedad)
            if "UP" not in detected:
                detected.add("DOWN")

    # ── RIGHT: muñeca derecha extendida lateralmente hacia la derecha ─────────
    # En imagen espejada (webcam), el brazo DERECHO del usuario
    # aparece a la IZQUIERDA de la pantalla (x menor que el hombro derecho).
    if valid(rw) and valid(re):
        # Extensión lateral: muñeca más separada que el hombro en X
        rw_lateral_x = (rs[0] - rw[0])    # positivo = muñeca a izquierda del hombro (imagen espejada)
        rw_v_diff    = abs(rw[1] - rs[1])  # diferencia vertical con hombro
        debug["rw_lateral_x"] = rw_lateral_x
        debug["rw_v_diff"]    = rw_v_diff
        if rw_lateral_x > side_h and rw_v_diff < side_v:
            detected.add("RIGHT")

    # ── LEFT: muñeca izquierda extendida lateralmente hacia la izquierda ──────
    if valid(lw) and valid(le):
        lw_lateral_x = (lw[0] - ls[0])    # positivo = muñeca a derecha del hombro
        lw_v_diff    = abs(lw[1] - ls[1])
        debug["lw_lateral_x"] = lw_lateral_x
        debug["lw_v_diff"]    = lw_v_diff
        if lw_lateral_x > side_h and lw_v_diff < side_v:
            detected.add("LEFT")

    # DOWN no puede coexistir con LEFT o RIGHT
    if "DOWN" in detected and ("LEFT" in detected or "RIGHT" in detected):
        detected.discard("DOWN")

    return frozenset(detected), debug


# ─── Dibujado ─────────────────────────────────────────────────────────────────

def draw_skeleton(frame, kpts, color=(0, 255, 0)):
    if kpts is None:
        return
    CONNECTIONS = [
        (LEFT_SHOULDER,  RIGHT_SHOULDER),
        (LEFT_SHOULDER,  LEFT_ELBOW),
        (LEFT_ELBOW,     LEFT_WRIST),
        (RIGHT_SHOULDER, RIGHT_ELBOW),
        (RIGHT_ELBOW,    RIGHT_WRIST),
        (LEFT_SHOULDER,  LEFT_HIP),
        (RIGHT_SHOULDER, RIGHT_HIP),
        (LEFT_HIP,       RIGHT_HIP),
        (LEFT_HIP,       LEFT_KNEE),
        (RIGHT_HIP,      RIGHT_KNEE),
        (LEFT_KNEE,      LEFT_ANKLE),
        (RIGHT_KNEE,     RIGHT_ANKLE),
    ]
    for a, b in CONNECTIONS:
        if a >= len(kpts) or b >= len(kpts):
            continue
        pa = (int(kpts[a][0]), int(kpts[a][1]))
        pb = (int(kpts[b][0]), int(kpts[b][1]))
        if pa == (0, 0) or pb == (0, 0):
            continue
        cv2.line(frame, pa, pb, color, 2, cv2.LINE_AA)
    for i, (x, y) in enumerate(kpts):
        if x < 5 and y < 5:
            continue
        cv2.circle(frame, (int(x), int(y)), 4, (255, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(frame, (int(x), int(y)), 4, color, 1, cv2.LINE_AA)


def draw_direction_indicators(frame, detected, confidence_t):
    """
    Dibuja flechas para cada dirección activa en el set `detected`.
    """
    cx, cy = W // 2, H // 2
    color  = POSE_COLORS.get(detected, (100, 100, 100))
    pulse  = 0.75 + 0.25 * math.sin(confidence_t * 8.0)

    if not detected:
        cv2.circle(frame, (cx, 420), 18, (100, 100, 100), 2, cv2.LINE_AA)
        cv2.putText(frame, "---", (cx - 22, 426),
                    cv2.FONT_HERSHEY_DUPLEX, 0.7, (100, 100, 100), 1, cv2.LINE_AA)
        return

    # Una flecha por cada dirección activa
    arrow_offsets = {
        "UP":    (0, -110),
        "DOWN":  (0,  110),
        "LEFT":  (-130, 0),
        "RIGHT": ( 130, 0),
    }
    arrow_pts_templates = {
        "UP":    [(0, -40), (-22, 0), (22, 0)],
        "DOWN":  [(0,  40), (-22, 0), (22, 0)],
        "LEFT":  [(-40, 0), (0, -22), (0, 22)],
        "RIGHT": [( 40, 0), (0, -22), (0, 22)],
    }

    for d in detected:
        dx, dy   = arrow_offsets[d]
        ax, ay   = cx + dx, cy + dy
        raw_pts  = arrow_pts_templates[d]
        pts      = np.array([(ax + p[0], ay + p[1]) for p in raw_pts], dtype=np.int32)
        overlay  = frame.copy()
        cv2.fillPoly(overlay, [pts], color, cv2.LINE_AA)
        cv2.addWeighted(overlay, 0.7 * pulse, frame, 1 - 0.7 * pulse, 0, frame)


def draw_hud(frame, detected, fps, debug, hold_t, confirmed):
    color = POSE_COLORS.get(detected, (100, 100, 100))
    pose_name = POSE_NAMES.get(detected, "+".join(sorted(detected)) if detected else "NEUTRAL")

    # Barra inferior
    bar_y = H - 60
    cv2.rectangle(frame, (0, bar_y), (W, H), (12, 10, 22), -1)

    # Nombre de la pose
    (tw, _), _ = cv2.getTextSize(pose_name, cv2.FONT_HERSHEY_DUPLEX, 1.2, 2)
    cv2.putText(frame, pose_name, (W // 2 - tw // 2 + 2, bar_y + 42),
                cv2.FONT_HERSHEY_DUPLEX, 1.2, C_BLACK, 4, cv2.LINE_AA)
    cv2.putText(frame, pose_name, (W // 2 - tw // 2, bar_y + 40),
                cv2.FONT_HERSHEY_DUPLEX, 1.2, color, 2, cv2.LINE_AA)

    # FPS
    cv2.putText(frame, f"FPS:{fps:.0f}", (8, bar_y + 20),
                cv2.FONT_HERSHEY_PLAIN, 0.9, C_GRAY, 1, cv2.LINE_AA)

    # Hold timer
    HOLD_NEEDED = 0.4
    bar_w = int(min(hold_t / HOLD_NEEDED, 1.0) * 200)
    cv2.rectangle(frame, (8, bar_y + 28), (208, bar_y + 38), (40, 35, 55), -1)
    cv2.rectangle(frame, (8, bar_y + 28), (8 + bar_w, bar_y + 38), color, -1)

    # CONFIRMED flash
    if confirmed:
        ov = frame.copy()
        cv2.rectangle(ov, (0, 0), (W, H), color, -1)
        cv2.addWeighted(ov, 0.18, frame, 0.82, 0, frame)
        (cw, _), _ = cv2.getTextSize("CONFIRMED", cv2.FONT_HERSHEY_DUPLEX, 1.2, 2)
        cv2.putText(frame, "CONFIRMED", (W // 2 - cw // 2, 60),
                    cv2.FONT_HERSHEY_DUPLEX, 1.2, color, 2, cv2.LINE_AA)

    # Debug info
    if debug:
        dy = 20
        sw = debug.get("sw", 0)
        cv2.putText(frame, f"sw={sw:.0f}  side_h={debug.get('side_h',0):.0f}  side_v={debug.get('side_v',0):.0f}",
                    (8, dy), cv2.FONT_HERSHEY_PLAIN, 0.75, C_GRAY, 1, cv2.LINE_AA)
        dy += 16
        show_keys = ["lw_above","rw_above","lw_below_shoulder","rw_below_shoulder",
                     "lw_near_body","rw_near_body",
                     "rw_lateral_x","lw_lateral_x","rw_v_diff","lw_v_diff"]
        for k in show_keys:
            if k in debug:
                v = debug[k]
                txt = f"{k}={v:.1f}" if isinstance(v, float) else f"{k}={v}"
                col = C_GREEN if v is True else C_RED if v is False else C_GRAY
                cv2.putText(frame, txt, (8, dy),
                            cv2.FONT_HERSHEY_PLAIN, 0.72, col, 1, cv2.LINE_AA)
                dy += 15


def draw_guide(frame):
    guides = [
        ("T-POSE",    "Ambos brazos LATERALES",   (200, 80, 255)),
        ("UP",        "Ambos brazos ARRIBA",       ( 50, 230, 80)),
        ("DOWN",      "Brazos pegados al cuerpo",  ( 60,  60, 255)),
        ("LEFT",      "Brazo IZQ lateral",         (  0, 220, 255)),
        ("RIGHT",     "Brazo DER lateral",         (  0, 180, 255)),
        ("UP+LEFT",   "Izq arriba + Der lateral",  (255, 200,  50)),
        ("UP+RIGHT",  "Der arriba + Izq lateral",  (255, 100, 150)),
    ]
    x0, y0 = W - 290, 10
    cv2.rectangle(frame, (x0 - 6, y0 - 4), (W - 4, y0 + len(guides) * 18 + 8),
                  (16, 12, 28), -1)
    for i, (d, desc, col) in enumerate(guides):
        cv2.putText(frame, f"{d}: {desc}", (x0, y0 + i * 18 + 14),
                    cv2.FONT_HERSHEY_PLAIN, 0.76, col, 1, cv2.LINE_AA)


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Test detección de brazos con YOLOv8-pose")
    parser.add_argument("--source", default="0",
                        help="Fuente de video: 0,1,2 (cámara) o ruta a archivo")
    parser.add_argument("--model",
                        default=os.path.expanduser(
                            "~/robotis_ws/install/yaren_dice/share/yaren_dice/models/yolov8s-pose.pt"),
                        help="Ruta al modelo YOLO")
    parser.add_argument("--infer-size", type=int, default=320,
                        help="Tamaño de inferencia YOLO (320 o 640)")
    parser.add_argument("--mirror", action="store_true", default=True,
                        help="Espejo horizontal (para webcam estándar)")
    parser.add_argument("--no-mirror", dest="mirror", action="store_false")
    parser.add_argument("--debug", action="store_true", default=True,
                        help="Mostrar valores de debug")
    args = parser.parse_args()

    print(f"[INFO] Cargando modelo: {args.model}")
    try:
        from ultralytics import YOLO
        model = YOLO(args.model)
        print("[INFO] Modelo cargado ✓")
    except Exception as e:
        print(f"[ERROR] No se pudo cargar el modelo: {e}")
        return

    src = int(args.source) if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        print(f"[ERROR] No se pudo abrir la fuente: {args.source}")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print(f"[INFO] Cámara abierta: {args.source}")

    WIN = "Yaren - Test Detección Brazos"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, W, H)

    # ── Estado ───────────────────────────────────────────────────────────────
    current_pose   = frozenset()   # pose activa actual
    pose_hold_t    = 0.0           # tiempo que lleva esta pose activa
    confirmed      = False
    confirmed_pose = frozenset()
    confirmed_t    = 0.0
    confirm_flash  = 0.0
    HOLD_NEEDED    = 0.4

    last_time  = time.time()
    fps_smooth = 0.0
    mirror     = args.mirror
    show_debug = args.debug

    print("\n[INFO] Controles:")
    print("  Q / ESC → salir")
    print("  M       → toggle espejo")
    print("  D       → toggle debug")
    print("  R       → resetear estado\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        now = time.time()
        dt  = now - last_time
        last_time = now
        fps_smooth = fps_smooth * 0.9 + (1.0 / max(dt, 1e-4)) * 0.1

        if mirror:
            frame = cv2.flip(frame, 1)

        # ── Inferencia ───────────────────────────────────────────────────────
        results  = model(frame, imgsz=args.infer_size, verbose=False)
        kpts_raw = results[0].keypoints
        kpts     = None
        if kpts_raw is not None and len(kpts_raw.xy) > 0:
            kpts = kpts_raw.xy[0].cpu().numpy()

        # ── Detectar ─────────────────────────────────────────────────────────
        detected, debug_info = detect_direction(kpts)

        # ── Hold para confirmar ───────────────────────────────────────────────
        if detected == current_pose and detected:
            pose_hold_t += dt
        else:
            current_pose = detected
            pose_hold_t  = 0.0

        confirmed = (pose_hold_t >= HOLD_NEEDED and bool(detected))
        if confirmed and detected != confirmed_pose:
            confirmed_pose = detected
            confirmed_t    = now
            name = POSE_NAMES.get(detected, "+".join(sorted(detected)))
            print(f"[DETECTED] {name} ✓")

        confirm_flash = max(0.0, confirm_flash - dt * 2.0)
        if confirmed:
            confirm_flash = 1.0

        # ── Render ───────────────────────────────────────────────────────────
        vis = cv2.resize(frame, (W, H))
        overlay = np.zeros_like(vis)
        cv2.addWeighted(overlay, 0.35, vis, 0.65, 0, vis)

        if kpts is not None:
            h_orig, w_orig = frame.shape[:2]
            kpts_vis = kpts.copy()
            kpts_vis[:, 0] *= W / w_orig
            kpts_vis[:, 1] *= H / h_orig
            skel_color = POSE_COLORS.get(detected, C_GRAY)
            draw_skeleton(vis, kpts_vis, skel_color)

        draw_direction_indicators(vis, detected, pose_hold_t)
        draw_hud(vis, detected, fps_smooth,
                 debug_info if show_debug else {}, pose_hold_t, confirmed)
        draw_guide(vis)

        # Título
        cv2.putText(vis, "TEST BRAZOS - YAREN DANCE", (W // 2 - 160, 22),
                    cv2.FONT_HERSHEY_DUPLEX, 0.65, (180, 160, 220), 1, cv2.LINE_AA)

        # Flash de confirmación
        if confirm_flash > 0.01:
            col = POSE_COLORS.get(confirmed_pose, C_GRAY)
            ov2 = vis.copy()
            cv2.rectangle(ov2, (0, 0), (W, H), col, -1)
            cv2.addWeighted(ov2, confirm_flash * 0.15, vis, 1 - confirm_flash * 0.15, 0, vis)

        cv2.imshow(WIN, vis)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break
        elif key == ord('m'):
            mirror = not mirror
            print(f"[INFO] Espejo: {'ON' if mirror else 'OFF'}")
        elif key == ord('d'):
            show_debug = not show_debug
        elif key == ord('r'):
            current_pose = frozenset(); pose_hold_t = 0.0
            confirmed = False; confirmed_pose = frozenset()
            print("[INFO] Estado reseteado")

    cap.release()
    cv2.destroyAllWindows()
    print("[INFO] Saliendo.")


if __name__ == "__main__":
    main()