#!/usr/bin/env python3
"""Convierte imagenes de lineas en yaren_paint/drawings_levels.py.

Estructura:  imagenes/<nivel>/<nombre>.png   (nivel = medio | alto)
Nombre:      "una_tortuga.png" -> title "una tortuga", label "Tortuga", id "tortuga"
             (si no empieza con un/una/el/la se usa "un").
"""
import glob
import os
import sys

import cv2
import numpy as np

BASE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(BASE, "imagenes")
OUT = os.path.join(BASE, "yaren_paint", "drawings_levels.py")
LONG, THR, EPS = 1000, 150, 1.6          # lado largo, umbral de tinta, simplificacion


def load_ink(path):
    im = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if im is None:
        raise SystemExit(f"No pude leer {path}")
    if im.ndim == 2:
        im = cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)
    if im.shape[2] == 4:
        a = im[:, :, 3:4] / 255.0
        im = (im[:, :, :3] * a + 255 * (1 - a)).astype(np.uint8)
    g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    s = LONG / max(g.shape)
    g = cv2.resize(g, None, fx=s, fy=s, interpolation=cv2.INTER_CUBIC)
    ink = (g < THR).astype(np.uint8)
    return cv2.copyMakeBorder(ink, 10, 10, 10, 10, cv2.BORDER_CONSTANT, value=0)


def regions(ink, min_area=120):
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    closed = cv2.dilate(ink, k)                       # cierra huecos pequenos
    n, lab, st, _ = cv2.connectedComponentsWithStats(1 - closed, connectivity=4)
    bg, out = lab[0, 0], []
    for i in range(1, n):
        if i == bg or st[i, 4] < min_area:
            continue
        m = cv2.dilate((lab == i).astype(np.uint8), k)  # deshace la dilatacion
        cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        c = max(cs, key=cv2.contourArea)
        out.append((cv2.contourArea(c), c))
    out.sort(key=lambda r: -r[0])                     # grandes primero
    return out


def names(stem):
    parts = stem.split("_")
    has = parts[0] in ("un", "una", "el", "la")
    words = parts[1:] if has else parts
    name = " ".join(words)
    art = parts[0] if has else ("una" if words[-1].endswith("a") else "un")
    return "".join(words).lower(), name.capitalize(), f"{art} {name}"


def convert(path, level):
    ink = load_ink(path)
    h, w = ink.shape
    sc = min(760 / w, 560 / h)
    ox, oy = (800 - w * sc) / 2, (600 - h * sc) / 2
    shapes = []
    for _, c in regions(ink):
        pts = (c.astype(np.float32) * sc + np.array([[[ox, oy]]], np.float32))
        ap = cv2.approxPolyDP(pts, EPS, True)[:, 0, :]
        if len(ap) >= 3:
            shapes.append(ap)
    sid, label, title = names(os.path.splitext(os.path.basename(path))[0])
    la = cv2.resize(ink * 255, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA)
    m = np.zeros((600, 800), np.uint8)
    y0, x0 = int(round(oy)), int(round(ox))
    hh, ww = min(la.shape[0], 600 - y0), min(la.shape[1], 800 - x0)
    m[y0:y0 + hh, x0:x0 + ww] = la[:hh, :ww]
    os.makedirs(os.path.join(BASE, "lineart"), exist_ok=True)
    cv2.imwrite(os.path.join(BASE, "lineart", sid + ".png"), m)
    return sid, label, title, level, shapes


def main():
    items = []
    for level in ("medio", "alto"):
        for p in sorted(glob.glob(os.path.join(SRC, level, "*.*"))):
            if p.lower().endswith((".png", ".jpg", ".jpeg", ".webp")):
                items.append(convert(p, level))
                print(f"{level:6} {os.path.basename(p):28} {len(items[-1][4])} partes")
    lines = ['"""drawings_levels.py - GENERADO por img2drawings.py (no editar)."""', "", "EXTRA = ["]
    for sid, label, title, level, shapes in items:
        lines.append(f'    {{"id": {sid!r}, "label": {label!r}, "title": {title!r}, '
                     f'"level": {level!r}, "shapes": [')
        for i, ap in enumerate(shapes, 1):
            flat = ",".join(f"({int(round(x))},{int(round(y))})" for x, y in ap)
            lines.append(f'        ("poly", [{flat}], "parte {i}"),')
        lines.append("    ]},")
    lines.append("]")
    with open(OUT, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"Escrito {OUT} ({len(items)} dibujos)")


if __name__ == "__main__":
    sys.exit(main())
