#!/usr/bin/env python3
"""robot_parts.py - filtro 'YAREN': brazos + pecho del robot sobre la persona; solo se ve la cara.

Recortes (una sola vez):  python3 robot_parts.py build izq.png der.png pecho.png assets/
Uso:
    from robot_parts import RobotFilter
    rf = RobotFilter("assets")
    out = rf.apply(frame_bgr, pose_px, vis, seg_mask, face_px, mirrored=True)
"""
import os
import sys

import cv2
import numpy as np

ARM_PIVOTS = {
    "izq": ((72, 75), (88, 222), (98, 345)),
    "der": ((95, 75), (88, 222), (80, 345)),
}
FACE_OVAL = [10, 338, 297, 332, 284, 251, 389, 356, 454, 323, 361, 288, 397, 365, 379,
             378, 400, 377, 152, 148, 176, 149, 150, 136, 172, 58, 132, 93, 234, 127,
             162, 21, 54, 103, 67, 109]


def _bg_mask(im):
    b = 6
    edge = np.vstack([im[:b].reshape(-1, 3), im[:, :b].reshape(-1, 3), im[:, -b:].reshape(-1, 3)])
    bg = np.median(edge, axis=0).astype(np.float32)
    m = (np.linalg.norm(im.astype(np.float32) - bg, axis=2) > 7).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))


def _yellow(im):
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    return ((hsv[..., 0] >= 12) & (hsv[..., 0] <= 40) & (hsv[..., 1] > 70) &
            (hsv[..., 2] > 90)).astype(np.uint8)


def _largest(m):
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    if n <= 1:
        return m
    return (lab == 1 + int(np.argmax(st[1:, 4]))).astype(np.uint8)


def _fill_holes(m, max_area=150):
    n, lab, st, _ = cv2.connectedComponentsWithStats(1 - m)
    for i in range(1, n):
        x, y, w, h, a = st[i]
        if a < max_area and x > 0 and y > 0:
            m[lab == i] = 1
    return m


def cut_arm(path):
    im = cv2.imread(path, cv2.IMREAD_COLOR)
    if im is None:
        raise SystemExit(f"No pude leer {path}")
    y = cv2.dilate(_yellow(im), np.ones((5, 5), np.uint8))
    m = _fill_holes(_largest(_bg_mask(im) & (1 - y)))
    a = cv2.GaussianBlur(m * 255, (3, 3), 0)
    return np.dstack([im, a])


def cut_chest(path):
    im = cv2.imread(path, cv2.IMREAD_COLOR)
    if im is None:
        raise SystemExit(f"No pude leer {path}")
    y = cv2.morphologyEx(_yellow(im), cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    m = _fill_holes(_largest(y), 400)
    a = cv2.GaussianBlur(m * 255, (3, 3), 0)
    xs, ys = np.where(m.T > 0)[0], np.where(m > 0)[0]
    rgba = np.dstack([im, a])
    return rgba[ys.min():ys.max() + 1, xs.min():xs.max() + 1]


def build(izq, der, pecho, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    cv2.imwrite(os.path.join(out_dir, "brazo_izq.png"), cut_arm(izq))
    cv2.imwrite(os.path.join(out_dir, "brazo_der.png"), cut_arm(der))
    cv2.imwrite(os.path.join(out_dir, "pecho.png"), cut_chest(pecho))
    print("recortes listos en", out_dir)


def _blend(frame, rgba, M):
    h, w = frame.shape[:2]
    warped = cv2.warpAffine(rgba, M, (w, h), flags=cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0, 0))
    a = warped[..., 3:4].astype(np.float32) / 255.0
    frame[:] = (frame * (1 - a) + warped[..., :3] * a).astype(np.uint8)


def _seg_transform(p0, p1, a, b, k=1.0):
    p0, p1, a, b = [np.asarray(v, np.float32) for v in (p0, p1, a, b)]
    v0, v1 = p1 - p0, b - a
    l0, l1 = np.linalg.norm(v0), np.linalg.norm(v1)
    if l0 < 1e-3 or l1 < 1.0:
        return None
    u0, u1 = v0 / l0, v1 / l1
    n0 = np.array([-u0[1], u0[0]], np.float32)
    n1 = np.array([-u1[1], u1[0]], np.float32)
    s = l1 / l0
    L = s * np.outer(u1, u0) + s * k * np.outer(n1, n0)
    t = a - L @ p0
    return np.hstack([L, t[:, None]]).astype(np.float32)


class RobotFilter:
    def __init__(self, assets_dir, bg_color=(235, 235, 235), min_vis=0.4,
                 arm_thick=0.75, chest_scale=0.8):
        rd = lambda n: cv2.imread(os.path.join(assets_dir, n), cv2.IMREAD_UNCHANGED)
        self.arm = {"izq": rd("brazo_izq.png"), "der": rd("brazo_der.png")}
        self.chest = rd("pecho.png")
        if any(v is None for v in (*self.arm.values(), self.chest)):
            raise FileNotFoundError(f"faltan recortes en {assets_dir} (corre 'build')")
        self.bg_color, self.min_vis = bg_color, min_vis
        self.arm_thick, self.chest_scale = arm_thick, chest_scale

    def _draw_arm(self, frame, side, sh, el, wr):
        (psh, pel, pwr) = ARM_PIVOTS[side]
        img = self.arm[side]
        elbow_y = pel[1]
        lower, upper = img.copy(), img.copy()
        lower[:elbow_y - 6, :, 3] = 0
        upper[elbow_y + 10:, :, 3] = 0
        M = _seg_transform(pel, pwr, el, wr, self.arm_thick)
        if M is not None:
            _blend(frame, lower, M)
        M = _seg_transform(psh, pel, sh, el, self.arm_thick)
        if M is not None:
            _blend(frame, upper, M)

    def _draw_chest(self, frame, ls, rs, lh, rh):
        h, w = self.chest.shape[:2]
        ls, rs, lh, rh = [np.asarray(v, np.float32) for v in (ls, rs, lh, rh)]
        ex = (rs - ls) * 0.12
        up = ((ls + rs) / 2 - (lh + rh) / 2) * 0.10
        dst = np.float32([rs + ex + up, ls - ex + up, lh, rh])
        c = dst.mean(axis=0)
        dst = (c + (dst - c) * self.chest_scale).astype(np.float32)
        src = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
        Hm = cv2.getPerspectiveTransform(src, dst)
        fh, fw = frame.shape[:2]
        wp = cv2.warpPerspective(self.chest, Hm, (fw, fh), flags=cv2.INTER_LINEAR,
                                 borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0, 0))
        a = wp[..., 3:4].astype(np.float32) / 255.0
        frame[:] = (frame * (1 - a) + wp[..., :3] * a).astype(np.uint8)

    def apply(self, frame, pose_px, vis, seg_mask=None, face_px=None, mirrored=True, bg=None):
        out = frame.copy()
        if seg_mask is not None:
            m = cv2.GaussianBlur((seg_mask > 0.5).astype(np.float32), (7, 7), 0)[..., None]
            back = bg if bg is not None else np.full_like(frame, self.bg_color)
            out = (frame * (1 - m) + back * m).astype(np.uint8)
        P = np.asarray(pose_px, np.float32)
        V = np.asarray(vis, np.float32)
        ok = lambda *ids: all(V[i] >= self.min_vis for i in ids)
        if ok(11, 12, 23, 24):
            self._draw_chest(out, P[11], P[12], P[23], P[24])
        pairs = (("izq", (11, 13, 15)), ("der", (12, 14, 16))) if mirrored else \
                (("izq", (12, 14, 16)), ("der", (11, 13, 15)))
        for side, (s, e, w) in pairs:
            if ok(s, e, w):
                self._draw_arm(out, side, P[s], P[e], P[w])
        if face_px is not None:
            F = np.asarray(face_px, np.float32)
            poly = F[FACE_OVAL].astype(np.int32)
            fm = np.zeros(frame.shape[:2], np.uint8)
            cv2.fillConvexPoly(fm, cv2.convexHull(poly), 255)
            fm = cv2.GaussianBlur(fm, (11, 11), 0).astype(np.float32)[..., None] / 255.0
            out = (out * (1 - fm) + frame * fm).astype(np.uint8)
        return out


if __name__ == "__main__":
    if len(sys.argv) == 6 and sys.argv[1] == "build":
        build(*sys.argv[2:6])
    else:
        print(__doc__)
