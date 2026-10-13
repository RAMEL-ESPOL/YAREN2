#!/usr/bin/env python3
import sys
import os
os.environ["GST_PLUGIN_FEATURE_RANK"] = "nvvp9dec:0,nvav1dec:0,nvvp8dec:0"
import subprocess
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QPushButton, QStackedWidget,
                             QGridLayout, QFrame, QSlider, QSizePolicy, QGraphicsOpacityEffect, QScrollArea)
from PyQt5.QtCore import Qt, pyqtSignal, QUrl, QTimer, QPropertyAnimation, QEasingCurve, QPoint
from PyQt5.QtGui import QFont, QCursor, QPixmap, QPainter, QLinearGradient, QColor, QPen
from PyQt5.QtMultimedia import QMediaPlayer, QMediaContent
from PyQt5.QtMultimediaWidgets import QVideoWidget

ICONS = os.path.expanduser('~/robotis_ws/src/YAREN2/yaren_radio/iconos')

VIDEOS = [
    {'id':'vid_pollito',        'title':'Pollito Pio',           'source':'Canciones de la Granja', 'genre':'infantil', 'image':f'{ICONS}/pollitopio.png', 'color':'#FFB347', 'file':'pollito_pio.mp4'},
    {'id':'vid_gallina',        'title':'Gallina Turuleca',      'source':'Canciones de Yaren',     'genre':'infantil', 'image':f'{ICONS}/gallita turuleca.png', 'color':'#FF8C42', 'file':'gallina_turuleca.mp4'},
    {'id':'vid_vaca',           'title':'La Vaca Lola',          'source':'Canciones Infantiles',   'genre':'infantil', 'image':f'{ICONS}/VacaLola.png', 'color':'#6BCB77', 'file':'vaca_lola.mp4'},
    {'id':'vid_susanita',       'title':'Susanita',              'source':'La Granja de Zenón',     'genre':'infantil', 'image':f'{ICONS}/Susanita.png', 'color':'#F368E0', 'file':'susanita.mp4'},
    {'id':'vid_rapunzel',       'title':'Rapunzel',              'source':'Disney',                 'genre':'infantil', 'image':f'{ICONS}/rapunzel.png', 'color':'#F7B731', 'file':'rapunzel.mp4'},
    {'id':'vid_coco',           'title':'COCO',                  'source':'Pixar',                  'genre':'infantil', 'image':f'{ICONS}/Coco.png', 'color':'#FF6B6B', 'file':'COCO.mp4'},
    {'id':'vid_jovenes_titanes','title':'Jovenes Titanes',       'source':'Serie Animada DC',       'genre':'accion',   'image':f'{ICONS}/TeenTitan.png', 'color':'#6C5CE7', 'file':'jovenes_titanes.mp4'},
    {'id':'vid_spiderman',      'title':'Spider-Man',            'source':'Serie Clasica Marvel',   'genre':'accion',   'image':f'{ICONS}/Spectacular-Spiderman.png', 'color':'#FF4757', 'file':'spiderman_gran_poder.mp4'},
    {'id':'vid_superman_zod',   'title':'Superman vs Zod',       'source':'Man of Steel',           'genre':'accion',   'image':f'{ICONS}/superman.png', 'color':'#EE5A24', 'file':'superman_vs_zod.mp4'},
    {'id':'vid_shifu',          'title':'Shifu vs Tai Lung',     'source':'Kung Fu Panda',          'genre':'accion',   'image':f'{ICONS}/Kung-Fu-Panda.png', 'color':'#00B894', 'file':'shifu_vs_tai_lung.mp4'},
    {'id':'vid_ekko_jinx',      'title':'Ekko vs Jinx',          'source':'Arcane',                 'genre':'accion',   'image':f'{ICONS}/arcane.png', 'color':'#A29BFE', 'file':'ekko_vs_jinx.mp4'},
    {'id':'vid_ben10',          'title':'Ben 10',                'source':'Pelea Clasica',          'genre':'aventura', 'image':f'{ICONS}/ben10k.png', 'color':'#00D4A1', 'file':'ben10.mp4'},
    {'id':'vid_goku_trans',     'title':'Goku Transformaciones', 'source':'Dragon Ball Z',          'genre':'aventura', 'image':f'{ICONS}/gokuTransformaciones.png', 'color':'#FF9F43', 'file':'goku_transformaciones.mp4'},
    {'id':'vid_goku_ultra',     'title':'Goku Ultra Instinto',   'source':'Dragon Ball Super',      'genre':'aventura', 'image':f'{ICONS}/gokuUltra.png', 'color':'#48DBFB', 'file':'goku_ultra_instinto.mp4'},
    {'id':'vid_pockemon',       'title':'Pokemon',               'source':'Serie Animada',          'genre':'aventura', 'image':f'{ICONS}/Pokemon.png', 'color':'#F9CA24', 'file':'pockemon.mp4'},
    {'id':'vid_superman_padre', 'title':'Superman y Padre',      'source':'Man of Steel',           'genre':'emocion',  'image':f'{ICONS}/superman2025.png', 'color':'#3498DB', 'file':'superman_padre.mp4'},
    {'id':'vid_cars1',          'title':'Cars',                  'source':'Pixar',                  'genre':'animado',  'image':f'{ICONS}/Cars_1.png', 'color':'#E74C3C', 'file':'cars1.mp4'},
    {'id':'vid_yakko',          'title':'Yakko del Mundo',       'source':'Animaniacs',             'genre':'animado',  'image':f'{ICONS}/Yakko.png', 'color':'#9B59B6', 'file':'Yakko-Mundo.mp4'},
    {'id':'vid_golden_kpop',    'title':'Golden K-Pop',          'source':'K-Pop',                  'genre':'musical',  'image':f'{ICONS}/Golden-K-Pop.png', 'color':'#FFD700', 'file':'Golden-Kpop.mp4'},
    {'id':'vid_soda_pop',       'title':'Soda Pop',              'source':'Pop Latino',             'genre':'musical',  'image':f'{ICONS}/K-Pop-Demon-Hunters.png', 'color':'#FF6B9D', 'file':'Soda-Pop.mp4'},
    {'id':'vid_something_new',  'title':'Something New',         'source':'Pop',                    'genre':'musical',  'image':f'{ICONS}/High_School_Musical.png', 'color':'#A8E063', 'file':'Something-new.mp4'},
    {'id':'vid_your_idol',      'title':'Your Idol',             'source':'K-Pop',                  'genre':'musical',  'image':f'{ICONS}/YourIdol.png', 'color':'#FC5C7D', 'file':'Your-Idol.mp4'},
]

STYLESHEET = """
    QMainWindow { background-color: #000000; }
    QWidget { background: transparent; }
    QFrame#AppContainer { background: qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #1C2350,stop:1 #0B0F2A); }
    QLabel { color: #FFFFFF; background: transparent; }
    QSlider::groove:horizontal { border-radius: 5px; height: 10px; background: rgba(255,255,255,0.22); }
    QSlider::sub-page:horizontal { background: #6C8CFF; border-radius: 5px; }
    QSlider::handle:horizontal { background: #FFFFFF; border: 3px solid #6C8CFF; width: 28px; height: 28px; margin: -12px 0; border-radius: 17px; }
"""

# =============================================================================
#  INTRO SCREEN (estilo tablet: marino + tarjetas + barra inferior)
# =============================================================================
import math
from PyQt5.QtCore import QPointF, QRectF, QElapsedTimer
from PyQt5.QtGui import QRadialGradient, QPainterPath, QFontMetricsF, QBrush

MASCOT_PATH = os.path.expanduser('~/robotis_ws/src/YAREN2/yaren_face_display/icons/yaren.png')


def _clamp01(x):
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


def _seg(t, a, b):
    return _clamp01((t - a) / (b - a))


def _smooth(x):
    return x * x * (3.0 - 2.0 * x)


def _out_cubic(x):
    return 1.0 - (1.0 - x) ** 3


def _out_back(x):
    c1 = 1.70158
    c3 = c1 + 1.0
    return 1.0 + c3 * (x - 1.0) ** 3 + c1 * (x - 1.0) ** 2


def mascot_circle(size):
    pm = QPixmap(size, size)
    pm.fill(Qt.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.Antialiasing)
    p.setRenderHint(QPainter.SmoothPixmapTransform)
    path = QPainterPath()
    path.addEllipse(QRectF(3, 3, size - 6, size - 6))
    p.fillPath(path, QColor("#3A4A9F"))
    img = QPixmap(MASCOT_PATH)
    p.setClipPath(path)
    if not img.isNull():
        side = int(size * 0.84)
        img = img.scaled(side, side, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        p.drawPixmap((size - img.width()) // 2, (size - img.height()) // 2, img)
    else:
        p.setPen(Qt.NoPen)
        p.setBrush(QColor("white"))
        p.drawEllipse(QPointF(size * 0.36, size * 0.46), size * 0.07, size * 0.09)
        p.drawEllipse(QPointF(size * 0.64, size * 0.46), size * 0.07, size * 0.09)
    p.setClipping(False)
    p.setPen(QPen(QColor("white"), 3))
    p.setBrush(Qt.NoBrush)
    p.drawEllipse(QRectF(3, 3, size - 6, size - 6))
    p.end()
    return pm


def _out_elastic(x):
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    return 2.0 ** (-10.0 * x) * math.sin((x * 10.0 - 0.75) * (2.0 * math.pi) / 3.0) + 1.0


class IntroScreen(QWidget):
    """Intro: robot + YAREN TV con animacion. Abajo: 'Dale para continuar'."""
    confirmed = pyqtSignal()

    W, H = 800.0, 480.0
    PROMPT = "Dale para continuar"
    T_PROMPT = 2.6
    T_END = 3.0
    ACCENT = "#6C8CFF"
    RX, RY = 400.0, 165.0          # centro del robot
    TY = 338.0                     # linea central del titulo
    SPARK = ["#FFD93D", "#FFFFFF", "#6C8CFF", "#FF8FB8", "#7CF2D0", "#FFFFFF"]

    def __init__(self, parent=None):
        super().__init__(parent)
        self._clock = QElapsedTimer()
        self._offset = 0.0
        self._t = 0.0
        self._timer = QTimer(self)
        self._timer.setInterval(16)
        self._timer.timeout.connect(self._tick)
        self._robot = QPixmap(MASCOT_PATH)
        if not self._robot.isNull():
            self._robot = self._robot.scaledToHeight(230, Qt.SmoothTransformation)
        f = QFont("Arial")
        f.setPixelSize(120)
        f.setBold(True)
        fm = QFontMetricsF(f)
        full = QPainterPath()
        full.addText(0, 0, f, "YAREN TV")
        br = full.boundingRect()
        self._tw = br.width()
        self._k = 500.0 / self._tw
        self._letters = []
        x = 0.0
        for ch in "YAREN TV":
            adv = fm.horizontalAdvance(ch)
            pth = None
            if ch != " ":
                pth = QPainterPath()
                pth.addText(-adv / 2.0, -br.center().y(), f, ch)
            self._letters.append((pth, x + adv / 2.0 - br.center().x()))
            x += adv
        self._pf = QFont("Arial")
        self._pf.setPixelSize(19)
        self._pf.setBold(True)

    def showEvent(self, e):
        self._offset = 0.0
        self._clock.start()
        self._timer.start()
        super().showEvent(e)

    def hideEvent(self, e):
        self._timer.stop()
        super().hideEvent(e)

    def mousePressEvent(self, e):
        if self._t >= self.T_PROMPT:
            self.confirmed.emit()
        else:
            self._offset += self.T_END - self._t
            self._tick()

    def _tick(self):
        self._t = self._offset + self._clock.elapsed() / 1000.0
        self.update()

    # ---------------------------------------------------------------- fondo
    def _bg(self, p, t):
        g = QLinearGradient(0, 0, 0, self.H)
        g.setColorAt(0, QColor("#1C2350"))
        g.setColorAt(1, QColor("#0B0F2A"))
        p.fillRect(QRectF(0, 0, self.W, self.H), QBrush(g))
        for (cx, cy, r, col, ph) in ((110, 80, 240, (76, 110, 255), 0.0),
                                     (700, 420, 270, (150, 90, 255), 1.7)):
            x = cx + math.sin(t * 0.6 + ph) * 26
            y = cy + math.cos(t * 0.5 + ph) * 20
            rg = QRadialGradient(QPointF(x, y), r)
            rg.setColorAt(0, QColor(col[0], col[1], col[2], 75))
            rg.setColorAt(1, QColor(col[0], col[1], col[2], 0))
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(rg))
            p.drawEllipse(QPointF(x, y), r, r)

    def _rings(self, p, t):
        a = _seg(t, 0.0, 0.6)
        if a > 0.0:
            pulse = 1.0 + 0.05 * math.sin(t * 2.0)
            r = 175.0 * a * pulse
            rg = QRadialGradient(QPointF(self.RX, self.RY), r)
            rg.setColorAt(0, QColor(108, 140, 255, 120))
            rg.setColorAt(0.6, QColor(108, 140, 255, 45))
            rg.setColorAt(1, QColor(108, 140, 255, 0))
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(rg))
            p.drawEllipse(QPointF(self.RX, self.RY), r, r)
        for i in range(3):
            t0 = 0.5 + i * 0.8
            if t < t0:
                continue
            ph = ((t - t0) % 2.4) / 2.4
            r = 85.0 + 210.0 * _out_cubic(ph)
            al = int(110 * (1.0 - ph))
            p.setBrush(Qt.NoBrush)
            p.setPen(QPen(QColor(150, 175, 255, al), 3))
            p.drawEllipse(QPointF(self.RX, self.RY), r, r * 0.92)

    # ---------------------------------------------------------------- robot
    def _star(self, p, c, r, col, al):
        path = QPainterPath()
        pts = []
        for i in range(8):
            rr = r if i % 2 == 0 else r * 0.32
            a = i * math.pi / 4.0 - math.pi / 2.0
            pts.append(QPointF(c.x() + rr * math.cos(a), c.y() + rr * math.sin(a)))
        path.moveTo(pts[0])
        for q in pts[1:]:
            path.lineTo(q)
        path.closeSubpath()
        cc = QColor(col)
        cc.setAlpha(int(255 * al))
        p.setPen(Qt.NoPen)
        p.setBrush(cc)
        p.drawPath(path)

    def _fallback_robot(self, p):
        p.setPen(QPen(QColor("#C9702A"), 3))
        p.setBrush(QColor("#FFC93D"))
        p.drawRoundedRect(QRectF(-95, -80, 190, 150), 56, 56)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor("#14183A"))
        p.drawRoundedRect(QRectF(-70, -52, 140, 90), 34, 34)
        p.setBrush(QColor("white"))
        p.drawEllipse(QPointF(-28, -8), 13, 17)
        p.drawEllipse(QPointF(28, -8), 13, 17)

    def _robot_draw(self, p, t):
        a = _clamp01(_seg(t, 0.1, 0.35))
        if a <= 0.0:
            return
        e = _out_back(_seg(t, 0.1, 0.85))
        dy = -320.0 * (1.0 - e)
        land = _seg(t, 0.75, 1.15)
        sq = math.sin(math.pi * land) * 0.07 if 0.0 < land < 1.0 else 0.0
        fl = _seg(t, 1.1, 1.6)
        bob = math.sin(t * 2.2) * 7.0 * fl
        tilt = math.sin(t * 1.3) * 2.5 * fl
        # sombra
        sw = 105.0 - (bob * -1.0) * 1.4 - max(0.0, -dy) * 0.2
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(0, 0, 0, int(80 * a)))
        p.drawEllipse(QPointF(self.RX, self.RY + 128), max(30.0, sw), 11)
        p.save()
        p.setOpacity(a)
        p.translate(self.RX, self.RY + dy + bob)
        p.rotate(tilt)
        p.scale(1.0 + sq, 1.0 - sq)
        if self._robot.isNull():
            self._fallback_robot(p)
        else:
            w, h = self._robot.width(), self._robot.height()
            p.drawPixmap(QPointF(-w / 2.0, -h / 2.0), self._robot)
        p.restore()

    def _sparks(self, p, t):
        for i in range(12):
            ap = _seg(t, 0.8 + 0.04 * i, 1.2 + 0.04 * i)
            if ap <= 0.0:
                continue
            ang = i * 2.0 * math.pi / 12.0 + t * (0.7 if i % 2 else -0.55)
            rx = 195.0 + 22.0 * math.sin(i * 1.7 + t)
            ry = 112.0 + 12.0 * math.cos(i * 1.3 + t * 1.1)
            c = QPointF(self.RX + rx * math.cos(ang), self.RY + 8 + ry * math.sin(ang))
            tw = 0.5 + 0.5 * math.sin(t * 3.2 + i)
            self._star(p, c, (4.0 + 5.0 * (i % 3)) * (0.55 + 0.6 * tw) * ap,
                       self.SPARK[i % len(self.SPARK)], 0.35 + 0.65 * tw)

    # ---------------------------------------------------------------- titulo
    def _title(self, p, t):
        k = self._k
        tw = self._tw
        u = ((t - 1.9) % 4.0) / 0.9 if t >= 1.9 else 2.0
        bc = (-tw / 2.0 - 110.0 + (tw + 220.0) * u) if u <= 1.0 else None
        idle = _seg(t, 2.0, 2.5)
        for j, (pth, x) in enumerate(self._letters):
            if pth is None:
                continue
            q = _seg(t, 0.9 + 0.07 * j, 1.35 + 0.07 * j)
            if q <= 0.0:
                continue
            e = _out_back(q)
            base = QColor("#FFD93D") if j >= 6 else QColor("white")
            y = self.TY + 70.0 * (1.0 - e) + math.sin(t * 3.0 - j * 0.6) * 3.0 * idle
            rot = (-16.0 if j % 2 else 16.0) * (1.0 - e)
            p.save()
            p.setOpacity(_clamp01(q * 3.0))
            p.translate(400.0 + x * k, y)
            p.rotate(rot)
            p.scale(max(k * e, 0.001), max(k * e, 0.001))
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(0, 0, 0, 95))
            p.save()
            p.translate(0, 7)
            p.drawPath(pth)
            p.restore()
            if bc is not None:
                loc = bc - x
                gr = QLinearGradient(loc - 70, -35, loc + 70, 35)
                gr.setColorAt(0.0, base)
                gr.setColorAt(0.5, QColor(255, 255, 255))
                gr.setColorAt(1.0, base)
                p.setBrush(QBrush(gr))
            else:
                p.setBrush(base)
            p.drawPath(pth)
            p.restore()
        u2 = _smooth(_seg(t, 1.5, 2.0))
        if u2 > 0.0:
            p.setPen(QPen(QColor(self.ACCENT), 5, Qt.SolidLine, Qt.RoundCap))
            p.drawLine(QPointF(400 - 150 * u2, self.TY + 52), QPointF(400 + 150 * u2, self.TY + 52))

    # ---------------------------------------------------------------- boton
    def _prompt(self, p, t):
        a = _seg(t, 2.2, 2.7)
        if a <= 0.0:
            return
        pulse = 1.0 + 0.045 * math.sin(t * 4.0)
        y = 438 + (1.0 - _out_cubic(a)) * 30
        p.save()
        p.setOpacity(a)
        p.translate(400, y)
        p.scale(pulse, pulse)
        p.setPen(QPen(QColor(self.ACCENT), 3))
        p.setBrush(QColor(255, 255, 255, 240))
        p.drawRoundedRect(QRectF(-150, -22, 300, 44), 22, 22)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor("#FFD93D"))
        p.drawEllipse(QPointF(-120, 0), 15, 15)
        p.setBrush(QColor("#3B2A00"))
        tri = QPainterPath()
        tri.moveTo(-124, -7)
        tri.lineTo(-124, 7)
        tri.lineTo(-113, 0)
        tri.closeSubpath()
        p.drawPath(tri)
        p.setFont(self._pf)
        p.setPen(QColor("#2B3A8F"))
        p.drawText(QRectF(-100, -20, 250, 40), Qt.AlignVCenter | Qt.AlignHCenter, self.PROMPT)
        p.restore()

    def paintEvent(self, event):
        t = self._t
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.TextAntialiasing)
        p.setRenderHint(QPainter.SmoothPixmapTransform)
        p.scale(self.width() / self.W, self.height() / self.H)
        self._bg(p, t)
        self._rings(p, t)
        self._robot_draw(p, t)
        self._sparks(p, t)
        self._title(p, t)
        self._prompt(p, t)
        p.end()


def style_player_overlay(ov, top):
    ov.setStyleSheet("QFrame#OverlayBar{background:rgba(14,19,48,0.92);"
                     "border-top-left-radius:28px;border-top-right-radius:28px;}")
    top.setStyleSheet("QFrame#TopOverlayBar{background:rgba(14,19,48,0.88);"
                      "border-bottom-left-radius:28px;border-bottom-right-radius:28px;}")
    top.lbl_title.setStyleSheet("color:white;font-size:22px;font-weight:bold;background:transparent;")
    css = ("QPushButton{background:#2A2F55;color:white;border:none;border-radius:30px;"
           "font-size:22px;font-weight:bold;} QPushButton:pressed{background:#4C6BFF;}")
    for b in (ov.btn_prev, ov.btn_next):
        b.setFixedSize(60, 60)
        b.setStyleSheet(css)
    ov.btn_play.setFixedSize(76, 76)
    ov.btn_play.setStyleSheet("QPushButton{background:#6C8CFF;color:white;border:none;border-radius:38px;"
                              "font-size:30px;font-weight:bold;} QPushButton:pressed{background:#4C6BFF;}")
    ov.btn_catalog.setText("🏠 Inicio")
    ov.btn_catalog.setStyleSheet("QPushButton{background:#2A2F55;color:white;border:2px solid #6C8CFF;"
                                 "border-radius:24px;font-size:17px;font-weight:bold;padding:10px 22px;}"
                                 "QPushButton:pressed{background:#4C6BFF;}")


import json
STATE_PATH = os.path.expanduser("~/.yaren_tv_state.json")
VID_BY_ID = {v['id']: v for v in VIDEOS}


def _load_state():
    try:
        with open(STATE_PATH, encoding="utf-8") as f:
            d = json.load(f)
        return {"recent": [i for i in d.get("recent", []) if i in VID_BY_ID],
                "fav": [i for i in d.get("fav", []) if i in VID_BY_ID]}
    except Exception:
        return {"recent": [], "fav": []}


STATE = _load_state()


def _save_state():
    try:
        with open(STATE_PATH, "w", encoding="utf-8") as f:
            json.dump(STATE, f)
    except Exception:
        pass


def add_recent(vid):
    if vid in STATE["recent"]:
        STATE["recent"].remove(vid)
    STATE["recent"].insert(0, vid)
    del STATE["recent"][12:]
    _save_state()


def is_fav(vid):
    return vid in STATE["fav"]


def toggle_fav(vid):
    if vid in STATE["fav"]:
        STATE["fav"].remove(vid)
    else:
        STATE["fav"].append(vid)
    _save_state()


def format_time(ms):
    s = (ms // 1000) % 60
    m = (ms // 60000) % 60
    h = (ms // 3600000)
    return f"{h:02d}:{m:02d}:{s:02d}" if h > 0 else f"{m:02d}:{s:02d}"


class JumpSlider(QSlider):
    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            val = self.minimum() + int((self.maximum() - self.minimum()) * e.x() / self.width())
            self.setValue(val)
            self.sliderMoved.emit(val)
            e.accept()
        else:
            super().mousePressEvent(e)

    def mouseMoveEvent(self, e):
        if e.buttons() & Qt.LeftButton:
            val = self.minimum() + int((self.maximum() - self.minimum()) * e.x() / self.width())
            val = max(self.minimum(), min(val, self.maximum()))
            self.setValue(val)
            self.sliderMoved.emit(val)
            e.accept()
        else:
            super().mouseMoveEvent(e)


def circle_pixmap(vd, size):
    pm = QPixmap(size, size)
    pm.fill(Qt.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.Antialiasing)
    p.setRenderHint(QPainter.SmoothPixmapTransform)
    p.setPen(QPen(QColor("white"), 4))
    p.setBrush(QColor(vd.get('color', '#2F73D2')))
    p.drawEllipse(3, 3, size - 6, size - 6)
    img = QPixmap(vd.get('image', ''))
    if not img.isNull():
        inner = int(size * 0.66)
        img = img.scaled(inner, inner, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        p.drawPixmap((size - img.width()) // 2, (size - img.height()) // 2, img)
    p.end()
    return pm


class DragScroll(QScrollArea):
    tapped = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setFrameShape(QFrame.NoFrame)
        self.viewport().setStyleSheet("background:transparent;")
        self.setStyleSheet("QScrollArea{background:transparent;}")
        self._down = None
        self._moved = False
        self._start = 0

    def mousePressEvent(self, e):
        self._down = e.pos()
        self._start = self.horizontalScrollBar().value()
        self._moved = False

    def mouseMoveEvent(self, e):
        if self._down is None:
            return
        dx = e.pos().x() - self._down.x()
        if abs(dx) > 10:
            self._moved = True
        if self._moved:
            self.horizontalScrollBar().setValue(self._start - dx)

    def mouseReleaseEvent(self, e):
        if self._down is not None and not self._moved:
            w = self.widget()
            ch = w.childAt(w.mapFrom(self.viewport(), e.pos())) if w else None
            if ch is not None:
                vd = next((v for v in VIDEOS if v['id'] == ch.objectName()), None)
                if vd:
                    self.tapped.emit(vd)
        self._down = None


def _blend(c, other, k):
    a, b = QColor(c), QColor(other)
    return QColor(int(a.red() * (1 - k) + b.red() * k),
                  int(a.green() * (1 - k) + b.green() * k),
                  int(a.blue() * (1 - k) + b.blue() * k))


class VideoCard(QFrame):
    clicked = pyqtSignal(dict)
    fav_toggled = pyqtSignal(str)

    def __init__(self, vd):
        super().__init__()
        self.video_data = vd
        self.setObjectName("VCard")
        self.setCursor(QCursor(Qt.PointingHandCursor))
        self.setFixedSize(212, 250)
        base = _blend(vd.get('color', '#2F73D2'), '#2B3A8F', 0.62)
        c1 = base.lighter(112)
        c2 = base.darker(160)
        self.setStyleSheet(
            f"QFrame#VCard{{background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 {c1.name()},stop:1 {c2.name()});"
            "border:2px solid rgba(255,255,255,0.35);border-radius:24px;}")

        def child(text, geo, css):
            l = QLabel(text, self)
            l.setGeometry(*geo)
            l.setStyleSheet(css)
            l.setAttribute(Qt.WA_TransparentForMouseEvents)
            return l

        child("", (10, 10, 192, 136), "background:rgba(255,255,255,0.14);border-radius:18px;")
        img = child("", (10, 10, 192, 136), "background:transparent;font-size:40px;")
        img.setAlignment(Qt.AlignCenter)
        pm = QPixmap(vd.get('image', ''))
        if not pm.isNull():
            img.setPixmap(pm.scaled(168, 118, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        else:
            img.setText("🎬")

        longest = max(len(w) for w in vd['title'].split())
        fs = 16 if longest <= 9 else (14 if longest <= 12 else 12)
        ti = child(vd['title'], (16, 152, 180, 42),
                   f"color:white;font-size:{fs}px;font-weight:bold;background:transparent;")
        ti.setWordWrap(True)
        ti.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        child(vd['source'], (16, 208, 136, 22), "color:rgba(255,255,255,0.72);font-size:12px;background:transparent;")

        play = child("▶", (160, 200, 38, 38),
                     f"background:white;color:{c2.name()};border-radius:19px;font-size:16px;font-weight:bold;")
        play.setAlignment(Qt.AlignCenter)

        fav = is_fav(vd['id'])
        heart = QPushButton("♥" if fav else "♡", self)
        heart.setGeometry(18, 18, 32, 32)
        heart.setCursor(QCursor(Qt.PointingHandCursor))
        heart.setStyleSheet("QPushButton{background:white;color:%s;border:none;border-radius:16px;font-size:17px;}"
                            % ('#F2587A' if fav else '#9AA3B5'))
        heart.clicked.connect(lambda: self.fav_toggled.emit(vd['id']))

    def mousePressEvent(self, e):
        self.clicked.emit(self.video_data)


class TopOverlayBar(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("TopOverlayBar")
        self.setFixedHeight(70)
        self._opacity = QGraphicsOpacityEffect(self)
        self._opacity.setOpacity(1.0)
        self.setGraphicsEffect(self._opacity)
        self._anim = QPropertyAnimation(self._opacity, b"opacity")
        self._anim.setDuration(250)
        self._anim.finished.connect(self._on_anim_finished)
        self._hide_timer = QTimer(self)
        self._hide_timer.setSingleShot(True)
        self._hide_timer.timeout.connect(self.fade_out)
        lay = QVBoxLayout(self)
        self.lbl_title = QLabel("")
        self.lbl_title.setStyleSheet("color: white; font-size: 24px; font-weight: bold; background: transparent;")
        self.lbl_title.setAlignment(Qt.AlignCenter)
        lay.addWidget(self.lbl_title)

    def _on_anim_finished(self):
        if self._anim.endValue() == 0.0:
            self.hide()

    def fade_in(self):
        if not self.isVisible(): self.show()
        self._anim.stop(); self._anim.setEndValue(1.0); self._anim.start()

    def fade_out(self):
        self._anim.stop(); self._anim.setEndValue(0.0); self._anim.start()

    def show_temporarily(self, ms=3500):
        self.fade_in(); self._hide_timer.start(ms)

    def keep_visible(self):
        self.fade_in(); self._hide_timer.stop()


class OverlayBar(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("OverlayBar")
        self.setFixedHeight(150)
        self._opacity = QGraphicsOpacityEffect(self)
        self._opacity.setOpacity(1.0)
        self.setGraphicsEffect(self._opacity)
        self._anim = QPropertyAnimation(self._opacity, b"opacity")
        self._anim.setDuration(250)
        self._anim.finished.connect(self._on_anim_finished)
        self._hide_timer = QTimer(self)
        self._hide_timer.setSingleShot(True)
        self._hide_timer.timeout.connect(self.fade_out)
        self._build_ui()

    def _build_ui(self):
        main_lay = QVBoxLayout(self)
        main_lay.setContentsMargins(30, 15, 30, 20)
        time_lay = QHBoxLayout()
        self.timeline_slider = JumpSlider(Qt.Horizontal)
        self.timeline_slider.setCursor(QCursor(Qt.PointingHandCursor))
        self.lbl_time = QLabel("00:00 / 00:00")
        self.lbl_time.setStyleSheet("color: white; font-size: 16px; font-weight: bold; background: transparent;")
        time_lay.addWidget(self.timeline_slider); time_lay.addSpacing(15); time_lay.addWidget(self.lbl_time)
        ctrl_lay = QHBoxLayout()
        left_lay = QHBoxLayout()
        self.lbl_vol_icon = QLabel("🔊")
        self.lbl_vol_icon.setStyleSheet("color: white; font-size: 24px; background: transparent;")
        self.vol_slider = JumpSlider(Qt.Horizontal)
        self.vol_slider.setRange(0, 100); self.vol_slider.setValue(80)
        self.vol_slider.setFixedWidth(160); self.vol_slider.setCursor(QCursor(Qt.PointingHandCursor))
        left_lay.addWidget(self.lbl_vol_icon); left_lay.addWidget(self.vol_slider); left_lay.addStretch()
        center_lay = QHBoxLayout()
        self.btn_prev = QPushButton("◄◄"); self.btn_play = QPushButton("►"); self.btn_next = QPushButton("►►")
        self.btn_prev.setProperty("class", "OvBtn"); self.btn_prev.setCursor(QCursor(Qt.PointingHandCursor))
        self.btn_next.setProperty("class", "OvBtn"); self.btn_next.setCursor(QCursor(Qt.PointingHandCursor))
        self.btn_play.setProperty("class", "OvBtnPrimary")
        self.btn_play.setProperty("state", "play")
        self.btn_play.setCursor(QCursor(Qt.PointingHandCursor))
        self.btn_prev.setFixedSize(70, 70); self.btn_play.setFixedSize(80, 80); self.btn_next.setFixedSize(70, 70)
        center_lay.addWidget(self.btn_prev); center_lay.addWidget(self.btn_play); center_lay.addWidget(self.btn_next)
        right_lay = QHBoxLayout()
        self.btn_catalog = QPushButton("🏠 Home")
        self.btn_catalog.setProperty("class", "OvCatalogBtn"); self.btn_catalog.setCursor(QCursor(Qt.PointingHandCursor))
        right_lay.addStretch(); right_lay.addWidget(self.btn_catalog)
        ctrl_lay.addLayout(left_lay, 1); ctrl_lay.addLayout(center_lay, 1); ctrl_lay.addLayout(right_lay, 1)
        main_lay.addLayout(time_lay); main_lay.addSpacing(10); main_lay.addLayout(ctrl_lay)

    def _on_anim_finished(self):
        if self._anim.endValue() == 0.0: self.hide()

    def fade_in(self):
        if not self.isVisible(): self.show()
        self._anim.stop(); self._anim.setEndValue(1.0); self._anim.start()

    def fade_out(self):
        self._anim.stop(); self._anim.setEndValue(0.0); self._anim.start()

    def show_temporarily(self, ms=3500):
        self.fade_in(); self._hide_timer.start(ms)

    def keep_visible(self):
        self.fade_in(); self._hide_timer.stop()


class PlayerView(QWidget):
    request_catalog = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)
        self.playlist = []
        self.current_idx = 0
        self._last_mouse_pos = None
        self.media_player = QMediaPlayer(None, QMediaPlayer.VideoSurface)
        self.video_widget = QVideoWidget(self)
        self.video_widget.setStyleSheet("background: black;")
        self.media_player.setVideoOutput(self.video_widget)
        self.video_widget.setMouseTracking(True)
        self.video_widget.installEventFilter(self)
        self.media_player.stateChanged.connect(self._on_state_changed)
        self.media_player.positionChanged.connect(self._on_position_changed)
        self.media_player.durationChanged.connect(self._on_duration_changed)
        self.media_player.mediaStatusChanged.connect(self._on_status_changed)
        self.top_overlay = TopOverlayBar(self)
        self.overlay = OverlayBar(self)
        self.btn_fav = QPushButton("♡", self.top_overlay)
        self.btn_fav.setFixedSize(46, 46)
        self.btn_fav.setCursor(QCursor(Qt.PointingHandCursor))
        self.btn_fav.clicked.connect(self._toggle_fav_current)
        self.overlay.btn_play.clicked.connect(self.toggle_play)
        self.overlay.btn_prev.clicked.connect(self.play_prev)
        self.overlay.btn_next.clicked.connect(self.play_next)
        self.overlay.btn_catalog.clicked.connect(self.request_catalog)
        style_player_overlay(self.overlay, self.top_overlay)
        self.overlay.timeline_slider.sliderMoved.connect(lambda v: [self.set_position(v), self._wake_ui()])
        self.overlay.vol_slider.sliderMoved.connect(lambda v: [self.set_volume(v), self._wake_ui()])
        self.center_osd = QLabel(self)
        self.center_osd.setStyleSheet("QLabel { background-color: rgba(0,0,0,150); color: white; border-radius: 40px; }")
        self.center_osd.setAlignment(Qt.AlignCenter)
        self.center_osd.hide()
        self.osd_opacity = QGraphicsOpacityEffect(self.center_osd)
        self.center_osd.setGraphicsEffect(self.osd_opacity)
        self.osd_anim = QPropertyAnimation(self.osd_opacity, b"opacity")
        self.osd_anim.setDuration(400)
        self.osd_anim.finished.connect(self.center_osd.hide)
        self._cursor_timer = QTimer(self)
        self._cursor_timer.setSingleShot(True)
        self._cursor_timer.timeout.connect(lambda: self.setCursor(QCursor(Qt.BlankCursor)))

    def resizeEvent(self, e):
        self.video_widget.setGeometry(0, 0, self.width(), self.height())
        self.top_overlay.setGeometry(0, 0, self.width(), 70)
        if hasattr(self, 'btn_fav'): self.btn_fav.move(self.width() - 66, 12)
        self.overlay.setGeometry(0, self.height() - 150, self.width(), 150)
        self.center_osd.setGeometry((self.width() - 80) // 2, (self.height() - 80) // 2, 80, 80)
        super().resizeEvent(e)

    def _wake_ui(self):
        self.setCursor(QCursor(Qt.ArrowCursor))
        if self.media_player.state() != QMediaPlayer.PausedState:
            self.overlay.show_temporarily(3500)
            self.top_overlay.show_temporarily(3500)
            self._cursor_timer.start(3500)
        else:
            self.overlay.keep_visible()
            self.top_overlay.keep_visible()
            self._cursor_timer.stop()

    def eventFilter(self, source, event):
        if event.type() == event.MouseMove:
            if event.pos() != self._last_mouse_pos:
                self._last_mouse_pos = event.pos()
                self._wake_ui()
        elif event.type() == event.MouseButtonPress and source == self.video_widget:
            self.toggle_play(); self._wake_ui()
        return super().eventFilter(source, event)

    def keyPressEvent(self, e):
        self._wake_ui()
        if e.key() == Qt.Key_Space: self.toggle_play()
        elif e.key() == Qt.Key_Right: self.media_player.setPosition(self.media_player.position() + 10000)
        elif e.key() == Qt.Key_Left: self.media_player.setPosition(self.media_player.position() - 10000)
        elif e.key() == Qt.Key_Escape: self.stop(); self.request_catalog.emit()
        else: super().keyPressEvent(e)

    def load_playlist(self, playlist, start_idx):
        self.playlist = playlist; self.current_idx = start_idx; self._play_current()

    def _play_current(self):
        if not self.playlist: return
        vd = self.playlist[self.current_idx]
        add_recent(vd['id'])
        path = os.path.join(os.path.expanduser("~/robotis_ws/src/YAREN2/yaren_radio/videos"), vd['file'])
        self.media_player.stop()
        self.media_player.setMedia(QMediaContent(QUrl.fromLocalFile(path)))
        self.media_player.play()
        self.top_overlay.lbl_title.setText(f"{vd['title']} • {vd['source']}")
        self._refresh_fav_btn()
        self._wake_ui(); self.setFocus()

    def _refresh_fav_btn(self):
        if not self.playlist:
            return
        fav = is_fav(self.playlist[self.current_idx]['id'])
        self.btn_fav.setText("♥" if fav else "♡")
        self.btn_fav.setStyleSheet("QPushButton{background:white;color:%s;border:none;border-radius:23px;"
                                   "font-size:24px;}" % ('#F2587A' if fav else '#9AA3B5'))

    def _toggle_fav_current(self):
        if self.playlist:
            toggle_fav(self.playlist[self.current_idx]['id'])
            self._refresh_fav_btn()
            self._wake_ui()

    def play_next(self):
        if self.playlist:
            self.current_idx = (self.current_idx + 1) % len(self.playlist)
            self._play_current()

    def play_prev(self):
        if self.playlist:
            self.current_idx = (self.current_idx - 1) % len(self.playlist)
            self._play_current()

    def toggle_play(self):
        if self.media_player.state() == QMediaPlayer.PlayingState:
            self.media_player.pause()
            self._show_osd("<span style='font-size:38px;'>❚❚</span>")
        else:
            self.media_player.play()
            self._show_osd("<span style='font-size:50px;'>►</span>")

    def _show_osd(self, text):
        self.center_osd.setText(text); self.center_osd.show(); self.center_osd.repaint()
        self.osd_anim.stop(); self.osd_anim.setStartValue(1.0); self.osd_anim.setEndValue(0.0); self.osd_anim.start()

    def stop(self): self.media_player.stop()

    def set_volume(self, v):
        self.media_player.setVolume(v)
        subprocess.Popen(["pactl", "set-sink-volume", "@DEFAULT_SINK@", f"{v}%"],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def set_position(self, position): self.media_player.setPosition(position)

    def _on_state_changed(self, state):
        if state == QMediaPlayer.PlayingState:
            self.overlay.btn_play.setText("❚❚"); self.overlay.btn_play.setProperty("state", "pause")
        else:
            self.overlay.btn_play.setText("►"); self.overlay.btn_play.setProperty("state", "play")
        self.overlay.btn_play.style().unpolish(self.overlay.btn_play)
        self.overlay.btn_play.style().polish(self.overlay.btn_play)
        self._wake_ui(); self.overlay.repaint(); self.top_overlay.repaint()

    def _on_position_changed(self, position):
        is_dragging = QApplication.mouseButtons() & Qt.LeftButton and self.overlay.timeline_slider.underMouse()
        if not is_dragging: self.overlay.timeline_slider.setValue(position)
        dur = max(self.media_player.duration(), 1)
        self.overlay.lbl_time.setText(f"{format_time(position)} / {format_time(dur)}")
        if self.overlay.isVisible(): self.overlay.update()

    def _on_duration_changed(self, duration): self.overlay.timeline_slider.setRange(0, duration)

    def _on_status_changed(self, status):
        if status == QMediaPlayer.EndOfMedia: self.play_next()


class DockItem(QFrame):
    clicked = pyqtSignal(str)

    def __init__(self, key, icon, label, color):
        super().__init__()
        self.key = key
        self.setObjectName("DockItem")
        self.setFixedSize(66, 54)
        self.setCursor(QCursor(Qt.PointingHandCursor))
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 3, 0, 2)
        lay.setSpacing(1)
        self.ico = QLabel(icon)
        self.ico.setFixedSize(30, 30)
        self.ico.setAlignment(Qt.AlignCenter)
        self.ico.setStyleSheet(f"background:{color};border-radius:10px;font-size:16px;color:white;")
        self.ico.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.txt = QLabel(label)
        self.txt.setAlignment(Qt.AlignCenter)
        self.txt.setAttribute(Qt.WA_TransparentForMouseEvents)
        lay.addWidget(self.ico, 0, Qt.AlignHCenter)
        lay.addWidget(self.txt)
        self.set_selected(False)

    def set_selected(self, sel):
        self.setStyleSheet("QFrame#DockItem{background:%s;border-radius:16px;}"
                           % ("rgba(255,255,255,0.18)" if sel else "transparent"))
        self.txt.setStyleSheet("font-size:10px;font-weight:bold;background:transparent;color:%s;"
                               % ("white" if sel else "rgba(255,255,255,0.7)"))

    def mousePressEvent(self, e):
        self.clicked.emit(self.key)


class YarenTVApp(QMainWindow):
    GENRES = [("Recientes", "🕒", "Recientes", "#7C5CFA"), ("Favoritos", "♥", "Favoritos", "#FF5C8A"),
              ("Todos", "▶", "Todos", "#4C8DFF"), ("Accion", "⚡", "Acción", "#FFB02E"),
              ("Aventura", "🗺", "Aventura", "#2ED1A0"), ("Emocion", "💙", "Emoción", "#3FA9F5"),
              ("Animado", "🎨", "Animado", "#FF7A59"), ("Infantil", "🧸", "Infantil", "#B07CFF"),
              ("Musical", "🎤", "Musical", "#FF5CC8")]

    def __init__(self):
        super().__init__()
        self.setWindowTitle("YAREN TV")
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Window)
        self.setStyleSheet(STYLESHEET)
        self.filtered_videos = VIDEOS.copy()
        self.current_page = 0
        self.items_per_page = 3
        self.current_genre = "Recientes"
        root = QWidget(); self.setCentralWidget(root)
        root_lay = QVBoxLayout(root); root_lay.setContentsMargins(0, 0, 0, 0)
        self.app_container = QFrame()
        self.app_container.setObjectName("AppContainer")
        self.app_container.setFixedSize(800, 480)
        root_lay.addWidget(self.app_container, alignment=Qt.AlignCenter)
        cont_lay = QVBoxLayout(self.app_container)
        cont_lay.setContentsMargins(0, 0, 0, 0)
        self.stack = QStackedWidget()
        cont_lay.addWidget(self.stack)
        self.intro_screen = IntroScreen()
        self.intro_screen.confirmed.connect(self._on_intro_confirmed)
        self.stack.addWidget(self.intro_screen)
        self._build_catalog_view()
        self._build_player_view()
        self._select_category("Recientes")
        self.stack.setCurrentIndex(0)
        self.showFullScreen()

    def closeEvent(self, event):
        subprocess.Popen([
            "ros2", "topic", "pub", "--once", "/yaren_mode",
            "std_msgs/msg/String", "{data: 'idle'}"
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        os._exit(0)

    def _on_intro_confirmed(self):
        self.stack.setCurrentIndex(1)

    def _pager_btn(self, txt, d):
        b = QPushButton(txt)
        b.setFixedSize(40, 40)
        b.setCursor(QCursor(Qt.PointingHandCursor))
        b.setStyleSheet("QPushButton{background:rgba(255,255,255,0.16);color:white;border:none;border-radius:20px;"
                        "font-size:16px;font-weight:bold;} QPushButton:pressed{background:#6C8CFF;}"
                        "QPushButton:disabled{background:rgba(255,255,255,0.05);color:rgba(255,255,255,0.25);}")
        b.clicked.connect(lambda: self._ch_page(d))
        return b

    def _mark_dock(self, sel):
        for key, item in self.dock_items.items():
            item.set_selected(key == sel)

    def _build_catalog_view(self):
        w = QWidget(); lay = QVBoxLayout(w)
        lay.setContentsMargins(18, 12, 18, 12); lay.setSpacing(8)

        # --- barra superior ---
        top = QHBoxLayout(); top.setSpacing(0)
        left = QWidget(); left.setFixedWidth(220)
        ll = QHBoxLayout(left); ll.setContentsMargins(0, 0, 0, 0); ll.setSpacing(10)
        avatar = QLabel(); avatar.setFixedSize(46, 46)
        avatar.setPixmap(mascot_circle(46))
        brand = QLabel("<span style='color:#FFD93D;'>YAREN</span> <span style='color:white;'>TV</span>")
        brand.setStyleSheet("font-size:16px;font-weight:bold;background:transparent;")
        ll.addWidget(avatar); ll.addWidget(brand); ll.addStretch()

        self.lbl_section = QLabel("Lo más reciente")
        self.lbl_section.setAlignment(Qt.AlignCenter)
        self.lbl_section.setStyleSheet("color:white;font-size:22px;font-weight:bold;background:transparent;"
                                       "border-bottom:3px solid #6C8CFF;padding:0 16px 2px 16px;")

        right = QWidget(); right.setFixedWidth(220)
        rl = QHBoxLayout(right); rl.setContentsMargins(0, 0, 0, 0); rl.setSpacing(8)
        self.lbl_count = QLabel("")
        self.lbl_count.setAlignment(Qt.AlignCenter)
        self.lbl_count.setFixedSize(130, 34)
        self.lbl_count.setStyleSheet("background:#262B52;color:white;border-radius:17px;"
                                     "font-size:13px;font-weight:bold;")
        close = QPushButton("✕"); close.setFixedSize(40, 40)
        close.setCursor(QCursor(Qt.PointingHandCursor))
        close.setStyleSheet("QPushButton{background:rgba(255,255,255,0.16);color:white;border:none;border-radius:20px;"
                            "font-size:16px;font-weight:bold;} QPushButton:pressed{background:#FF5A5F;}")
        close.clicked.connect(self.close)
        rl.addStretch(); rl.addWidget(self.lbl_count); rl.addWidget(close)

        top.addWidget(left); top.addStretch(); top.addWidget(self.lbl_section); top.addStretch(); top.addWidget(right)
        lay.addLayout(top)

        # --- tarjetas ---
        mid = QHBoxLayout(); mid.setSpacing(8)
        self.btn_prev = self._pager_btn("◀", -1)
        self.btn_next = self._pager_btn("▶", 1)
        self.grid_widget = QWidget()
        self.grid_layout = QGridLayout(self.grid_widget)
        self.grid_layout.setContentsMargins(0, 0, 0, 0)
        self.grid_layout.setSpacing(12)
        self.grid_layout.setAlignment(Qt.AlignCenter)
        mid.addWidget(self.btn_prev); mid.addWidget(self.grid_widget, 1); mid.addWidget(self.btn_next)
        lay.addLayout(mid, 1)

        self.lbl_dots = QLabel("")
        self.lbl_dots.setAlignment(Qt.AlignCenter)
        lay.addWidget(self.lbl_dots)

        # --- barra inferior de categorias ---
        self.dock = QFrame(); self.dock.setObjectName("Dock")
        self.dock.setFixedSize(700, 66)
        self.dock.setStyleSheet("QFrame#Dock{background:#262B52;border-radius:33px;}")
        dl = QHBoxLayout(self.dock); dl.setContentsMargins(22, 5, 22, 5); dl.setSpacing(6)
        self.dock_items = {}
        for key, icon, label, color in self.GENRES:
            it = DockItem(key, icon, label, color)
            it.clicked.connect(self._select_category)
            self.dock_items[key] = it
            dl.addWidget(it)
        lay.addWidget(self.dock, 0, Qt.AlignHCenter)

        self.stack.addWidget(w)

    def _build_player_view(self):
        self.player_view = PlayerView()
        self.player_view.request_catalog.connect(self._back_to_catalog)
        self.stack.addWidget(self.player_view)

    def _apply_filter(self):
        g = self.current_genre
        gmap = {"Todos": None, "Accion": "accion", "Aventura": "aventura", "Emocion": "emocion",
                "Animado": "animado", "Infantil": "infantil", "Musical": "musical"}
        if g == "Recientes":
            vids = [VID_BY_ID[i] for i in STATE["recent"] if i in VID_BY_ID]
            self.filtered_videos = vids if vids else VIDEOS.copy()
            title = "Lo más reciente" if vids else "Todos los videos"
        elif g == "Favoritos":
            self.filtered_videos = [VID_BY_ID[i] for i in STATE["fav"] if i in VID_BY_ID]
            title = "Mis favoritos"
        else:
            k = gmap.get(g)
            self.filtered_videos = VIDEOS.copy() if k is None else [v for v in VIDEOS if v['genre'] == k]
            title = "Todos los videos" if k is None else next(l for kk, _, l, _ in self.GENRES if kk == g)
        self.lbl_section.setText(title)

    def _update_page(self):
        while self.grid_layout.count():
            wd = self.grid_layout.takeAt(0).widget()
            if wd: wd.deleteLater()
        n = self.items_per_page
        total = max(1, -(-len(self.filtered_videos) // n))
        self.current_page = max(0, min(self.current_page, total - 1))
        s = self.current_page * n
        if not self.filtered_videos:
            hint = QLabel("Toca ♡ en un video para guardarlo aquí")
            hint.setAlignment(Qt.AlignCenter)
            hint.setStyleSheet("color:rgba(255,255,255,0.8);font-size:16px;background:transparent;")
            self.grid_layout.addWidget(hint, 0, 0, 1, 3)
        for i, vid in enumerate(self.filtered_videos[s:s + n]):
            card = VideoCard(vid)
            card.clicked.connect(self._open_player)
            card.fav_toggled.connect(self._toggle_fav)
            self.grid_layout.addWidget(card, 0, i)
        self.btn_prev.setEnabled(self.current_page > 0)
        self.btn_next.setEnabled(s + n < len(self.filtered_videos))
        self.lbl_count.setText(f"🎬 {len(self.filtered_videos)} videos")
        self.lbl_dots.setText(" ".join(
            f"<span style='color:{'#6C8CFF' if p == self.current_page else 'rgba(255,255,255,0.4)'};"
            f"font-size:{20 if p == self.current_page else 12}px;'>●</span>" for p in range(total)))

    def _toggle_fav(self, vid):
        toggle_fav(vid)
        if self.current_genre == "Favoritos":
            self._apply_filter()
        self._update_page()

    def _ch_page(self, d):
        self.current_page += d
        self._update_page()

    def _select_category(self, genre):
        self.current_genre = genre
        self.current_page = 0
        self._apply_filter()
        self._mark_dock(genre)
        self._update_page()
        self.stack.setCurrentIndex(1)

    def _back_to_catalog(self):
        self.player_view.stop()
        self._select_category(self.current_genre)

    def _open_player(self, video_data):
        idx = next((i for i, v in enumerate(self.filtered_videos) if v['id'] == video_data['id']), 0)
        self.stack.setCurrentIndex(2)
        self.player_view.load_playlist(self.filtered_videos, idx)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = YarenTVApp()
    sys.exit(app.exec_())