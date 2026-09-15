#!/usr/bin/env python3
import sys
import os
import subprocess
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QPushButton, QStackedWidget,
                             QGridLayout, QFrame, QSlider, QSizePolicy, QGraphicsOpacityEffect)
from PyQt5.QtCore import Qt, pyqtSignal, QUrl, QTimer, QPropertyAnimation, QEasingCurve
from PyQt5.QtGui import QFont, QCursor, QPixmap
from PyQt5.QtMultimedia import QMediaPlayer, QMediaContent
from PyQt5.QtMultimediaWidgets import QVideoWidget

# Ruta base de tus íconos
ICONS = os.path.expanduser('~/robotis_ws/src/YAREN2/yaren_radio/iconos')

# =============================================================================
#  CATÁLOGO DE VIDEOS (Actualizado con imágenes)
# =============================================================================
VIDEOS = [
    {'id':'vid_pollito',        'title':'Pollito Pío',           'source':'Canciones de la Granja', 'genre':'infantil', 'image':f'{ICONS}/pollitopio.png', 'color':'#FFB347', 'file':'pollito_pio.mp4'},
    {'id':'vid_gallina',        'title':'Gallina Turuleca',      'source':'Canciones de Yaren',     'genre':'infantil', 'image':f'{ICONS}/gallita turuleca.png', 'color':'#FF8C42', 'file':'gallina_turuleca.mp4'},
    {'id':'vid_vaca',           'title':'La Vaca Lola',          'source':'Canciones Infantiles',   'genre':'infantil', 'image':f'{ICONS}/VacaLola.png', 'color':'#6BCB77', 'file':'vaca_lola.mp4'},
    {'id':'vid_susanita',       'title':'Susanita',              'source':'La Granja de Zenón',     'genre':'infantil', 'image':f'{ICONS}/Susanita.png', 'color':'#F368E0', 'file':'susanita.mp4'},
    {'id':'vid_rapunzel',       'title':'Rapunzel',              'source':'Disney',                 'genre':'infantil', 'image':f'{ICONS}/rapunzel.png', 'color':'#F7B731', 'file':'rapunzel.mp4'},
    {'id':'vid_coco',           'title':'COCO',                  'source':'Pixar',                  'genre':'infantil', 'image':f'{ICONS}/Coco.png', 'color':'#FF6B6B', 'file':'COCO.mp4'},
    {'id':'vid_jovenes_titanes','title':'Jóvenes Titanes',       'source':'Serie Animada DC',       'genre':'accion',   'image':f'{ICONS}/TeenTitan.png', 'color':'#6C5CE7', 'file':'jovenes_titanes.mp4'},
    {'id':'vid_spiderman',      'title':'Spider-Man',            'source':'Serie Clásica Marvel',   'genre':'accion',   'image':f'{ICONS}/Spectacular-Spiderman.png', 'color':'#FF4757', 'file':'spiderman_gran_poder.mp4'},
    {'id':'vid_superman_zod',   'title':'Superman vs Zod',       'source':'Man of Steel',           'genre':'accion',   'image':f'{ICONS}/superman.png', 'color':'#EE5A24', 'file':'superman_vs_zod.mp4'},
    {'id':'vid_shifu',          'title':'Shifu vs Tai Lung',     'source':'Kung Fu Panda',          'genre':'accion',   'image':f'{ICONS}/Kung-Fu-Panda.png', 'color':'#00B894', 'file':'shifu_vs_tai_lung.mp4'},
    {'id':'vid_ekko_jinx',      'title':'Ekko vs Jinx',          'source':'Arcane',                 'genre':'accion',   'image':f'{ICONS}/arcane.png', 'color':'#A29BFE', 'file':'ekko_vs_jinx.mp4'},
    {'id':'vid_ben10',          'title':'Ben 10',                'source':'Pelea Clásica',          'genre':'aventura', 'image':f'{ICONS}/ben10k.png', 'color':'#00D4A1', 'file':'ben10.mp4'},
    {'id':'vid_goku_trans',     'title':'Goku Transformaciones', 'source':'Dragon Ball Z',          'genre':'aventura', 'image':f'{ICONS}/gokuTransformaciones.png', 'color':'#FF9F43', 'file':'goku_transformaciones.mp4'},
    {'id':'vid_goku_ultra',     'title':'Goku Ultra Instinto',   'source':'Dragon Ball Super',      'genre':'aventura', 'image':f'{ICONS}/gokuUltra.png', 'color':'#48DBFB', 'file':'goku_ultra_instinto.mp4'},
    {'id':'vid_pockemon',       'title':'Pokémon',               'source':'Serie Animada',          'genre':'aventura', 'image':f'{ICONS}/Pokemon.png', 'color':'#F9CA24', 'file':'pockemon.mp4'},
    {'id':'vid_superman_padre', 'title':'Superman y Padre',      'source':'Man of Steel',           'genre':'emocion',  'image':f'{ICONS}/superman2025.png', 'color':'#3498DB', 'file':'superman_padre.mp4'},
    {'id':'vid_cars1',          'title':'Cars',                  'source':'Pixar',                  'genre':'animado',  'image':f'{ICONS}/Cars_1.png', 'color':'#E74C3C', 'file':'cars1.mp4'},
    {'id':'vid_yakko',          'title':'Yakko del Mundo',       'source':'Animaniacs',             'genre':'animado',  'image':f'{ICONS}/Yakko.png', 'color':'#9B59B6', 'file':'Yakko-Mundo.mp4'},
    {'id':'vid_golden_kpop',    'title':'Golden K-Pop',          'source':'K-Pop',                  'genre':'musical',  'image':f'{ICONS}/Golden-K-Pop.png', 'color':'#FFD700', 'file':'Golden-Kpop.mp4'},
    {'id':'vid_soda_pop',       'title':'Soda Pop',              'source':'Pop Latino',             'genre':'musical',  'image':f'{ICONS}/K-Pop-Demon-Hunters.png', 'color':'#FF6B9D', 'file':'Soda-Pop.mp4'},
    {'id':'vid_something_new',  'title':'Something New',         'source':'Pop',                    'genre':'musical',  'image':f'{ICONS}/High_School_Musical.png', 'color':'#A8E063', 'file':'Something-new.mp4'},
    {'id':'vid_your_idol',      'title':'Your Idol',             'source':'K-Pop',                  'genre':'musical',  'image':f'{ICONS}/YourIdol.png', 'color':'#FC5C7D', 'file':'Your-Idol.mp4'},
]

STYLESHEET = """
    QMainWindow, QWidget { background-color: #000000; }
    QFrame#AppContainer  { background-color: #080A12; border: 1px solid #1E2240; border-radius: 8px; }
    QLabel { color: #E8EBF5; }

    QPushButton.CategoryDropdownBtn, QPushButton.LargeCategoryBtn, QPushButton.PaginationBtn, QPushButton.BackBtn {
        background-color: #141728; border: 2px solid #1E2240; border-radius: 20px; color: #E8EBF5; font-weight: bold;
    }
    QPushButton.CategoryDropdownBtn { padding: 8px 20px; font-size: 15px; }
    QPushButton.LargeCategoryBtn { font-size: 20px; padding: 15px; }
    QPushButton.PaginationBtn { font-size: 18px; }
    QPushButton.BackBtn { border-radius: 16px; padding: 6px 16px; font-size: 14px; }
    
    QPushButton.CategoryDropdownBtn:hover, QPushButton.LargeCategoryBtn:hover, 
    QPushButton.PaginationBtn:hover, QPushButton.BackBtn:hover {
        border-color: #00E5FF; background-color: #1a1e36; color: #00E5FF;
    }
    QPushButton.PaginationBtn:disabled { background-color: transparent; border-color: transparent; color: transparent; }

    QPushButton#CloseBtn { background-color: transparent; color: #5A6080; font-size: 22px; font-weight: bold; border-radius: 18px; }
    QPushButton#CloseBtn:hover { color: #FF4757; background-color: rgba(255,71,87,0.1); }

    /* OVERLAY ESTILO NETFLIX */
    QFrame#OverlayBar {
        background-color: #0A0C16;
        border-top: 1px solid #1E2240;
    }
    QPushButton.OvBtn { background-color: transparent; border: none; color: #FFFFFF; font-size: 24px; font-weight: bold; }
    QPushButton.OvBtn:hover { color: #00E5FF; }
    QPushButton.OvBtnPrimary { background-color: transparent; border: none; color: #FFFFFF; font-size: 32px; font-weight: bold; }
    QPushButton.OvBtnPrimary:hover { color: #6C5CE7; }
    
    QPushButton.OvCatalogBtn {
        background-color: #1a1e36; border: 1px solid #1E2240; border-radius: 15px;
        color: #FFFFFF; font-size: 14px; font-weight: bold; padding: 5px 15px;
    }
    QPushButton.OvCatalogBtn:hover { background-color: #6C5CE7; border-color: #00E5FF; }

    /* SLIDERS */
    QSlider::groove:horizontal { border-radius: 2px; height: 4px; background: rgba(255,255,255,0.3); }
    QSlider::sub-page:horizontal { background: #E50914; border-radius: 2px; }
    QSlider::handle:horizontal { background: #FFFFFF; width: 14px; height: 14px; margin: -5px 0; border-radius: 7px; }
    QSlider::handle:horizontal:hover { transform: scale(1.2); background: #E50914; }
"""

def format_time(ms):
    s = (ms // 1000) % 60
    m = (ms // 60000) % 60
    h = (ms // 3600000)
    return f"{h:02d}:{m:02d}:{s:02d}" if h > 0 else f"{m:02d}:{s:02d}"

# ─────────────────────────────────────────────────────────────────────────────
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

class VideoCard(QFrame):
    clicked = pyqtSignal(dict)
    def __init__(self, vd):
        super().__init__()
        self.video_data = vd
        self.setProperty("class", "VideoCard")
        self.setCursor(QCursor(Qt.PointingHandCursor))
        self.setFixedSize(210, 150)
        
        # Color dinámico para la tarjeta
        card_color = vd.get('color', '#00E5FF')
        self.setStyleSheet(f"""
            QFrame.VideoCard {{
                background-color: #141728; 
                border: 2px solid #1E2240; 
                border-bottom: 4px solid {card_color};
                border-radius: 12px;
            }}
            QFrame.VideoCard:hover {{ 
                border-color: {card_color}; 
                background-color: #1a1e36; 
            }}
            QLabel.CardTitle {{ font-weight: bold; font-size: 13px; color: #FFFFFF; background: transparent; }}
            QLabel.CardSource {{ color: #8F95B2; font-size: 11px; background: transparent; }}
            QLabel.CardThumb {{ background-color: #0E1020; border-top-left-radius: 10px; border-top-right-radius: 10px; }}
        """)
        
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 8); lay.setSpacing(2)
        
        self.thumb = QLabel()
        image_path = vd.get('image', '')
        pixmap = QPixmap(image_path)
        
        if not pixmap.isNull():
            # CAMBIO AQUÍ: Usar Qt.KeepAspectRatio para que la imagen no se recorte
            pixmap = pixmap.scaled(210, 85, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.thumb.setPixmap(pixmap)
        else:
            self.thumb.setText("Sin Imagen")
            self.thumb.setStyleSheet(f"color: {card_color}; font-size: 14px; font-weight: bold; background: transparent;")
            
        self.thumb.setProperty("class", "CardThumb")
        self.thumb.setAlignment(Qt.AlignCenter)
        self.thumb.setMinimumHeight(85)
        self.thumb.setMaximumHeight(85)
        
        ti = QLabel(vd['title']); ti.setProperty("class", "CardTitle"); ti.setContentsMargins(10,4,10,0)
        so = QLabel(vd['source']); so.setProperty("class", "CardSource"); so.setContentsMargins(10,0,10,0)
        lay.addWidget(self.thumb); lay.addWidget(ti); lay.addWidget(so)

    def mousePressEvent(self, e): 
        self.clicked.emit(self.video_data)
# ─────────────────────────────────────────────────────────────────────────────
class OverlayBar(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("OverlayBar")
        self.setFixedHeight(110)

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
        main_lay.setContentsMargins(20, 20, 20, 15)
        
        time_lay = QHBoxLayout()
        self.timeline_slider = JumpSlider(Qt.Horizontal)
        self.timeline_slider.setCursor(QCursor(Qt.PointingHandCursor))
        self.lbl_time = QLabel("00:00 / 00:00")
        self.lbl_time.setStyleSheet("color: white; font-size: 12px; font-weight: bold; background: transparent;")
        time_lay.addWidget(self.timeline_slider); time_lay.addSpacing(10); time_lay.addWidget(self.lbl_time)
        
        ctrl_lay = QHBoxLayout()
        self.btn_prev = QPushButton("⏮"); self.btn_play = QPushButton("⏸"); self.btn_next = QPushButton("⏭")
        for btn in (self.btn_prev, self.btn_next):
            btn.setProperty("class", "OvBtn"); btn.setCursor(QCursor(Qt.PointingHandCursor))
        self.btn_play.setProperty("class", "OvBtnPrimary"); self.btn_play.setCursor(QCursor(Qt.PointingHandCursor))

        self.lbl_vol_icon = QLabel("🔊")
        self.lbl_vol_icon.setStyleSheet("color: white; font-size: 18px; background: transparent;")
        self.vol_slider = JumpSlider(Qt.Horizontal)
        self.vol_slider.setRange(0, 100); self.vol_slider.setValue(80)
        self.vol_slider.setFixedWidth(100); self.vol_slider.setCursor(QCursor(Qt.PointingHandCursor))

        self.lbl_title = QLabel("")
        self.lbl_title.setStyleSheet("color: white; font-size: 16px; font-weight: bold; background: transparent;")
        self.lbl_title.setAlignment(Qt.AlignCenter)

        self.btn_catalog = QPushButton("⊞ Catálogo")
        self.btn_catalog.setProperty("class", "OvCatalogBtn"); self.btn_catalog.setCursor(QCursor(Qt.PointingHandCursor))

        ctrl_lay.addWidget(self.btn_prev); ctrl_lay.addWidget(self.btn_play); ctrl_lay.addWidget(self.btn_next)
        ctrl_lay.addSpacing(20); ctrl_lay.addWidget(self.lbl_vol_icon); ctrl_lay.addWidget(self.vol_slider)
        ctrl_lay.addSpacing(10); ctrl_lay.addWidget(self.lbl_title, stretch=1); ctrl_lay.addSpacing(10)
        ctrl_lay.addWidget(self.btn_catalog)

        main_lay.addLayout(time_lay); main_lay.addLayout(ctrl_lay)

    def _on_anim_finished(self):
        if self._anim.endValue() == 0.0:
            self.hide() 

    def fade_in(self):
        if not self.isVisible():
            self.show()
        self._anim.stop(); self._anim.setEndValue(1.0); self._anim.start()

    def fade_out(self):
        self._anim.stop(); self._anim.setEndValue(0.0); self._anim.start()

    def show_temporarily(self, ms=3500):
        self.fade_in(); self._hide_timer.start(ms)

    def keep_visible(self):
        self.fade_in(); self._hide_timer.stop()

# ─────────────────────────────────────────────────────────────────────────────
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

        self.overlay = OverlayBar(self)
        self.overlay.btn_play.clicked.connect(self.toggle_play)
        self.overlay.btn_prev.clicked.connect(self.play_prev)
        self.overlay.btn_next.clicked.connect(self.play_next)
        self.overlay.btn_catalog.clicked.connect(self.request_catalog)
        
        self.overlay.timeline_slider.sliderMoved.connect(lambda v: [self.set_position(v), self._wake_ui()])
        self.overlay.vol_slider.sliderMoved.connect(lambda v: [self.set_volume(v), self._wake_ui()])

        self.center_osd = QLabel(self)
        self.center_osd.setStyleSheet("""
            QLabel {
                background-color: rgba(0, 0, 0, 150);
                color: white; 
                font-size: 50px; 
                border-radius: 40px;
            }
        """)
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
        bar_h = 110 
        self.overlay.setGeometry(0, self.height() - bar_h, self.width(), bar_h)
        self.center_osd.setGeometry((self.width() - 80) // 2, (self.height() - 80) // 2, 80, 80)
        super().resizeEvent(e)

    def _wake_ui(self):
        self.setCursor(QCursor(Qt.ArrowCursor))
        if self.media_player.state() != QMediaPlayer.PausedState:
            self.overlay.show_temporarily(3500)
            self._cursor_timer.start(3500)
        else:
            self.overlay.keep_visible()
            self._cursor_timer.stop()

    def eventFilter(self, source, event):
        if event.type() == event.MouseMove:
            if event.pos() != self._last_mouse_pos:
                self._last_mouse_pos = event.pos()
                self._wake_ui()
        elif event.type() == event.MouseButtonPress and source == self.video_widget:
            self.toggle_play()
            self._wake_ui()
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
        path = os.path.join(os.path.expanduser("~/robotis_ws/src/YAREN2/yaren_radio/videos"), vd['file'])
        
        self.media_player.stop()
        self.media_player.setMedia(QMediaContent(QUrl.fromLocalFile(path)))
        self.media_player.play()
        
        self.overlay.lbl_title.setText(f"{vd['title']} • {vd['source']}")
        self._wake_ui()
        self.setFocus() 

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
            self.media_player.pause(); self._show_osd("⏸")
        else:
            self.media_player.play(); self._show_osd("▶")

    def _show_osd(self, text):
        self.center_osd.setText(text)
        self.center_osd.show()
        self.center_osd.repaint()
        self.osd_anim.stop()
        self.osd_anim.setStartValue(1.0)
        self.osd_anim.setEndValue(0.0)
        self.osd_anim.start()

    def stop(self): self.media_player.stop()

    def set_volume(self, v: int):
        self.media_player.setVolume(v)
        subprocess.Popen(["pactl", "set-sink-volume", "@DEFAULT_SINK@", f"{v}%"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def set_position(self, position): self.media_player.setPosition(position)

    def _on_state_changed(self, state):
        self.overlay.btn_play.setText("⏸" if state == QMediaPlayer.PlayingState else "▶")
        self._wake_ui()
        self.overlay.repaint()

    def _on_position_changed(self, position):
        is_dragging = QApplication.mouseButtons() & Qt.LeftButton and self.overlay.timeline_slider.underMouse()
        if not is_dragging: self.overlay.timeline_slider.setValue(position)
        dur = max(self.media_player.duration(), 1)
        self.overlay.lbl_time.setText(f"{format_time(position)} / {format_time(dur)}")
        
        if self.overlay.isVisible():
            self.overlay.update()

    def _on_duration_changed(self, duration): self.overlay.timeline_slider.setRange(0, duration)
    def _on_status_changed(self, status):
        if status == QMediaPlayer.EndOfMedia: self.play_next()

# ─────────────────────────────────────────────────────────────────────────────
class YarenTVApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("YAREN TV")
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Window)
        self.setStyleSheet(STYLESHEET)

        self.filtered_videos = VIDEOS.copy()
        self.current_page = 0
        self.items_per_page = 6

        root = QWidget(); self.setCentralWidget(root)
        root_lay = QVBoxLayout(root); root_lay.setContentsMargins(0,0,0,0)

        self.app_container = QFrame(); self.app_container.setObjectName("AppContainer")
        self.app_container.setFixedSize(800, 480)
        root_lay.addWidget(self.app_container, alignment=Qt.AlignCenter)

        cont_lay = QVBoxLayout(self.app_container); cont_lay.setContentsMargins(0,0,0,0)
        self.stack = QStackedWidget()
        cont_lay.addWidget(self.stack)

        self._build_catalog_view()
        self._build_player_view() 
        self._build_categories_view()

        self.stack.setCurrentIndex(0)
        self._update_page()
        self.showFullScreen()

    def _build_catalog_view(self):
        w = QWidget(); lay = QVBoxLayout(w); lay.setContentsMargins(15, 20, 15, 20)
        
        top = QHBoxLayout(); top.setContentsMargins(10,0,10,0)
        logo = QLabel("YAREN TV"); logo.setFont(QFont("Arial",18,QFont.Bold)); logo.setStyleSheet("color:#fff;")
        dot  = QLabel(" •");       dot.setFont(QFont("Arial",18,QFont.Bold));  dot.setStyleSheet("color:#E50914;")
        top.addWidget(logo); top.addWidget(dot); top.addSpacing(25)

        self.btn_cat_label = QPushButton("Categorías: Todos ▼")
        self.btn_cat_label.setProperty("class","CategoryDropdownBtn"); self.btn_cat_label.setCursor(QCursor(Qt.PointingHandCursor))
        self.btn_cat_label.clicked.connect(lambda: self.stack.setCurrentIndex(2))
        top.addWidget(self.btn_cat_label); top.addStretch()

        close = QPushButton("✕"); close.setObjectName("CloseBtn")
        close.setFixedSize(36,36); close.setCursor(QCursor(Qt.PointingHandCursor)); close.clicked.connect(self.close)
        top.addWidget(close)
        lay.addLayout(top); lay.addSpacing(15)

        mid = QHBoxLayout()
        self.btn_prev = QPushButton("◀"); self.btn_prev.setProperty("class","PaginationBtn"); self.btn_prev.setFixedSize(36,36)
        self.btn_prev.setCursor(QCursor(Qt.PointingHandCursor)); self.btn_prev.clicked.connect(lambda: self._ch_page(-1))

        self.grid_widget = QWidget(); self.grid_layout = QGridLayout(self.grid_widget)
        self.grid_layout.setAlignment(Qt.AlignCenter); self.grid_layout.setSpacing(12)

        self.btn_next = QPushButton("▶"); self.btn_next.setProperty("class","PaginationBtn"); self.btn_next.setFixedSize(36,36)
        self.btn_next.setCursor(QCursor(Qt.PointingHandCursor)); self.btn_next.clicked.connect(lambda: self._ch_page(1))

        mid.addStretch(); mid.addWidget(self.btn_prev); mid.addSpacing(10); mid.addWidget(self.grid_widget); mid.addSpacing(10)
        mid.addWidget(self.btn_next); mid.addStretch()
        lay.addLayout(mid); lay.addStretch(); self.stack.addWidget(w)

    def _build_player_view(self):
        self.player_view = PlayerView()
        self.player_view.request_catalog.connect(lambda: [self.player_view.stop(), self.stack.setCurrentIndex(0)])
        self.stack.addWidget(self.player_view)

    def _build_categories_view(self):
        w = QWidget(); lay = QVBoxLayout(w); lay.setContentsMargins(40,30,40,30)
        lbl = QLabel("Selecciona una Categoría"); lbl.setFont(QFont("Arial",22,QFont.Bold)); lbl.setAlignment(Qt.AlignCenter)
        lay.addWidget(lbl); lay.addSpacing(20)

        grid = QGridLayout(); grid.setSpacing(16)
        genres = [("Todos","🌟"),("Acción","⚡"),("Aventura","🗺"),("Emoción","💙"),("Animado","🎨"),("Infantil","🧸"),("Musical","🎤")]
        for i, (name, icon) in enumerate(genres):
            btn = QPushButton(f"{icon}  {name}"); btn.setProperty("class","LargeCategoryBtn")
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding); btn.setCursor(QCursor(Qt.PointingHandCursor))
            btn.clicked.connect(lambda _, g=name: self._select_category(g))
            grid.addWidget(btn, i//3, i%3)
            
        lay.addLayout(grid); lay.addSpacing(20)
        cancel = QPushButton("✕ Cancelar"); cancel.setProperty("class","BackBtn"); cancel.setFixedSize(160,45)
        cancel.setCursor(QCursor(Qt.PointingHandCursor)); cancel.clicked.connect(lambda: self.stack.setCurrentIndex(0))
        bot = QHBoxLayout(); bot.addStretch(); bot.addWidget(cancel); bot.addStretch(); lay.addLayout(bot); self.stack.addWidget(w)

    def _update_page(self):
        while self.grid_layout.count(): self.grid_layout.takeAt(0).widget().deleteLater()
        s = self.current_page * self.items_per_page
        for i, vid in enumerate(self.filtered_videos[s:s+self.items_per_page]):
            card = VideoCard(vid); card.clicked.connect(self._open_player)
            self.grid_layout.addWidget(card, i//3, i%3)
        self.btn_prev.setEnabled(self.current_page > 0)
        self.btn_next.setEnabled(s + self.items_per_page < len(self.filtered_videos))

    def _ch_page(self, dir):
        self.current_page += dir; self._update_page()

    def _select_category(self, genre):
        self.btn_cat_label.setText(f"Categorías: {genre} ▼")
        gmap = {"Todos":None, "Acción":"accion", "Aventura":"aventura", "Emoción":"emocion", "Animado":"animado", "Infantil":"infantil", "Musical":"musical"}
        g = gmap.get(genre)
        self.filtered_videos = VIDEOS.copy() if g is None else [v for v in VIDEOS if v['genre']==g]
        self.current_page = 0; self._update_page(); self.stack.setCurrentIndex(0)

    def _open_player(self, video_data):
        idx = next((i for i,v in enumerate(self.filtered_videos) if v['id']==video_data['id']), 0)
        self.stack.setCurrentIndex(1)
        self.player_view.load_playlist(self.filtered_videos, idx)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = YarenTVApp()
    sys.exit(app.exec_())