#!/usr/bin/env python3
import sys
import os
import subprocess
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QPushButton, QStackedWidget, 
                             QGridLayout, QFrame, QSlider, QSizePolicy)
from PyQt5.QtCore import Qt, pyqtSignal, QUrl
from PyQt5.QtGui import QFont, QCursor
from PyQt5.QtMultimedia import QMediaPlayer, QMediaContent
from PyQt5.QtMultimediaWidgets import QVideoWidget

# =============================================================================
#  CATÁLOGO COMPLETO — todos los archivos de ~/robotis_ws/.../videos/
#  Géneros: "infantil" | "accion" | "aventura" | "emocion" | "animado" | "musical"
# =============================================================================
VIDEOS = [
    # ── INFANTIL ──────────────────────────────────────────────────────────────
    {'id':'vid_pollito',
     'title':'Pollito Pío',          'source':'Canciones de la Granja',
     'genre':'infantil',  'emoji':'🐥', 'color':'#FFB347', 'file':'pollito_pio.mp4'},

    {'id':'vid_gallina',
     'title':'Gallina Turuleca',     'source':'Canciones de Yaren',
     'genre':'infantil',  'emoji':'🐔', 'color':'#FF8C42', 'file':'gallina_turuleca.mp4'},

    {'id':'vid_vaca',
     'title':'La Vaca Lola',         'source':'Canciones Infantiles',
     'genre':'infantil',  'emoji':'🐄', 'color':'#6BCB77', 'file':'vaca_lola.mp4'},

    {'id':'vid_susanita',
     'title':'Susanita',             'source':'La Granja de Zenón',
     'genre':'infantil',  'emoji':'👧', 'color':'#F368E0', 'file':'susanita.mp4'},

    {'id':'vid_rapunzel',
     'title':'Rapunzel',             'source':'Disney',
     'genre':'infantil',  'emoji':'👸', 'color':'#F7B731', 'file':'rapunzel.mp4'},

    {'id':'vid_coco',
     'title':'COCO',                 'source':'Pixar',
     'genre':'infantil',  'emoji':'💀', 'color':'#FF6B6B', 'file':'COCO.mp4'},

    # ── ACCIÓN ────────────────────────────────────────────────────────────────
    {'id':'vid_jovenes_titanes',
     'title':'Jóvenes Titanes',      'source':'Serie Animada DC',
     'genre':'accion',    'emoji':'⚡', 'color':'#6C5CE7', 'file':'jovenes_titanes.mp4'},

    {'id':'vid_spiderman',
     'title':'Spider-Man',           'source':'Serie Clásica Marvel',
     'genre':'accion',    'emoji':'🕷️', 'color':'#FF4757', 'file':'spiderman_gran_poder.mp4'},

    {'id':'vid_superman_zod',
     'title':'Superman vs Zod',      'source':'Man of Steel',
     'genre':'accion',    'emoji':'💥', 'color':'#EE5A24', 'file':'superman_vs_zod.mp4'},

    {'id':'vid_shifu',
     'title':'Shifu vs Tai Lung',    'source':'Kung Fu Panda',
     'genre':'accion',    'emoji':'🐼', 'color':'#00B894', 'file':'shifu_vs_tai_lung.mp4'},

    {'id':'vid_ekko_jinx',
     'title':'Ekko vs Jinx',         'source':'Arcane',
     'genre':'accion',    'emoji':'🎭', 'color':'#A29BFE', 'file':'ekko_vs_jinx.mp4'},

    # ── AVENTURA ──────────────────────────────────────────────────────────────
    {'id':'vid_ben10',
     'title':'Ben 10',               'source':'Pelea Clásica',
     'genre':'aventura',  'emoji':'👾', 'color':'#00D4A1', 'file':'ben10.mp4'},

    {'id':'vid_goku_trans',
     'title':'Goku Transformaciones','source':'Dragon Ball Z',
     'genre':'aventura',  'emoji':'🔥', 'color':'#FF9F43', 'file':'goku_transformaciones.mp4'},

    {'id':'vid_goku_ultra',
     'title':'Goku Ultra Instinto',  'source':'Dragon Ball Super',
     'genre':'aventura',  'emoji':'✨', 'color':'#48DBFB', 'file':'goku_ultra_instinto.mp4'},

    {'id':'vid_jake',
     'title':'Jake el Perro',        'source':'Hora de Aventura',
     'genre':'aventura',  'emoji':'🐶', 'color':'#FECA57', 'file':'jake.mp4'},

    {'id':'vid_pockemon',
     'title':'Pokémon',              'source':'Serie Animada',
     'genre':'aventura',  'emoji':'⚡', 'color':'#F9CA24', 'file':'pockemon.mp4'},

    # ── EMOCIÓN ───────────────────────────────────────────────────────────────
    {'id':'vid_superman_padre',
     'title':'Superman y Padre',     'source':'Man of Steel',
     'genre':'emocion',   'emoji':'💙', 'color':'#3498DB', 'file':'superman_padre.mp4'},

    # ── ANIMADO ───────────────────────────────────────────────────────────────
    {'id':'vid_cars1',
     'title':'Cars',                 'source':'Pixar',
     'genre':'animado',   'emoji':'🏎️', 'color':'#E74C3C', 'file':'cars1.mp4'}, 
     
    {'id':'vid_yakko',
     'title':'Yakko del Mundo',      'source':'Animaniacs',
     'genre':'animado',   'emoji':'🌍', 'color':'#9B59B6', 'file':'Yakko-Mundo.mp4'},

    # ── MUSICAL ───────────────────────────────────────────────────────────────
    {'id':'vid_golden_kpop',
     'title':'Golden K-Pop',         'source':'K-Pop',
     'genre':'musical',   'emoji':'🎤', 'color':'#FFD700', 'file':'Golden-Kpop.mp4'},

    {'id':'vid_soda_pop',
     'title':'Soda Pop',             'source':'Pop Latino',
     'genre':'musical',   'emoji':'🎵', 'color':'#FF6B9D', 'file':'Soda-Pop.mp4'},

    {'id':'vid_something_new',
     'title':'Something New',        'source':'Pop',
     'genre':'musical',   'emoji':'🎶', 'color':'#A8E063', 'file':'Something-new.mp4'},

    {'id':'vid_your_idol',
     'title':'Your Idol',            'source':'K-Pop',
     'genre':'musical',   'emoji':'⭐', 'color':'#FC5C7D', 'file':'Your-Idol.mp4'},
]

# =============================================================================
STYLESHEET = """
    QMainWindow { background-color: #000000; }
    QFrame#AppContainer { 
        background-color: #080A12; 
        border: 1px solid #1E2240;
        border-radius: 8px;
    }
    QLabel { color: #E8EBF5; }
    
    QPushButton.CategoryDropdownBtn {
        background-color: #141728; border: 2px solid #1E2240; border-radius: 20px; 
        padding: 8px 20px; color: #E8EBF5; font-weight: bold; font-size: 15px;
    }
    QPushButton.CategoryDropdownBtn:hover { border-color: #00E5FF; background-color: #1a1e36; }
    
    QPushButton.LargeCategoryBtn {
        background-color: #141728; border: 2px solid #1E2240; border-radius: 20px;
        color: #E8EBF5; font-size: 22px; font-weight: bold; padding: 15px;
    }
    QPushButton.LargeCategoryBtn:hover { border-color: #6C5CE7; background-color: #1a1e36; color: #00E5FF; }
    
    QPushButton.PaginationBtn {
        background-color: #141728; border: 2px solid #1E2240; border-radius: 18px;
        color: #E8EBF5; font-size: 18px; font-weight: bold;
    }
    QPushButton.PaginationBtn:hover { border-color: #00E5FF; color: #00E5FF; background-color: #1a1e36; }
    QPushButton.PaginationBtn:disabled { background-color: transparent; border-color: transparent; color: transparent; }
    
    QPushButton#CloseBtn {
        background-color: transparent; color: #5A6080; font-size: 22px; font-weight: bold; border-radius: 18px;
    }
    QPushButton#CloseBtn:hover { color: #FF4757; background-color: rgba(255, 71, 87, 0.1); }
    
    QFrame.VideoCard {
        background-color: #141728; border: 1.5px solid #1E2240; border-radius: 12px;
    }
    QFrame.VideoCard:hover { border-color: #00E5FF; background-color: #1a1e36; }
    QLabel.CardTitle { font-weight: bold; font-size: 13px; color: #FFFFFF; }
    QLabel.CardSource { color: #8F95B2; font-size: 11px; }
    QLabel.CardThumb { 
        background-color: #0E1020; font-size: 45px; border-top-left-radius: 12px; border-top-right-radius: 12px; 
    }
    
    QFrame.PlayerHeader { background-color: transparent; }
    QPushButton.BackBtn {
        background-color: #141728; border: 2px solid #1E2240; border-radius: 16px; padding: 6px 16px; color: #E8EBF5; font-weight: bold; font-size: 14px;
    }
    QPushButton.BackBtn:hover { border-color: #00E5FF; color: #00E5FF; }
    
    QFrame.VideoArea { background-color: #000000; border-radius: 12px; border: 1px solid #1E2240; }
    QFrame.ControlsStrip { background-color: transparent; }
    
    QPushButton.CtrlBtn {
        background-color: #141728; border: 2px solid #1E2240; border-radius: 17px;
        color: #E8EBF5; font-size: 14px; font-weight: bold; padding: 0px;
    }
    QPushButton.CtrlBtn:hover { border-color: #00E5FF; color: #00E5FF; background-color: #1a1e36; }
    QPushButton.CtrlBtn:pressed { background-color: #0E1020; }
    
    QPushButton.CtrlBtnPrimary {
        background-color: #6C5CE7; border: 2px solid #6C5CE7; border-radius: 22px;
        color: #FFFFFF; font-size: 16px; font-weight: bold; padding: 0px;
    }
    QPushButton.CtrlBtnPrimary:hover { background-color: #7d6ef0; border-color: #00E5FF; }
    QPushButton.CtrlBtnPrimary:pressed { background-color: #5a4bd1; border-color: #5a4bd1; }
    
    QSlider::groove:horizontal { border-radius: 3px; height: 6px; background: #1E2240; }
    QSlider::add-page:horizontal { background: #141728; border-radius: 3px; }
    QSlider::handle:horizontal { background: #FFFFFF; width: 14px; height: 14px; margin: -4px 0; border-radius: 7px; }
    QSlider::sub-page:horizontal { background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #6C5CE7, stop:1 #00E5FF); border-radius: 3px; }
"""


class JumpSlider(QSlider):
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            val = self.minimum() + int(
                (self.maximum() - self.minimum()) * event.x() / self.width()
            )
            self.setValue(val)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.LeftButton:
            val = self.minimum() + int(
                (self.maximum() - self.minimum()) * event.x() / self.width()
            )
            self.setValue(val)
            return
        super().mouseMoveEvent(event)


class VideoCard(QFrame):
    clicked = pyqtSignal(dict)

    def __init__(self, video_data):
        super().__init__()
        self.video_data = video_data
        self.setProperty("class", "VideoCard")
        self.setCursor(QCursor(Qt.PointingHandCursor))
        self.setFixedSize(210, 150)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 8)
        layout.setSpacing(2)

        self.thumb = QLabel(video_data['emoji'])
        self.thumb.setProperty("class", "CardThumb")
        self.thumb.setAlignment(Qt.AlignCenter)
        self.thumb.setMinimumHeight(85)

        self.title = QLabel(video_data['title'])
        self.title.setProperty("class", "CardTitle")
        self.title.setContentsMargins(10, 4, 10, 0)

        self.source = QLabel(video_data['source'])
        self.source.setProperty("class", "CardSource")
        self.source.setContentsMargins(10, 0, 10, 0)

        layout.addWidget(self.thumb)
        layout.addWidget(self.title)
        layout.addWidget(self.source)

    def mousePressEvent(self, event):
        self.clicked.emit(self.video_data)


class YarenTVApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("YAREN TV")
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Window)
        self.setStyleSheet(STYLESHEET)

        self.wrapper_widget = QWidget()
        self.setCentralWidget(self.wrapper_widget)
        wrapper_layout = QVBoxLayout(self.wrapper_widget)
        wrapper_layout.setAlignment(Qt.AlignCenter)

        self.app_container = QFrame()
        self.app_container.setObjectName("AppContainer")
        self.app_container.setFixedSize(800, 480)
        wrapper_layout.addWidget(self.app_container)

        main_layout = QVBoxLayout(self.app_container)
        main_layout.setContentsMargins(0, 0, 0, 0)

        self.stack = QStackedWidget()
        main_layout.addWidget(self.stack)

        self.filtered_videos = VIDEOS.copy()
        self.current_page    = 0
        self.items_per_page  = 6
        self.current_video_data = None
        self.current_video_idx  = 0

        self.build_catalog_view()
        self.build_player_view()
        self.build_categories_view()

        self.stack.setCurrentIndex(0)
        self.update_page()
        self.showFullScreen()

    # ── CATÁLOGO ─────────────────────────────────────────────────────────────
    def build_catalog_view(self):
        catalog_widget = QWidget()
        layout = QVBoxLayout(catalog_widget)
        layout.setContentsMargins(15, 20, 15, 20)

        top_bar = QHBoxLayout()
        top_bar.setContentsMargins(10, 0, 10, 0)

        logo = QLabel("YAREN TV")
        logo.setFont(QFont("Arial", 18, QFont.Bold))
        logo.setStyleSheet("color: #FFFFFF;")

        logo_accent = QLabel(" •")
        logo_accent.setFont(QFont("Arial", 18, QFont.Bold))
        logo_accent.setStyleSheet("color: #00E5FF;")

        top_bar.addWidget(logo)
        top_bar.addWidget(logo_accent)
        top_bar.addSpacing(25)

        self.btn_categorias = QPushButton("Categorías: Todos ▼")
        self.btn_categorias.setProperty("class", "CategoryDropdownBtn")
        self.btn_categorias.setCursor(QCursor(Qt.PointingHandCursor))
        self.btn_categorias.clicked.connect(lambda: self.stack.setCurrentIndex(2))
        top_bar.addWidget(self.btn_categorias)

        top_bar.addStretch()

        close_btn = QPushButton("✕")
        close_btn.setObjectName("CloseBtn")
        close_btn.setFixedSize(36, 36)
        close_btn.setCursor(QCursor(Qt.PointingHandCursor))
        close_btn.clicked.connect(self.close)
        top_bar.addWidget(close_btn)

        layout.addLayout(top_bar)
        layout.addSpacing(15)

        content_layout = QHBoxLayout()

        self.btn_prev_page = QPushButton("◀")
        self.btn_prev_page.setProperty("class", "PaginationBtn")
        self.btn_prev_page.setFixedSize(36, 36)
        self.btn_prev_page.setCursor(QCursor(Qt.PointingHandCursor))
        self.btn_prev_page.clicked.connect(self.prev_page)

        self.grid_widget = QWidget()
        self.grid_widget.setStyleSheet("background: transparent;")
        self.grid_layout = QGridLayout(self.grid_widget)
        self.grid_layout.setAlignment(Qt.AlignCenter)
        self.grid_layout.setSpacing(12)
        self.grid_layout.setContentsMargins(0, 0, 0, 0)

        self.btn_next_page = QPushButton("▶")
        self.btn_next_page.setProperty("class", "PaginationBtn")
        self.btn_next_page.setFixedSize(36, 36)
        self.btn_next_page.setCursor(QCursor(Qt.PointingHandCursor))
        self.btn_next_page.clicked.connect(self.next_page)

        content_layout.addStretch()
        content_layout.addWidget(self.btn_prev_page)
        content_layout.addSpacing(10)
        content_layout.addWidget(self.grid_widget)
        content_layout.addSpacing(10)
        content_layout.addWidget(self.btn_next_page)
        content_layout.addStretch()

        layout.addLayout(content_layout)
        layout.addStretch()

        self.stack.addWidget(catalog_widget)

    # ── PLAYER ───────────────────────────────────────────────────────────────
    def build_player_view(self):
        player_widget = QWidget()
        layout = QVBoxLayout(player_widget)
        layout.setContentsMargins(15, 10, 15, 8)
        layout.setSpacing(6)

        header = QFrame()
        header.setProperty("class", "PlayerHeader")
        h_layout = QHBoxLayout(header)
        h_layout.setContentsMargins(0, 0, 0, 0)

        back_btn = QPushButton("← Volver")
        back_btn.setProperty("class", "BackBtn")
        back_btn.setCursor(QCursor(Qt.PointingHandCursor))
        back_btn.clicked.connect(self.show_catalog)

        self.player_title = QLabel("Título del Video")
        self.player_title.setFont(QFont("Arial", 16, QFont.Bold))
        self.player_title.setStyleSheet("color: white;")

        h_layout.addWidget(back_btn)
        h_layout.addSpacing(15)
        h_layout.addWidget(self.player_title)
        h_layout.addStretch()
        layout.addWidget(header)

        self.video_area = QFrame()
        self.video_area.setProperty("class", "VideoArea")
        v_layout = QVBoxLayout(self.video_area)
        v_layout.setContentsMargins(0, 0, 0, 0)

        self.media_player = QMediaPlayer(None, QMediaPlayer.VideoSurface)
        self.video_widget = QVideoWidget()
        self.media_player.setVideoOutput(self.video_widget)
        self.media_player.error.connect(
            lambda err: print(f"[ERROR] {err}: {self.media_player.errorString()}")
        )

        v_layout.addWidget(self.video_widget)
        layout.addWidget(self.video_area, stretch=1)

        controls = QFrame()
        controls.setProperty("class", "ControlsStrip")
        c_layout = QHBoxLayout(controls)
        c_layout.setContentsMargins(0, 0, 0, 0)
        c_layout.setSpacing(10)

        prev_btn = QPushButton("◀◀")
        prev_btn.setProperty("class", "CtrlBtn")
        prev_btn.setFixedSize(34, 34)
        prev_btn.setCursor(QCursor(Qt.PointingHandCursor))
        prev_btn.clicked.connect(self.play_prev_video)

        self.play_btn = QPushButton("▶")
        self.play_btn.setProperty("class", "CtrlBtnPrimary")
        self.play_btn.setFixedSize(44, 44)
        self.play_btn.setCursor(QCursor(Qt.PointingHandCursor))
        self.play_btn.clicked.connect(self.toggle_play)

        next_btn = QPushButton("▶▶")
        next_btn.setProperty("class", "CtrlBtn")
        next_btn.setFixedSize(34, 34)
        next_btn.setCursor(QCursor(Qt.PointingHandCursor))
        next_btn.clicked.connect(self.play_next_video)

        vol_icon = QLabel("🔊")
        vol_icon.setStyleSheet("font-size: 16px; background: transparent;")

        self.vol_slider = JumpSlider(Qt.Horizontal)
        self.vol_slider.setRange(0, 100)
        self.vol_slider.setValue(80)
        self.vol_slider.setFixedWidth(130)
        self.vol_slider.setCursor(QCursor(Qt.PointingHandCursor))
        self.vol_slider.valueChanged.connect(self.change_volume)
        self.change_volume(self.vol_slider.value())

        c_layout.addStretch()
        c_layout.addWidget(prev_btn)
        c_layout.addWidget(self.play_btn)
        c_layout.addWidget(next_btn)
        c_layout.addSpacing(40)
        c_layout.addWidget(vol_icon)
        c_layout.addWidget(self.vol_slider)
        c_layout.addStretch()

        layout.addWidget(controls)
        self.stack.addWidget(player_widget)

        self.media_player.stateChanged.connect(self.media_state_changed)
        self.media_player.mediaStatusChanged.connect(self.media_status_changed)

    # ── CATEGORÍAS ───────────────────────────────────────────────────────────
    def build_categories_view(self):
        cat_widget = QWidget()
        layout = QVBoxLayout(cat_widget)
        layout.setContentsMargins(40, 30, 40, 30)

        title = QLabel("Selecciona una Categoría")
        title.setFont(QFont("Arial", 22, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        layout.addSpacing(20)

        grid = QGridLayout()
        grid.setSpacing(20)

        # 6 géneros ahora incluyendo "Musical"
        genres = [
            ("Todos",    "🌟"),
            ("Acción",   "⚡"),
            ("Aventura", "🗺️"),
            ("Emoción",  "💙"),
            ("Animado",  "🎨"),
            ("Infantil", "🧸"),
            ("Musical",  "🎤"),
        ]

        row, col = 0, 0
        for genre, icon in genres:
            btn = QPushButton(f"{icon}  {genre}")
            btn.setProperty("class", "LargeCategoryBtn")
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            btn.setCursor(QCursor(Qt.PointingHandCursor))
            btn.clicked.connect(lambda checked, g=genre: self.select_category(g))
            grid.addWidget(btn, row, col)
            col += 1
            if col > 2:
                col = 0
                row += 1

        layout.addLayout(grid)
        layout.addSpacing(25)

        back_layout = QHBoxLayout()
        back_layout.addStretch()
        btn_cancel = QPushButton("✕ Cancelar")
        btn_cancel.setProperty("class", "BackBtn")
        btn_cancel.setFixedSize(160, 45)
        btn_cancel.setCursor(QCursor(Qt.PointingHandCursor))
        btn_cancel.clicked.connect(lambda: self.stack.setCurrentIndex(0))
        back_layout.addWidget(btn_cancel)
        back_layout.addStretch()

        layout.addLayout(back_layout)
        self.stack.addWidget(cat_widget)

    # ── LÓGICA ───────────────────────────────────────────────────────────────
    def update_page(self):
        while self.grid_layout.count():
            item = self.grid_layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)
                widget.deleteLater()

        start_idx  = self.current_page * self.items_per_page
        end_idx    = start_idx + self.items_per_page
        page_videos = self.filtered_videos[start_idx:end_idx]

        for i, vid in enumerate(page_videos):
            card = VideoCard(vid)
            card.clicked.connect(self.open_player)
            self.grid_layout.addWidget(card, i // 3, i % 3)

        self.btn_prev_page.setEnabled(self.current_page > 0)
        self.btn_next_page.setEnabled(end_idx < len(self.filtered_videos))
        self.grid_widget.layout().activate()
        self.grid_widget.updateGeometry()
        self.grid_widget.update()

    def next_page(self):
        self.current_page += 1
        self.update_page()

    def prev_page(self):
        if self.current_page > 0:
            self.current_page -= 1
            self.update_page()

    def select_category(self, genre):
        self.btn_categorias.setText(f"Categorías: {genre} ▼")
        genre_map = {
            "Todos":    None,
            "Acción":   "accion",
            "Aventura": "aventura",
            "Emoción":  "emocion",
            "Animado":  "animado",
            "Infantil": "infantil",
            "Musical":  "musical",
        }
        g = genre_map.get(genre)
        self.filtered_videos = VIDEOS.copy() if g is None else [v for v in VIDEOS if v['genre'] == g]
        self.current_page = 0
        self.update_page()
        self.stack.setCurrentIndex(0)

    def open_player(self, video_data):
        self.media_player.stop()
        self.media_player.setMedia(QMediaContent())

        self.current_video_data = video_data
        self.current_video_idx  = next(
            (i for i, v in enumerate(self.filtered_videos) if v['id'] == video_data['id']), 0
        )

        self.player_title.setText(f"{video_data['title']} — {video_data['source']}")
        self.stack.setCurrentIndex(1)

        base_path = os.path.expanduser(
            "~/robotis_ws/src/YAREN2/yaren_radio/videos"
        )
        full_path = os.path.join(base_path, video_data['file'])
        print(f"[YAREN TV] ▶ {full_path}")

        self.media_player.setMedia(QMediaContent(QUrl.fromLocalFile(full_path)))
        self.media_player.play()
        self.play_btn.setText("❚❚")

    def show_catalog(self):
        self.media_player.stop()
        self.stack.setCurrentIndex(0)

    def toggle_play(self):
        if self.media_player.state() == QMediaPlayer.PlayingState:
            self.media_player.pause()
        else:
            self.media_player.play()

    def play_next_video(self):
        if not self.filtered_videos:
            return
        self.current_video_idx = (self.current_video_idx + 1) % len(self.filtered_videos)
        self.open_player(self.filtered_videos[self.current_video_idx])

    def play_prev_video(self):
        if not self.filtered_videos:
            return
        self.current_video_idx = (self.current_video_idx - 1) % len(self.filtered_videos)
        self.open_player(self.filtered_videos[self.current_video_idx])

    def change_volume(self, value):
        self.media_player.setVolume(value)
        subprocess.Popen(
            ["pactl", "set-sink-volume", "@DEFAULT_SINK@", f"{value}%"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

    def media_state_changed(self, state):
        self.play_btn.setText("❚❚" if state == QMediaPlayer.PlayingState else "▶")

    def media_status_changed(self, status):
        if status == QMediaPlayer.EndOfMedia:
            self.play_next_video()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.media_player.stop()
            self.close()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = YarenTVApp()
    sys.exit(app.exec_())