#!/usr/bin/env python3
"""voice_commands_node.py: saluda / duerme / mira, con reconocimiento de caras."""
import os, json, time, math, threading, unicodedata
from collections import deque

import numpy as np
import cv2
import pyaudio
from vosk import Model, KaldiRecognizer, SetLogLevel

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
from cv_bridge import CvBridge

try:
    from cryptography.fernet import Fernet
except Exception:
    Fernet = None

AUTHORIZED = {'roberto', 'francisco'}            # unicos que pueden guardar su cara
KEY_PATH = os.path.expanduser('~/.yaren/face.key')


def load_fernet():
    """Clave: env YAREN_FACE_KEY o ~/.yaren/face.key (0600, se crea sola)."""
    if Fernet is None:
        return None
    key = os.environ.get('YAREN_FACE_KEY', '').strip().encode()
    try:
        if not key:
            if os.path.exists(KEY_PATH):
                key = open(KEY_PATH, 'rb').read().strip()
            else:
                os.makedirs(os.path.dirname(KEY_PATH), mode=0o700, exist_ok=True)
                key = Fernet.generate_key()
                fd = os.open(KEY_PATH, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
                with os.fdopen(fd, 'wb') as f:
                    f.write(key)
        return Fernet(key)
    except Exception:
        return None


def _wipe(path):
    try:
        n = os.path.getsize(path)
        with open(path, 'r+b') as f:
            f.write(b'\0' * n); f.flush(); os.fsync(f.fileno())
    except Exception:
        pass
    try:
        os.remove(path)
    except Exception:
        pass


GREET_MAX = 8.0    # s max del saludo
SLEEP_S = 8.0      # s que dura la cara de dormir

# poses estilo yaren_dance_radio: (hombro, abduccion, rotacion, codo)
A_REST = (0.0, 0.0, 0.0, 0.5)
A_UP   = (3.0, 0.0, 3.0, 0.0)
A_WAVE = (3.0, 0.8, 3.0, 0.0)


def pose12(r=A_REST, l=A_REST, j1=0.0, j3=0.0, j4=0.0):
    return [j1, 0.0, j3, j4, r[0], r[1], -r[2], r[3], -l[0], l[1], l[2], l[3]]


# Saludo: sube la mano, 3 olas moviendo mano y cabeza, giro final, baja a REST (~7.3 s)
SALUDO = [(pose12(A_UP, j1=0.2, j3=0.3), 1.5)]
for _ in range(3):
    SALUDO += [(pose12(A_WAVE, j1=0.2, j3=-0.3), 0.4),
               (pose12(A_UP, j1=0.2, j3=0.3), 0.4)]
SALUDO += [(pose12(A_UP, j1=-0.2), 0.4), (pose12(), 1.6)]


def build_traj(routine):
    P = [p for p, _ in routine]; T, acc = [], 0.0
    for _, t in routine:
        acc += t; T.append(acc)
    pts = []
    for i, p in enumerate(P):
        vel = [0.0] * 12
        if 0 < i < len(P) - 1:
            for j in range(12):
                s0 = (P[i][j] - P[i-1][j]) / (T[i] - T[i-1])
                s1 = (P[i+1][j] - P[i][j]) / (T[i+1] - T[i])
                if s0 * s1 > 0:
                    v = (s0 + s1) / 2
                    vel[j] = math.copysign(min(abs(v), 3 * min(abs(s0), abs(s1))), v)
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in p]; pt.velocities = vel
        pt.time_from_start = Duration(sec=int(T[i]), nanosec=int((T[i] % 1) * 1e9))
        pts.append(pt)
    return pts, T[-1]


JOINTS = [f'joint_{i}' for i in range(1, 13)]
REST = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5]
ARM_UP = [0.0, 0.0, 0.0, 0.0, 3.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.5]
ARM_LOW = list(ARM_UP); ARM_LOW[4] = 2.2

TXT = {
    'greet':       ("¡Hola! ¿Cómo estás?", "Hello! How are you?"),
    'greet_known': ("Hola {name}, ¿cómo vas hoy?", "Hi {name}, how are you doing today?"),
    'greet_new':   ("¡Hola! Mi nombre es Yaren, mucho gusto.", "Hello! My name is Yaren, nice to meet you."),
    'greet_all':   ("¡Hola a todos! Mi nombre es Yaren.", "Hello everyone! My name is Yaren."),
    'look1':       ("¡Ok, te miro!", "Okay, I'm looking at you!"),
    'look_known':  ("¡Ok {name}, te miro!", "Okay {name}, I'm looking at you!"),
    'ask_name':    ("¿Cómo te llamas?", "What is your name?"),
    'nice':        ("Un gusto, {name}.", "Nice to meet you, {name}."),
    'noname':      ("No te entendí bien, pero igual te miro.", "I didn't catch that, but I'll look at you anyway."),
    'many':        ("¡Wow! Son muchos, no sé a cuál mirar.", "Wow! There are so many, I don't know who to look at."),
    'none':        ("No logro verte. Ponte frente a mí.", "I can't see you. Please stand in front of me."),
    'nocam':       ("No puedo usar mi cámara ahora.", "I can't use my camera right now."),
}

GRAMMAR = ["saluda robot", "saluda", "saludar", "robot saluda",
           "duerme robot", "duerme", "robot duerme",
           "mírame robot", "mírame", "mira robot", "mira", "robot mira",
           "preséntate robot", "preséntate", "presentate", "[unk]"]


def norm(s):
    s = unicodedata.normalize('NFD', s.lower())
    return ''.join(c for c in s if unicodedata.category(c) != 'Mn').strip()


def parse(text, require_robot):
    t = norm(text)
    if not t or (require_robot and 'robot' not in t):
        return None
    c = t.replace(' ', '')
    if 'present' in c: return 'intro'
    if 'saluda' in c: return 'greet'
    if 'duerme' in c: return 'sleep'
    if 'mira' in c:   return 'look'
    return None


ALIASES = {
    'muerto': 'roberto', 'roberto': 'roberto', 'robert': 'roberto', 'alberto': 'roberto',
    'francisco': 'francisco', 'fransisco': 'francisco', 'cisco': 'francisco',
}


def clean_name(text):
    drop = {'me', 'llamo', 'mi', 'nombre', 'es', 'soy', 'yo', 'el', 'la', 'hola', 'pues', 'eh', 'um', '[unk]'}
    w = [x for x in text.lower().split() if x not in drop]
    for x in w:
        if x in ALIASES:
            return ALIASES[x].title()
    return ' '.join(w[:2]).title()

class FaceDB:
    def __init__(self, folder, det_path, rec_path, thr):
        self.dir, self.thr = folder, thr
        os.makedirs(folder, exist_ok=True)
        try:
            os.chmod(folder, 0o700)
        except Exception:
            pass
        self.path = os.path.join(folder, 'faces.enc')
        self.old_path = os.path.join(folder, 'faces.json')
        self.fer = load_fernet()
        self.db, self.locked = {}, False
        self._load()
        self.det = self.rec = None
        try:
            self.det = cv2.FaceDetectorYN.create(det_path, '', (320, 320), 0.85, 0.3, 5000)
            self.rec = cv2.FaceRecognizerSF.create(rec_path, '')
        except Exception:
            self.det = self.rec = None

    def _write_enc(self, path, data):
        tmp = path + '.tmp'
        fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, 'wb') as f:
            f.write(self.fer.encrypt(data))
        os.replace(tmp, path)

    def _save(self):
        raw = {k: [e.flatten().tolist() for e in v] for k, v in self.db.items()}
        self._write_enc(self.path, json.dumps(raw).encode())

    def _load(self):
        raw, legacy = None, False
        try:
            if self.fer is not None and os.path.exists(self.path):
                raw = json.loads(self.fer.decrypt(open(self.path, 'rb').read()))
            elif os.path.exists(self.old_path):
                raw = json.load(open(self.old_path)); legacy = True
        except Exception:
            raw = None
            self.locked = self.fer is not None and os.path.exists(self.path)
        if raw:
            self.db = {k: [np.array(v, np.float32).reshape(1, -1) for v in vs] for k, vs in raw.items()}
        if legacy and self.fer is not None:           # migra texto plano -> cifrado
            try:
                self._save()
                _wipe(self.old_path)
                for fn in os.listdir(self.dir):
                    if fn.lower().endswith('.jpg'):
                        p = os.path.join(self.dir, fn)
                        self._write_enc(p + '.enc', open(p, 'rb').read())
                        _wipe(p)
            except Exception:
                pass

    @property
    def ok(self):
        return self.det is not None and self.rec is not None

    def detect(self, img):
        h, w = img.shape[:2]
        self.det.setInputSize((w, h))
        _, f = self.det.detect(img)
        return [] if f is None else list(f)

    def embed(self, img, row):
        return self.rec.feature(self.rec.alignCrop(img, row))

    def identify(self, emb):
        best, score = None, 0.0
        for n, es in self.db.items():
            for e in es:
                s = self.rec.match(emb, e, cv2.FaceRecognizerSF_FR_COSINE)
                if s > score: best, score = n, s
        return (best, score) if score >= self.thr else (None, score)

    def add(self, name, embs, crop):
        if self.fer is None:
            raise RuntimeError('sin cryptography/clave: no se guarda la cara')
        if self.locked:
            raise RuntimeError('faces.enc no se pudo descifrar (clave distinta)')
        lst = self.db.setdefault(name, []) + [e.reshape(1, -1) for e in embs]
        self.db[name] = lst[-8:]
        self._save()
        if crop is not None and crop.size:
            ok, buf = cv2.imencode('.jpg', crop)
            if ok:
                self._write_enc(os.path.join(self.dir, f"{name.replace('/', '_')}_{int(time.time())}.jpg.enc"), buf.tobytes())


class VoiceCommandsNode(Node):
    def __init__(self):
        super().__init__('voice_commands_node')
        P = self.declare_parameter
        ws = os.getcwd()
        P('model_path', os.path.join(ws, 'src', 'YAREN2', 'yaren_chat', 'models', 'STT', 'vosk-model-small-es-0.42'))
        P('speak_topic', '/yaren/speak_direct')
        P('image_topic', '/image_raw')
        P('traj_topic', '/joint_trajectory_controller/joint_trajectory')
        P('tts_node', 'tts_lifecycle_node')
        P('cam_node', 'csi_cam_node')
        P('faces_dir', os.path.join(os.path.dirname(os.path.realpath(__file__)), 'faces_db'))
        P('match_thr', 0.363)
        P('require_robot', False)
        P('pan_joint_index', 2)
        P('pan_limit', 0.7)
        P('pan_sign', 1.0)
        P('track_gain', 0.35)
        P('track_max_step', 0.04)
        P('post_tts_tail', 0.8)
        P('video_path', os.path.join(ws, 'src', 'YAREN2', 'yaren_radio', 'videos', 'presentacion_yaren.mp4'))
        P('video_max', 180.0)
        P('enroll_enabled', True)
        g = lambda n: self.get_parameter(n).value
        self.speak_topic, self.pan_idx = g('speak_topic'), g('pan_joint_index')
        self.pan_limit, self.pan_sign = g('pan_limit'), g('pan_sign')
        self.gain, self.max_step = g('track_gain'), g('track_max_step')
        self.tail, self.require_robot = g('post_tts_tail'), g('require_robot')
        self.tts_node, self.cam_node = g('tts_node'), g('cam_node')
        self.video_path, self.video_max = g('video_path'), g('video_max')
        self.enroll_enabled = bool(g('enroll_enabled'))
        self._video_done = threading.Event()

        model_path = g('model_path')
        if not os.path.exists(model_path):
            self.get_logger().error(f'Vosk model not found: {model_path}')
            raise SystemExit(1)
        SetLogLevel(-1)
        self.model = Model(model_path)
        self.rec = KaldiRecognizer(self.model, 16000, json.dumps(GRAMMAR, ensure_ascii=False))

        fd = g('faces_dir')
        self.fdb = FaceDB(fd, os.path.join(fd, 'face_detection_yunet_2023mar.onnx'),
                          os.path.join(fd, 'face_recognition_sface_2021dec.onnx'), g('match_thr'))
        if self.fdb.ok:
            self.get_logger().info(f'Reconocimiento facial OK. Personas guardadas: {list(self.fdb.db)}')
        else:
            self.get_logger().warn('Modelos YuNet/SFace no disponibles: solo conteo con Haar, sin nombres.')
        self.cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

        self.tts_active = False
        self._tts_seen = False
        self._tts_end = 0.0
        self._speaking = False
        self.ui_state = ''
        self.mic_owner = 'none'
        self.is_english = False
        self.state = 'none'
        self._pose = list(REST)
        self._pan = 0.0
        self._detect_on = False
        self._last_det = 0.0
        self._hist = deque(maxlen=60)       # (t, n, boxes, width)
        self._frames = deque(maxlen=6)      # (t, img, rows)
        self._we_activated = set()
        self._run_lock = threading.Lock()
        self._cancel = threading.Event()
        self._last_cmd = ('', 0.0)
        self._capture = None
        self._run = True
        self.stream = None
        self.mic = pyaudio.PyAudio()
        self.bridge = CvBridge()
        self._img_param = g('image_topic'); self._img_sub = None; self._img_topic = None

        tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cb = ReentrantCallbackGroup()
        self.create_subscription(Bool, '/audio_playing', self._cb_tts, 10)
        self.create_subscription(String, '/yaren/ui_state', self._cb_ui, tl)
        self.create_subscription(String, '/yaren/mic_owner', self._cb_mic, tl)
        self.create_subscription(Bool, '/yaren/is_english', self._cb_lang, tl)
        self.speak_pub = self.create_publisher(String, self.speak_topic, 10)
        self.traj_pub = self.create_publisher(JointTrajectory, g('traj_topic'), 10)
        self.state_pub = self.create_publisher(String, '/yaren/voice_state', tl)
        self.video_pub = self.create_publisher(String, '/yaren/play_video', tl)
        self.create_subscription(Bool, '/yaren/video_done', lambda m: self._video_done.set(), 10)
        self._lc = {}
        for n in (self.tts_node, self.cam_node):
            self._lc[n] = (self.create_client(GetState, f'/{n}/get_state', callback_group=cb),
                           self.create_client(ChangeState, f'/{n}/change_state', callback_group=cb))
        self._set_state('none')
        threading.Thread(target=self._audio_loop, daemon=True).start()
        self.get_logger().info('voice_commands_node ready')

    # ------------------------------------------------------------ callbacks
    def _cb_tts(self, m):
        self.tts_active = m.data
        if m.data: self._tts_seen = True
        else: self._tts_end = time.monotonic()

    def _cb_ui(self, m):
        self.ui_state = m.data
        if m.data != 'face' and self.state != 'video':
            self._cancel.set()
            self._detect_on = False
            self._release_camera()
            self._set_state('none')

    def _cb_mic(self, m): self.mic_owner = m.data
    def _cb_lang(self, m): self.is_english = m.data

    def _cb_image(self, msg):
        if not self._detect_on:
            return
        now = time.monotonic()
        if now - self._last_det < 0.15:
            return
        self._last_det = now
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return
        small = cv2.resize(img, None, fx=480.0 / img.shape[1], fy=480.0 / img.shape[1])
        if self.fdb.ok:
            rows = self.fdb.detect(small)
            boxes = [tuple(int(v) for v in r[:4]) for r in rows]
        else:
            gray = cv2.equalizeHist(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY))
            boxes = [tuple(int(v) for v in f) for f in self.cascade.detectMultiScale(gray, 1.15, 5, minSize=(40, 40))]
            rows = []
        self._hist.append((now, len(boxes), boxes, small.shape[1]))
        self._frames.append((now, small, rows))

    def _bind_image(self, cancel, wait=8.0):
        t_end = time.monotonic() + wait
        while not cancel.is_set() and time.monotonic() < t_end:
            cands = [n for n, ty in self.get_topic_names_and_types()
                     if 'sensor_msgs/msg/Image' in ty and 'face_screen' not in n]
            pick = self._img_param if self._img_param in cands else (
                sorted(cands, key=lambda n: ('cam' not in n and 'raw' not in n, n))[0] if cands else None)
            if pick:
                if pick != self._img_topic:
                    if self._img_sub: self.destroy_subscription(self._img_sub)
                    self._img_sub = self.create_subscription(Image, pick, self._cb_image, qos_profile_sensor_data)
                    self._img_topic = pick
                    self.get_logger().info(f'image topic: {pick} (candidatos: {cands})')
                return True
            cancel.wait(0.5)
        self.get_logger().warn('no hay topics de imagen')
        return False

    # ------------------------------------------------------------ audio
    def _mic_allowed(self):
        return self.ui_state == 'face' and self.mic_owner in ('none', 'wake_word') and self.state != 'video'

    def _guard(self):
        return self._speaking or self.tts_active or time.monotonic() < self._tts_end + self.tail

    def _reset_recs(self):
        self.rec.Reset()

    def _on_audio(self, data):
        cap = self._capture
        if cap is not None:                      # modo captura de nombre
            if cap['rec'].AcceptWaveform(data):
                t = json.loads(cap['rec'].Result()).get('text', '').strip()
                if t:
                    cap['text'] = t; cap['ev'].set()
            return
        if self.rec.AcceptWaveform(data):
            text = json.loads(self.rec.Result()).get('text', '')
            cmd = parse(text, self.require_robot)
            if text: self.get_logger().info(f'heard: "{text}" -> {cmd}')
            if cmd: self._dispatch(cmd)

    def _audio_loop(self):
        was_guard = False
        while self._run:
            try:
                if not self._mic_allowed():
                    self._close_stream(); time.sleep(0.2); continue
                if self.stream is None:
                    self.stream = self.mic.open(format=pyaudio.paInt16, channels=1, rate=16000,
                                                input=True, frames_per_buffer=4000)
                data = self.stream.read(2000, exception_on_overflow=False)
                if self._guard():
                    was_guard = True
                    continue
                if was_guard:
                    self._reset_recs(); was_guard = False
                    continue
                self._on_audio(data)
            except Exception as e:
                self.get_logger().warn(f'audio loop: {e}')
                self._close_stream(); time.sleep(0.5)

    def _close_stream(self):
        if self.stream is not None:
            try:
                self.stream.stop_stream(); self.stream.close()
            except Exception:
                pass
            self.stream = None

    def _listen_name(self, cancel, timeout=9.0):
        cap = {'rec': KaldiRecognizer(self.model, 16000), 'text': '', 'ev': threading.Event()}
        self._capture = cap
        try:
            t0 = time.monotonic()
            while not cancel.is_set() and not cap['ev'].is_set() and time.monotonic() - t0 < timeout:
                cancel.wait(0.1)
        finally:
            self._capture = None
        self.get_logger().info(f'nombre oído: "{cap["text"]}"')
        return clean_name(cap['text']) if cap['text'] else ''

    # ------------------------------------------------------------ dispatch
    def _dispatch(self, cmd):
        now = time.monotonic()
        if cmd == self._last_cmd[0] and now - self._last_cmd[1] < 2.0:
            return
        self._last_cmd = (cmd, now)
        self._cancel.set()
        cancel = self._cancel = threading.Event()
        fn = {'greet': self._beh_greet, 'sleep': self._beh_sleep,
              'look': self._beh_look, 'intro': self._beh_intro}[cmd]
        threading.Thread(target=self._run_behavior, args=(fn, cancel), daemon=True).start()

    def _run_behavior(self, fn, cancel):
        with self._run_lock:
            if cancel.is_set():
                return
            final = 'none'
            try:
                final = fn(cancel) or 'none'
            except Exception as e:
                self.get_logger().error(f'{fn.__name__}: {e}')
                self._capture = None
                self._release_camera()
            if not cancel.is_set():
                if final != 'look':
                    self._detect_on = False
                    self._release_camera()
                self._set_state('none' if final == 'look' else final)

    def _set_state(self, s):
        self.state = s
        self.state_pub.publish(String(data=s))

    # ------------------------------------------------------------ motors
    def _send(self, pos, secs):
        m = JointTrajectory(); m.joint_names = JOINTS
        p = JointTrajectoryPoint(); p.positions = [float(v) for v in pos]
        p.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        m.points = [p]
        self.traj_pub.publish(m)
        self._pose = list(pos)

    def _send_traj(self, routine):
        m = JointTrajectory(); m.joint_names = JOINTS
        m.points, total = build_traj(routine)
        self.traj_pub.publish(m)
        self._pose = None          # fuerza _rest_if_needed si se cancela a mitad
        return total

    def _pose_with_pan(self, pan):
        pan = max(-self.pan_limit, min(self.pan_limit, pan))
        self._pan = pan
        p = list(REST); p[self.pan_idx] = pan
        return p

    def _rest_if_needed(self, cancel):
        if self._pose != REST:
            self._send(REST, 1.5); self._pan = 0.0
            cancel.wait(1.6)

    # ------------------------------------------------------------ lifecycle / tts
    def _call(self, client, req, timeout):
        ev = threading.Event()
        fut = client.call_async(req)
        fut.add_done_callback(lambda f: ev.set())
        return fut.result() if ev.wait(timeout) else None

    def _state(self, node):
        gs, _ = self._lc[node]
        if not gs.wait_for_service(3.0):
            return None
        r = self._call(gs, GetState.Request(), 3.0)
        return None if r is None else r.current_state.id

    def _ensure_active(self, node, timeout=20.0):
        _, cs = self._lc[node]
        for _ in range(3):
            st = self._state(node)
            if st is None: return False
            if st == State.PRIMARY_STATE_ACTIVE: return True
            if st == State.PRIMARY_STATE_UNCONFIGURED: tid = Transition.TRANSITION_CONFIGURE
            elif st == State.PRIMARY_STATE_INACTIVE: tid = Transition.TRANSITION_ACTIVATE
            else: return False
            rq = ChangeState.Request(); rq.transition.id = tid
            res = self._call(cs, rq, timeout)
            if not (res and res.success): return False
            self._we_activated.add(node)
        return self._state(node) == State.PRIMARY_STATE_ACTIVE

    def _release_camera(self):
        if self.cam_node in self._we_activated:
            self._we_activated.discard(self.cam_node)
            rq = ChangeState.Request(); rq.transition.id = Transition.TRANSITION_DEACTIVATE
            self._lc[self.cam_node][1].call_async(rq)

    def _say(self, key, cancel, tick=None, max_t=None, **fmt):
        es, en = TXT[key]
        text = (en if self.is_english else es).format(**fmt)
        limit = time.monotonic() + max_t if max_t else float('inf')
        ok = lambda: not cancel.is_set() and time.monotonic() < limit
        self._ensure_active(self.tts_node, 10.0)
        self._speaking = True
        try:
            self._tts_seen = False
            self.speak_pub.publish(String(data=text))
            t0 = time.monotonic()
            while ok() and not self._tts_seen and time.monotonic() - t0 < 4.0:
                if tick: tick()
                cancel.wait(0.1)
            if not self._tts_seen:
                end = time.monotonic() + 1.5 + 0.07 * len(text)
                while ok() and time.monotonic() < end:
                    if tick: tick()
                    cancel.wait(0.1)
            while ok() and self.tts_active:
                if tick: tick()
                cancel.wait(0.1)
        finally:
            self._speaking = False
            self._tts_end = time.monotonic()

    # ------------------------------------------------------------ visión
    def _observe(self, cancel, secs=2.5):
        """Activa la cámara, observa `secs` y devuelve {'n': personas, 'samples': [(img,row)]} o None."""
        if not self._ensure_active(self.cam_node, 30.0):
            return None
        if cancel.is_set():
            return None
        self._hist.clear(); self._frames.clear(); self._detect_on = True
        if not self._bind_image(cancel):
            return None
        t0 = time.monotonic()
        cancel.wait(secs)
        fr = [(c, b) for (ts, c, b, _) in list(self._hist) if ts >= t0]
        if cancel.is_set() or len(fr) < 3:
            if len(fr) < 3: self.get_logger().warn('no camera frames (check image_topic)')
            return None
        counts = [c for c, _ in fr]
        nz = [c for c in counts if c > 0]
        n = 0 if len(nz) < 0.4 * len(counts) else max(set(nz), key=lambda c: (nz.count(c), -c))
        samples = []
        if n == 1:
            samples = [(img, rows[0]) for (_, img, rows) in list(self._frames) if len(rows) == 1]
        self.get_logger().info(f'observado: {n} persona(s), {len(samples)} muestras')
        return {'n': n, 'samples': samples}

    def _who(self, obs):
        if not self.fdb.ok or not obs['samples']:
            return None
        votes = {}
        for img, row in obs['samples']:
            name, sc = self.fdb.identify(self.fdb.embed(img, row))
            if name: votes[name] = votes.get(name, 0) + 1
        if not votes:
            return None
        name = max(votes, key=votes.get)
        self.get_logger().info(f'reconocido: {name} ({votes[name]}/{len(obs["samples"])})')
        return name if votes[name] >= max(1, len(obs['samples']) // 2) else None

    def _enroll(self, name, obs):
        first = norm(name).split(' ')[0] if name else ''
        if not self.enroll_enabled or first not in AUTHORIZED:
            self.get_logger().warn(f'registro de cara NO autorizado: "{name}"')
            return False
        canon = first.title()
        if canon in self.fdb.db and self._who(obs) != canon:   # anti-suplantacion
            self.get_logger().warn(f'{canon} ya esta registrado y esta cara no coincide: no se guarda')
            return False
        try:
            smp = obs['samples'][-4:]
            embs = [self.fdb.embed(img, row) for img, row in smp]
            img, row = smp[-1]
            x, y, w, h = [int(v) for v in row[:4]]
            crop = img[max(0, y):y + h, max(0, x):x + w]
            self.fdb.add(canon, embs, crop)
        except Exception as e:
            self.get_logger().error(f'no se pudo guardar la cara: {e}')
            return False
        self.get_logger().info(f'cara guardada (cifrada): {canon}')
        return True

    # ------------------------------------------------------------ behaviors
    def _prepare(self, cancel, keep_camera=False):
        self._detect_on = False
        if not keep_camera:
            self._release_camera()
        self._set_state('busy')

    def _beh_greet(self, cancel):
        t0 = time.monotonic()
        self._prepare(cancel, keep_camera=True)

        # 1) mirar primero (max 1.8 s) con el robot quieto
        res, oc = {}, threading.Event()
        th = threading.Thread(target=lambda: res.update(obs=self._observe(oc, 1.2)), daemon=True)
        th.start(); th.join(1.8); oc.set()
        obs = res.get('obs')
        self._detect_on = False
        if cancel.is_set():
            return

        key, fmt = 'greet', {}
        if obs and obs['n'] >= 2:
            key = 'greet_all'
        elif obs and obs['n'] == 1:
            name = self._who(obs)
            key, fmt = ('greet_known', {'name': name}) if name else ('greet_new', {})

        # 2) TTS listo ANTES de mover, para que voz y gesto arranquen juntos
        self._ensure_active(self.tts_node, 10.0)
        if cancel.is_set():
            return
        t1 = time.monotonic()
        total = self._send_traj(SALUDO)
        left = max(1.0, GREET_MAX - (t1 - t0) - 0.2)
        self._say(key, cancel, max_t=left, **fmt)

        # 3) espera a que termine el gesto sin pasar de 8 s en total
        cancel.wait(max(0.0, min(t1 + total, t0 + GREET_MAX) - time.monotonic()))
        if cancel.is_set() or time.monotonic() < t1 + total - 0.2:
            self._send(REST, 1.0)                       # corta el gesto y baja
        self._pose = list(REST); self._pan = 0.0
        return 'none'
    def _beh_sleep(self, cancel):
        self._prepare(cancel)
        self._rest_if_needed(cancel)
        self._set_state('sleep')
        cancel.wait(SLEEP_S)
        return 'none'          # _run_behavior publica 'none' -> la cara vuelve

    def _beh_intro(self, cancel):
        self._prepare(cancel)
        self._rest_if_needed(cancel)
        self._video_done.clear()
        self.video_pub.publish(String(data=self.video_path))   # primero la ruta
        self._set_state('video')                                # luego el estado
        t0 = time.monotonic()
        while (not cancel.is_set() and not self._video_done.is_set()
               and time.monotonic() - t0 < self.video_max):
            cancel.wait(0.2)
        return 'none'

    def _beh_look(self, cancel):
        self._prepare(cancel, keep_camera=True)
        self._rest_if_needed(cancel)
        obs = self._observe(cancel, 2.5)
        if cancel.is_set():
            return 'none'
        if obs is None:
            self._say('nocam', cancel); return 'none'
        if obs['n'] == 0:
            self._say('none', cancel); return 'none'
        if obs['n'] >= 2:                         # muchas personas: lo dice y termina ahí
            self._say('many', cancel); return 'none'
        name = self._who(obs)
        if name:
            self._say('look_known', cancel, name=name)
        else:
            self._say('look1', cancel)
            if self.fdb.ok and obs['samples'] and not cancel.is_set():
                self._say('ask_name', cancel)
                nm = self._listen_name(cancel)
                if cancel.is_set():
                    return 'none'
                if nm:
                    self._enroll(nm, obs)
                    self._say('nice', cancel, name=nm)
                else:
                    self._say('noname', cancel)
        if cancel.is_set():
            return 'none'
        self._track(cancel)
        return 'look'

    def _track(self, cancel):
        self._set_state('busy')
        last_seen = 0
        while not cancel.is_set():
            if self._hist and self._hist[-1][0] > last_seen and self._hist[-1][1] > 0:
                ts, _, boxes, w = self._hist[-1]
                last_seen = ts
                x, y, fw, fh = max(boxes, key=lambda b: b[2] * b[3])
                err = (x + fw / 2.0) / w - 0.5
                if abs(err) > 0.06:
                    step = max(-self.max_step, min(self.max_step, self.pan_sign * self.gain * err))
                    self._send(self._pose_with_pan(self._pan + step), 0.25)
            cancel.wait(0.1)

    def shutdown(self):
        self._run = False
        self._cancel.set()
        self._close_stream()
        try:
            self.mic.terminate()
        except Exception:
            pass


def main():
    rclpy.init()
    node = VoiceCommandsNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
