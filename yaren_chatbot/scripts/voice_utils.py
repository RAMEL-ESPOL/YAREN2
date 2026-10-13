"""Utilidades de voz SIN dependencias de ROS (se pueden probar en cualquier PC)."""
import itertools, json, os, re, unicodedata
import numpy as np


def strip_accents(w):
    return "".join(c for c in unicodedata.normalize("NFD", w.lower()) if unicodedata.category(c) != "Mn")


# ---------------- STT ----------------
def load_vocab(model_path):
    """words.txt del modelo Vosk -> {palabra_sin_tildes: [variantes con tildes]}."""
    base = os.path.expanduser(model_path)
    for rel in ("graph/words.txt", "words.txt", "am/words.txt"):
        p = os.path.join(base, rel)
        if os.path.isfile(p):
            vocab = {}
            with open(p, encoding="utf-8") as f:
                for line in f:
                    parts = line.split()
                    if not parts or parts[0][0] in "<#[":
                        continue
                    vocab.setdefault(strip_accents(parts[0]), set()).add(parts[0])
            return {k: sorted(v) for k, v in vocab.items()}
    return {}


def build_grammar(hint, vocab, max_combos=6):
    """Lista de palabras/frases esperadas -> JSON de gramática para Vosk (o None = sin restricción)."""
    phrases = []
    for h in hint:
        parts = strip_accents(h).split()
        if not parts:
            continue
        if not vocab:
            phrases.append(" ".join(parts))
            continue
        opts = [vocab.get(p) for p in parts]
        if any(o is None for o in opts):          # palabra fuera del vocabulario del modelo
            continue
        for combo in itertools.islice(itertools.product(*opts), max_combos):
            phrases.append(" ".join(combo))
    phrases = sorted(set(phrases))
    return json.dumps(phrases + ["[unk]"], ensure_ascii=False) if phrases else None


def choose_text(free, gram):
    """Combina el reconocedor libre y el de gramática restringida.
    Devuelve (texto, fuente, ininteligible).
    - Si la gramática entendió todo con palabras esperadas -> se confía en ella (más precisa).
    - Si no, se usa el libre (permite preguntas fuera del tema).
    - Si no hay texto pero la gramática oyó algo ([unk]) -> ininteligible."""
    free, gram = (free or "").strip().lower(), (gram or "").strip().lower()
    if gram and "[unk]" not in gram:
        return gram, "gramática", False
    if free:
        return free, "libre", False
    return "", "", "[unk]" in gram


def apply_gain(pcm, gain):
    if gain == 1.0:
        return pcm
    return np.clip(pcm.astype(np.float32) * gain, -32768, 32767).astype(np.int16)


def resample(pcm, r_in, r_out):
    if r_in == r_out or len(pcm) == 0:
        return pcm
    try:
        from scipy.signal import resample_poly
        from math import gcd
        g = gcd(int(r_in), int(r_out))
        return resample_poly(pcm.astype(np.float32), int(r_out) // g, int(r_in) // g).astype(np.int16)
    except ImportError:
        n = int(len(pcm) * r_out / r_in)
        return np.interp(np.linspace(0, len(pcm) - 1, n), np.arange(len(pcm)), pcm).astype(np.int16)


# ---------------- TTS ----------------
DEFAULT_PRON = {
    "YAREN": "Yaren", "ESPOL": "Espol", "RAMEL": "Ramel", "FIMCP": "fi em ce pe", "FIEC": "fiec",
    "FICT": "fict", "FCNM": "efe ce ene eme", "ROS 2": "ros dos", "H2O": "hache dos o",
    "911": "nueve uno uno", "WALL-E": "Wali", "R2-D2": "erre dos de dos", "Just Dance": "Yast Dans",
    "Paint": "Peint", "Dice": "Dais", "D. C.": "de ce", "NASA": "Nasa", "3D": "tres de",
}


def load_pron(path=""):
    pron = dict(DEFAULT_PRON)
    if path and os.path.isfile(os.path.expanduser(path)):
        with open(os.path.expanduser(path), encoding="utf-8") as f:
            pron.update(json.load(f))
    return pron


def normalize_tts(text, pron):
    t = re.sub(r"[\U0001F300-\U0001FAFF\u2600-\u27BF]", "", text)     # emojis
    t = re.sub(r"[*_`#~<>|\[\]{}]", " ", t)
    for k in sorted(pron, key=len, reverse=True):
        t = re.sub(rf"(?<!\w){re.escape(k)}(?!\w)", pron[k], t)
    return re.sub(r"\s+", " ", t).strip()


def split_sentences(text, maxlen=220):
    """Frases cortas: el robot empieza a hablar antes de terminar de sintetizar todo."""
    out = []
    for p in re.split(r"(?<=[.!?…])\s+", text.strip()):
        p = p.strip()
        while len(p) > maxlen:
            cut = max(p.rfind(",", 0, maxlen), p.rfind(" ", 0, maxlen))
            cut = cut if cut > 40 else maxlen
            out.append(p[:cut].strip(" ,"))
            p = p[cut:].strip(" ,")
        if p:
            out.append(p)
    return out
