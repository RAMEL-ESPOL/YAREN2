import sqlite3, re, unicodedata
from difflib import SequenceMatcher

STOP = set("""a al algo ante con como cual cuales cuando de del el ella ellos en es esta este esto
la las lo los me mi mis muy no o para pero por que quien se si sin sobre su sus te tu tus un una uno y yo
puedes puedo dime dame favor hola yaren""".split())


def norm(text):
    t = unicodedata.normalize("NFD", text.lower())
    t = "".join(c for c in t if unicodedata.category(c) != "Mn")
    return re.sub(r"[^a-z0-9\s]", " ", t)


def tokens(text):
    allt = [t for t in norm(text).split() if len(t) > 1]
    return [t for t in allt if t not in STOP] or allt   # si todo es "stopword", conservar (ej. "hola")


def fono(w):
    """Clave fonética del español: b/v, ll/y, c/k/qu, s/z/ce/ci, g/j, h muda, rr, 's' final."""
    w = re.sub(r"[^a-z0-9]", "", w.lower())
    w = w.replace("ch", "X").replace("ll", "y")
    w = re.sub(r"gu([ei])", r"g\1", w)
    w = re.sub(r"g([ei])", r"j\1", w)
    w = re.sub(r"qu([ei])", r"k\1", w).replace("q", "k")
    w = re.sub(r"c([ei])", r"s\1", w)
    w = w.replace("c", "k").replace("z", "s").replace("v", "b").replace("h", "")
    w = w.replace("x", "ks").replace("y", "i").replace("w", "u")
    w = re.sub(r"(.)\1+", r"\1", w)
    if len(w) > 3:
        w = w.rstrip("s")
    return w


def lev(a, b):
    prev = list(range(len(b) + 1))
    for i, x in enumerate(a, 1):
        cur = [i]
        for j, y in enumerate(b, 1):
            cur.append(min(prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + (x != y)))
        prev = cur
    return prev[-1]


def same(a, b):
    """Igualdad tolerante: mismo sonido, plural/conjugación, o 1–2 letras distintas en palabras largas."""
    if a == b:
        return True
    fa, fb = fono(a), fono(b)
    if fa == fb:
        return True
    n = min(len(fa), len(fb))
    if n < 4:
        return False
    cp = 0
    for x, y in zip(fa, fb):
        if x != y:
            break
        cp += 1
    if cp >= max(4, n - 1) and abs(len(fa) - len(fb)) <= 3:   # plural / conjugación (juego-juegos, baile-bailar)
        return True
    m = max(len(fa), len(fb))
    return m >= 5 and abs(len(fa) - len(fb)) <= 1 and lev(fa, fb) <= (1 if m < 8 else 2)


SCHEMA = """
CREATE TABLE IF NOT EXISTS qa(
  id INTEGER PRIMARY KEY, question TEXT, norm_q TEXT, fono_q TEXT, answer TEXT, category TEXT);
CREATE VIRTUAL TABLE IF NOT EXISTS qa_fts USING fts5(norm_q, fono_q, content='qa', content_rowid='id');
CREATE TABLE IF NOT EXISTS pending(
  id INTEGER PRIMARY KEY, question TEXT, ts DATETIME DEFAULT CURRENT_TIMESTAMP, answered INTEGER DEFAULT 0);
"""


def _stem(t):
    return t[:max(4, len(t) - 2)] if len(t) >= 5 else t


class KB:
    def __init__(self, path, threshold=0.45, min_cov=0.6, min_qcov=0.45, scan_limit=20000):
        self.db = sqlite3.connect(path, check_same_thread=False)
        self.db.executescript(SCHEMA)
        self.threshold, self.min_cov, self.min_qcov, self.scan_limit = threshold, min_cov, min_qcov, scan_limit
        self._all = None

    def add(self, question, answer, category=""):
        nq = " ".join(tokens(question))
        fq = " ".join(fono(t) for t in nq.split())
        cur = self.db.execute(
            "INSERT INTO qa(question,norm_q,fono_q,answer,category) VALUES(?,?,?,?,?)",
            (question, nq, fq, answer, category))
        self.db.execute("INSERT INTO qa_fts(rowid,norm_q,fono_q) VALUES(?,?,?)", (cur.lastrowid, nq, fq))
        self._all = None

    def _score(self, toks, q, nq):
        ct = nq.split()
        if not ct:
            return 0.0
        cov = sum(any(same(c, t) for t in toks) for c in ct) / len(ct)
        qcov = sum(any(same(t, c) for c in ct) for t in toks) / len(toks)
        if cov < self.min_cov or qcov < self.min_qcov:
            return 0.0
        exact = sum(c in toks for c in ct) / len(ct)
        return 0.4 * cov + 0.25 * exact + 0.15 * qcov + 0.2 * SequenceMatcher(None, q, nq).ratio()

    def _best(self, rows, toks, q):
        best, best_s = None, 0.0
        for nq, ans in rows:
            s = self._score(toks, q, nq)
            if s > best_s:
                best, best_s = ans, s
        return best, best_s

    def search(self, text, k=40):
        """Devuelve (respuesta, score) o (None, score). Tolera errores de pronunciación/STT."""
        toks = tokens(text)
        if not toks:
            return None, 0.0
        stems = {_stem(t) for t in toks} | {_stem(fono(t)) for t in toks}
        match = " OR ".join(f'"{s}"*' for s in stems if s)
        q = " ".join(toks)
        rows = self.db.execute(
            "SELECT qa.norm_q, qa.answer FROM qa_fts JOIN qa ON qa.id = qa_fts.rowid "
            "WHERE qa_fts MATCH ? ORDER BY bm25(qa_fts) LIMIT ?", (match, k)).fetchall()
        best, best_s = self._best(rows, toks, q)
        if best_s < self.threshold:          # respaldo: recorrido completo (tolera errores raros)
            if self._all is None:
                self._all = self.db.execute("SELECT norm_q, answer FROM qa").fetchall()
            if len(self._all) <= self.scan_limit:
                best, best_s = self._best(self._all, toks, q)
        return (best if best_s >= self.threshold else None), best_s

    def log_pending(self, question):
        self.db.execute("INSERT INTO pending(question) VALUES(?)", (question,))
        self.db.commit()
