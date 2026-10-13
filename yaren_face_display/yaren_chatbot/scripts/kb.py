import sqlite3, re, unicodedata
from difflib import SequenceMatcher

STOP = set("""a al algo ante con como cual cuales cuando de del el ella ellos en es esta este esto
la las lo los me mi mis muy no o para pero por que quien se si sin sobre su sus te tu tus un una uno y yo
puedes puedo dime dame favor hola""".split())


def norm(text):
    t = unicodedata.normalize("NFD", text.lower())
    t = "".join(c for c in t if unicodedata.category(c) != "Mn")
    return re.sub(r"[^a-z0-9\s]", " ", t)


def tokens(text):
    return [t for t in norm(text).split() if t not in STOP and len(t) > 1]


SCHEMA = """
CREATE TABLE IF NOT EXISTS qa(
  id INTEGER PRIMARY KEY, question TEXT, norm_q TEXT, answer TEXT, category TEXT);
CREATE VIRTUAL TABLE IF NOT EXISTS qa_fts USING fts5(norm_q, content='qa', content_rowid='id');
CREATE TABLE IF NOT EXISTS pending(
  id INTEGER PRIMARY KEY, question TEXT, ts DATETIME DEFAULT CURRENT_TIMESTAMP, answered INTEGER DEFAULT 0);
"""


class KB:
    def __init__(self, path, threshold=0.45):
        self.db = sqlite3.connect(path, check_same_thread=False)
        self.db.executescript(SCHEMA)
        self.threshold = threshold

    def add(self, question, answer, category=""):
        nq = " ".join(tokens(question))
        cur = self.db.execute(
            "INSERT INTO qa(question,norm_q,answer,category) VALUES(?,?,?,?)",
            (question, nq, answer, category))
        self.db.execute("INSERT INTO qa_fts(rowid,norm_q) VALUES(?,?)", (cur.lastrowid, nq))

    def search(self, text, k=15):
        """Devuelve (respuesta, score) o (None, score)."""
        toks = tokens(text)
        if not toks:
            return None, 0.0
        match = " OR ".join(f'"{t}"*' for t in toks)
        rows = self.db.execute(
            "SELECT qa.norm_q, qa.answer FROM qa_fts JOIN qa ON qa.id = qa_fts.rowid "
            "WHERE qa_fts MATCH ? ORDER BY bm25(qa_fts) LIMIT ?", (match, k)).fetchall()
        q = " ".join(toks)
        qs = set(toks)
        best, best_s = None, 0.0
        for nq, ans in rows:
            s_set = set(nq.split())
            jac = len(qs & s_set) / max(1, len(qs | s_set))
            s = 0.5 * jac + 0.5 * SequenceMatcher(None, q, nq).ratio()
            if s > best_s:
                best, best_s = ans, s
        return (best if best_s >= self.threshold else None), best_s

    def log_pending(self, question):
        self.db.execute("INSERT INTO pending(question) VALUES(?)", (question,))
        self.db.commit()
