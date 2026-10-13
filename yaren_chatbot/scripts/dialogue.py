"""Motor de conversación guiada OFFLINE, pensado para voz de niños y STT con errores.

- Entiende por sonido (b/v, ll/y, c/k/s...), palabras cortadas ("fue bol") y errores de 1-2 letras.
- Si no está seguro de lo que entendió, CONFIRMA ("¿Dijiste fútbol?") antes de avanzar.
- Ignora ruido ("eh", "mmm", "[unk]"), entiende "repite" / "qué?" y reintenta con otras palabras.
- El flujo vive en data/conversacion.json.
"""
import json, random, re, time
from kb import norm, same, fono

GREET = {"hola", "ola", "buenas", "buenos", "hey", "saludos", "hello"}
CHAT_REQ = {"hablemos", "conversemos", "charlemos", "platiquemos", "conversar", "charlar"}
NOISE = {"eh", "ah", "mm", "mmm", "um", "uh", "em", "hm", "ehh", "aa", "ee", "oh", "unk", "uhm", "ajam"}
QWORDS = {"que", "como", "cual", "cuales", "cuando", "donde", "quien", "quienes", "cuanto", "cuanta",
          "cuantos", "cuantas", "por", "puedes", "sabes", "eres", "tienes", "dime", "cuentame",
          "explicame", "hay", "podemos", "quieres", "existe"}
STRICT_Q = QWORDS - {"por", "hay"}
FAREWELL = ["adios", "chao", "hasta luego", "nos vemos", "me voy", "hasta pronto", "hasta manana", "bye"]
CHANGE_TOPIC = ["cambiemos de tema", "otro tema", "hablemos de otra cosa", "de otra cosa", "algo diferente"]
REPEAT_PHRASES = ["repite", "otra vez", "no entendi", "no escuche", "como dices", "que dijiste",
                  "no te entiendo", "mas despacio", "repitelo", "repetir", "dilo otra vez", "puedes repetir"]
REPEAT_ONE = {"que", "como", "eh", "mande", "perdon", "cual", "hm", "ah"}
YES = {"si", "claro", "obvio", "dale", "ok", "listo", "tengo", "quiero", "vamos", "bueno", "aja", "ya",
       "seguro", "yes", "afirmativo", "tambien", "sabia", "gustaria", "ajá"}
NO = {"no", "nada", "nunca", "ninguno", "ninguna", "tampoco", "nop", "nel", "paso"}
YES_F = {fono(w) for w in YES}
NO_F = {fono(w) for w in NO}
COMMON = {"hola", "buenas", "buenos", "dias", "tardes", "noches", "bien", "mal", "gracias", "si", "no",
          "yaren", "que", "como", "cual", "nada", "ok", "hey", "saludos", "hello", "hi", "claro", "bueno",
          "ola", "ci", "nop", "eh", "mmm", "unk"}
NOT_NAME = {"de", "muy", "un", "una", "el", "la", "tu", "yo", "hincha", "nuevo", "nueva", "estudiante",
            "delantero", "portero", "feliz", "yaren", "amigo", "amiga", "bueno", "buena", "tan", "mi", "es"}
FILLER = {"pues", "bueno", "eh", "este", "me", "gusta", "gustan", "mi", "favorito", "favorita", "es", "son",
          "le", "voy", "a", "al", "soy", "hincha", "de", "se", "llama", "el", "la", "los", "las", "un", "una",
          "unos", "unas", "yo", "con", "que", "tengo", "estoy", "en", "del", "juego", "mas"}
NEG_FREE = {"no", "ninguno", "ninguna", "nadie", "nada", "no se", "no tengo", "no sé"}
WORD_RE = re.compile(r"[a-záéíóúüñ]+")


class SafeDict(dict):
    def __missing__(self, k):
        return ""


def is_questionish(text):
    if "?" in text or "¿" in text:
        return True
    toks = norm(text).split()
    return bool(toks) and toks[0] in QWORDS


def with_pairs(toks):
    """Añade palabras pegadas ("fue"+"bol" -> "fuebol"): el STT suele partir las palabras de los niños."""
    return toks + [a + b for a, b in zip(toks, toks[1:])]


def hit(kw, ntext, toks):
    """2 = coincidencia exacta, 1 = por sonido/parecido, 0 = nada."""
    k = norm(kw).strip()
    if not k:
        return 0
    if " " in k:
        return 2 if f" {k} " in f" {ntext} " else 0
    fk = fono(k)
    if k in toks or any(fono(t) == fk for t in toks):
        return 2
    return 1 if any(same(k, t) for t in with_pairs(toks)) else 0


def any_hit(lst, ntext, toks):
    return any(hit(k, ntext, toks) for k in lst)


class Dialogue:
    def __init__(self, flow_path, kb=None, timeout=180.0, seed=None):
        with open(flow_path, encoding="utf-8") as f:
            self.flow = json.load(f)
        self.nodes = self.flow["nodes"]
        self.kb = kb
        self.timeout = timeout
        self.rng = random.Random(seed)
        self.reset()

    # ---------- estado ----------
    def reset(self):
        self.node = "inicio"
        self.slots, self.visited, self.retries, self.last_pick = {}, set(), {}, {}
        self.pending = None
        self.last_ts = 0.0

    def in_flow(self):
        return self.node not in ("inicio", "libre")

    def start(self):
        """Inicia la charla por iniciativa del robot (ej. al detectar una persona)."""
        self.reset()
        self.last_ts = time.time()
        return self._enter("saludo")

    # ---------- utilidades ----------
    def _pick(self, key, variants):
        if len(variants) == 1:
            return variants[0]
        opts = [v for v in variants if v != self.last_pick.get(key)] or variants
        c = self.rng.choice(opts)
        self.last_pick[key] = c
        return c

    def _fmt(self, text, extra=None):
        ctx = SafeDict(self.slots)
        ctx["nombre"] = self.slots.get("nombre") or "amigo"
        ctx.update(extra or {})
        return text.format_map(ctx)

    def _say(self, nid, reask=False):
        n = self.nodes[nid]
        v = (n.get("reask") if reask else None) or n.get("say")
        return self._fmt(self._pick("say:" + nid, v)) if v else ""

    def _enter(self, nid):
        if nid == "otro":
            base = self.nodes["gustos2"]["options"]
            if not [k for k, o in base.items() if o.get("topic") not in self.visited]:
                nid = "cierre"
        self.node = nid
        self.retries[nid] = 0
        return self._say(nid)

    def reprompt(self):
        return self._say(self.node, reask=True) if self.in_flow() else ""

    def _kb(self, text, strong=False):
        if not self.kb:
            return None
        ans, s = self.kb.search(text)
        return ans if ans and (not strong or s >= 0.6) else None

    # ---------- pistas para el STT (gramática restringida) ----------
    def expected_words(self):
        """Palabras que se esperan ahora (vacío = respuesta libre). Útil para limitar el vocabulario del STT."""
        if not self.in_flow():
            return []
        words = set()
        if self.pending or self.nodes[self.node].get("parse") == "yesno":
            words |= YES | NO
        elif self.nodes[self.node].get("parse") == "options":
            for o in self._options(self.node).values():
                for k in o.get("kw", []):
                    words |= {norm(k).strip()} | set(norm(k).split())      # frase completa y palabras sueltas
            words |= {"si", "no", "gracias", "adios"}
        return sorted(words)

    # ---------- comprensión ----------
    def _extract_name(self, text, bare):
        words = WORD_RE.findall(text.lower())
        for i, w in enumerate(words[:-1]):
            nxt = words[i + 1]
            if same(norm(w), "llamo") or w in ("soy", "dicen") or (w == "nombre" and nxt == "es" and i + 2 < len(words)):
                cand = words[i + 2] if (w == "nombre" and nxt == "es") else nxt
                n = norm(cand)
                if n not in NOT_NAME and n not in COMMON and len(n) > 1:
                    return cand.title()
        if bare and 0 < len(words) <= 2 and not is_questionish(text):
            if all(norm(w) not in COMMON and norm(w) not in NOT_NAME and len(w) > 1 for w in words):
                return words[0].title()
        return None

    def _yesno(self, toks, ntext):
        fs = [fono(t) for t in toks]
        has_no = any(f in NO_F for f in fs) or "que no" in ntext
        has_yes = any(f in YES_F for f in fs)
        if has_no and (not has_yes or fs[0] in NO_F or "que no" in ntext):
            return "no"
        return "si" if has_yes else None

    def _clean(self, text, title):
        words = WORD_RE.findall(text.lower()) or re.findall(r"[a-z0-9]+", text.lower())
        while len(words) > 1 and norm(words[0]) in FILLER:
            words.pop(0)
        out = " ".join(words[:4])
        return out.title() if title else out

    def _options(self, nid):
        n = self.nodes[nid]
        opts = {}
        if "options_from" in n:
            for k, o in self.nodes[n["options_from"]]["options"].items():
                if not (n.get("exclude_visited") and o.get("topic") in self.visited):
                    opts[k] = o
        opts.update(n.get("options", {}))
        return opts

    def _res(self, node, key, o):
        r = {"key": key, "reply": o.get("reply"), "next": o.get("next") or node.get("next"), "slots": {},
             "label": o.get("label") or o.get("topic_name") or key}
        if node.get("slot"):
            r["slots"][node["slot"]] = o.get("slot_value", key)
        if o.get("topic"):
            r["topic"], r["topic_name"] = o["topic"], o.get("topic_name", o["topic"])
        return r

    def _free(self, node, text, ntext, toks, reply):
        if len(toks) >= 3 and toks[0] in STRICT_Q:
            return None
        val = self._clean(text, node.get("titlecase"))
        if ntext in NEG_FREE or toks[0] == "no" or not val:
            return {"key": "none", "reply": node.get("reply_none", ["¡Está bien!"]), "next": node.get("next"),
                    "slots": {}}
        r = {"key": "free", "reply": reply, "next": node.get("next"), "valor": val, "slots": {}}
        if node.get("slot"):
            r["slots"][node["slot"]] = val
        return r

    def _parse(self, nid, text, ntext, toks):
        node, kind = self.nodes[nid], self.nodes[nid].get("parse")
        if kind == "name":
            nm = self._extract_name(text, bare=True)
            if not nm:
                return None
            r = {"key": "name", "reply": node.get("reply"), "next": node.get("next"),
                 "slots": {"nombre": nm}, "valor": nm, "label": nm}
            if node.get("confirm_say"):
                r["confirm"] = node["confirm_say"]
            return r
        if kind == "yesno":
            a = self._yesno(toks, ntext)
            return self._res(node, a, node["options"][a]) if a else None
        if kind == "options":
            best, bh, bx = None, 0, False
            opts = self._options(nid)
            for generic in (False, True):               # las opciones "generic" solo si nada más coincide
                for key, o in opts.items():
                    if bool(o.get("generic")) != generic:
                        continue
                    hs = [hit(k, ntext, toks) for k in o.get("kw", [])]
                    h = sum(hs)
                    if h > bh:
                        best, bh, bx = key, h, 2 in hs
                if best:
                    break
            if best:
                r = self._res(node, best, opts[best])
                if not bx and not node.get("noconfirm"):          # solo coincidió "por sonido" → confirmar
                    r["confirm"] = self.flow["confirm"]
                return r
            if node.get("allow_free"):
                if self._kb(text, strong=True) or (len(toks) == 1 and self._yesno(toks, ntext) == "si"):
                    return None                       # ej. "gracias" o "sí" no son una respuesta, son charla
                return self._free(node, text, ntext, toks, node.get("reply_free"))
            return None
        if kind == "free":
            return self._free(node, text, ntext, toks, node.get("reply"))
        return None

    # ---------- avance ----------
    def _advance(self, nid, res):
        self.slots.update({k: v for k, v in res.get("slots", {}).items() if v})
        if res.get("topic"):
            self.visited.add(res["topic"])
            self.slots["tema"] = res["topic_name"]
        reply = ""
        if res.get("reply"):
            reply = self._fmt(self._pick(f"r:{nid}:{res['key']}", res["reply"]), {"valor": res.get("valor", "")})
        say = self._enter(res["next"]) if res.get("next") else ""
        return (reply + " " + say).strip()

    def _ask_confirm(self, nid, res):
        self.pending = {"nid": nid, "res": res}
        q = self._pick("confirm", res["confirm"])
        return self._fmt(q, {"label": res.get("label", "")})

    def _retry(self, nid):
        node = self.nodes[nid]
        n = self.retries.get(nid, 0) + 1
        self.retries[nid] = n
        if n <= 2:
            return (self._fmt(self._pick("retry", self.flow["retry"])) + " " + self._say(nid, reask=True)).strip()
        if node.get("parse") == "name":
            self.slots["nombre"] = "amigo"
        nxt = node.get("skip") or node.get("next") or "otro"
        return (self._fmt(self._pick("skip", self.flow["skipmsg"])) + " " + self._enter(nxt)).strip()

    def _idle(self, text, ntext, toks):
        greet = bool(set(toks) & GREET) and len(toks) <= 6
        chat = bool(set(toks) & CHAT_REQ)
        explicit = any(same(t, "llamo") for t in toks) and "me" in toks or "nombre" in toks and "es" in toks
        more = "otro" if self.visited else "gustos2"
        if self.node == "inicio" and (greet or chat or explicit):
            nm = self._extract_name(text, bare=False)
            if nm:
                n = self.nodes["saludo"]
                res = {"key": "name", "reply": n.get("reply"), "next": "gustos", "slots": {"nombre": nm},
                       "valor": nm, "label": nm}
                self.node = "saludo"
                if n.get("confirm_say"):
                    res["confirm"] = n["confirm_say"]
                    return self._ask_confirm("saludo", res), "flow"
                return self._advance("saludo", res), "flow"
            return self._enter("saludo"), "flow"
        if self.node == "libre" and (greet or chat):
            lead = self._fmt(self._pick("again", self.flow["again"]) if greet else "¡Claro!")
            return (lead + " " + self._enter(more)).strip(), "flow"
        ans = self._kb(text)
        return (ans, "kb") if ans else (None, None)

    # ---------- API principal ----------
    def handle(self, text):
        """Devuelve (respuesta, tipo). tipo: 'flow' | 'kb' | 'ignore' | None (None = no sé → fallback)."""
        now = time.time()
        if self.node != "inicio" and self.last_ts and now - self.last_ts > self.timeout:
            self.reset()
        self.last_ts = now
        if text.strip().lower() in ("[unk]", "unk"):          # el STT oyó hablar pero no entendió nada
            if self.in_flow() and not self.pending:
                return (self._fmt(self._pick("repeat", self.flow["repeat"])) + " "
                        + self._say(self.node, reask=True)).strip(), "flow"
            return None, "ignore"
        # ruido: quitar muletillas; si no queda nada útil, no responder
        toks = [t for t in norm(text).split() if t not in NOISE]
        if not toks or all(len(t) <= 1 for t in toks):
            return None, "ignore"
        ntext = " ".join(toks)

        if self.node != "inicio" and any_hit(FAREWELL, ntext, toks):
            r = self._fmt(self._pick("bye", self.flow["farewell"]))
            self.reset()
            return r, "flow"

        # confirmación pendiente ("¿Dijiste fútbol?")
        if self.pending:
            p, a = self.pending, self._yesno(toks, ntext)
            self.pending = None
            if a == "si":
                return self._advance(p["nid"], p["res"]), "flow"
            if a == "no":
                return (self._fmt(self._pick("confirm_no", self.flow["confirm_no"])) + " "
                        + self._say(p["nid"], reask=True)).strip(), "flow"
            # respondió otra cosa: se descarta la duda y se procesa normal

        if not self.in_flow():
            return self._idle(text, ntext, toks)

        # "¿qué?", "repite", "no entendí"
        if (len(toks) == 1 and toks[0] in REPEAT_ONE) or (len(toks) <= 4 and any_hit(REPEAT_PHRASES, ntext, toks)):
            return (self._fmt(self._pick("repeat", self.flow["repeat"])) + " " + self._say(self.node, reask=True)).strip(), "flow"

        if self.node == "saludo" and set(toks) <= GREET | {"yaren"}:
            return self._say("saludo"), "flow"

        q = is_questionish(text)
        if q:
            ans = self._kb(text, strong=True)
            if ans:
                return f"{ans} {self.reprompt()}".strip(), "kb"
        if any_hit(CHANGE_TOPIC, ntext, toks):
            return "Claro, cambiemos de tema. " + self._enter("otro" if self.visited else "gustos2"), "flow"

        res = self._parse(self.node, text, ntext, toks)
        if res:
            if res.get("confirm"):
                return self._ask_confirm(self.node, res), "flow"
            return self._advance(self.node, res), "flow"
        if q or self.nodes[self.node].get("parse") != "name":   # en la pregunta del nombre no responder charla suelta
            ans = self._kb(text)
            if ans:
                return f"{ans} {self.reprompt()}".strip(), "kb"
        if q:
            return None, None
        return self._retry(self.node), "flow"
