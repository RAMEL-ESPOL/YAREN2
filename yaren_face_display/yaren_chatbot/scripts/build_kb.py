#!/usr/bin/env python3
# uso: build_kb.py datos.csv resource/kb.db [--reset]
# CSV con columnas: question,answer,category  (variantes de pregunta separadas por |)
import csv, os, sys
from kb import KB

args = [a for a in sys.argv[1:] if not a.startswith("--")]
src, dst = args[0], args[1]
if "--reset" in sys.argv and os.path.exists(dst):
    os.remove(dst)
kb = KB(dst)
n = 0
with open(src, newline="", encoding="utf-8") as f:
    for row in csv.DictReader(f):
        for q in row["question"].split("|"):
            if q.strip():
                kb.add(q.strip(), row["answer"].strip(), row.get("category", ""))
                n += 1
kb.db.commit()
print(f"{n} preguntas cargadas en {dst}")
