#!/usr/bin/env python3
"""Prueba rápida de audio: lista dispositivos, graba 3 s, muestra el nivel y lo reproduce.
uso: audio_check.py [mic] [parlante]     (índices o nombres; vacío = por defecto)"""
import sys
import numpy as np
import sounddevice as sd

dev = lambda i: None if len(sys.argv) <= i or sys.argv[i] == "" else (int(sys.argv[i]) if sys.argv[i].isdigit() else sys.argv[i])
print(sd.query_devices())
mic, spk = dev(1), dev(2)
print("\nHabla durante 3 segundos...")
rec = sd.rec(3 * 16000, samplerate=16000, channels=1, dtype="int16", device=mic)
sd.wait()
x = rec[:, 0].astype(np.float32)
print(f"nivel: pico={np.abs(x).max():.0f}/32767  rms={np.sqrt((x ** 2).mean()):.0f}")
print("-> pico < 3000: sube 'gain' o acerca el micrófono; pico = 32767: hay saturación, baja la ganancia.")
print("Reproduciendo lo grabado...")
sd.play(rec, 16000, device=spk)
sd.wait()
