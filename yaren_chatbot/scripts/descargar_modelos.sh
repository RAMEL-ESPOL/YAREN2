#!/usr/bin/env bash
# Descarga el modelo Vosk (STT) y una voz Piper femenina (TTS) en ~/yaren_models
set -e
D="${1:-$HOME/yaren_models}"; mkdir -p "$D"; cd "$D"
if [ ! -d vosk-model-small-es-0.42 ]; then
  wget -q --show-progress https://alphacephei.com/vosk/models/vosk-model-small-es-0.42.zip
  unzip -q vosk-model-small-es-0.42.zip && rm vosk-model-small-es-0.42.zip
fi

# Voz femenina: es_MX-claude-high
V=es_MX-claude-high
BASE=https://huggingface.co/rhasspy/piper-voices/resolve/main/es/es_MX/claude/high
if [ ! -f "$V.onnx" ]; then
  wget -q --show-progress "$BASE/$V.onnx"
  wget -q --show-progress "$BASE/$V.onnx.json"
fi
echo "Listo en $D"; ls "$D"