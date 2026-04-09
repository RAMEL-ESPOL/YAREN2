from utils import read_file
from assistant import Assistant

import sys
import os

import json

# Agrega la carpeta principal (afuera de audio_processing)
directorio_principal = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(directorio_principal)

CONFIG_PARAMS = read_file(directorio_principal + "/" +"config", "yaml")

def main():

    model = CONFIG_PARAMS["stt"]["model_size"]
    record_timeout = CONFIG_PARAMS["stt"]["recording_time"]
    phrase_timeout = CONFIG_PARAMS["stt"]["silence_break"]
    energy_threshold = CONFIG_PARAMS["stt"]["sensibility"]
    wake_word = CONFIG_PARAMS["assistant"]["wake_word"]

    comands = CONFIG_PARAMS["comands"]

    prompt = CONFIG_PARAMS["prompt"]

    basics_responses = {}

    try:
        with open(directorio_principal + "/" + "basics_responses.json", "r", encoding='utf-8') as f:
            basics_responses = json.load(f)
            basics_responses = {key.lower(): value for key, value in basics_responses.items()}
    except (FileNotFoundError, json.JSONDecodeError):
        print("Error al cargar respuestas básicas.")
        basics_responses = {}

    while True:
        va = Assistant(model, record_timeout, phrase_timeout, energy_threshold, wake_word,comands,prompt,basics_responses)
        print(va.listen_movement_comand())
    #va.write_transcript()

if __name__ == "__main__":
    main()

#en caso de que no coja el mic poner esto en bash: rm -rf ~/.config/pulse/ 