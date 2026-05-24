import re

class TextProcessor:
    @staticmethod
    def clean_text(text: str) -> list[str]:
        """
        Divide el texto en oraciones y limpia caracteres problemáticos para TTS.
        IMPORTANTE: primero dividir, DESPUÉS limpiar cada oración.
        """
        try:
            # 1. Normalizar saltos de línea y espacios extra
            text = re.sub(r'\s+', ' ', text.strip())

            # 2. Dividir por puntuación de fin de oración (conservándola)
            #    Separamos en: "Hola, ¿cómo estás?" → ["Hola, ¿cómo estás?"]
            raw_sentences = re.split(r'(?<=[.!?])\s+', text)

            results = []
            for sentence in raw_sentences:
                sentence = sentence.strip()
                if not sentence:
                    continue

                # 3. Limpiar SOLO caracteres que confunden al TTS,
                #    pero CONSERVAR letras, números, comas y signos de puntuación
                #    que el TTS sí sabe pronunciar
                cleaned = re.sub(r'[*\[\]{}¡¿]', '', sentence)
                cleaned = re.sub(r'\s+', ' ', cleaned).strip()

                if cleaned:
                    results.append(cleaned)

            return results if results else [text.strip()]

        except Exception:
            return [text.strip()]