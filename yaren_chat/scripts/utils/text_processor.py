import re

class TextProcessor:
    @staticmethod
    def clean_text(text):
        """Clean text and split into sentences, similar to C++ implementation"""
        try:
            punct_pattern = r'[!¡?¿*,.:;()\[\]{}]'
            cleaned = re.sub(punct_pattern, ' ', text)
            
            cleaned = re.sub(r'\s+', ' ', cleaned)
            
            sentences = []
            sentence_pattern = r'([.!?])\s+'
            parts = re.split(sentence_pattern, cleaned)
            
            sentence = ""
            for i, part in enumerate(parts):
                sentence += part
                if re.match(r'[.!?]', part):
                    sentences.append(sentence.strip())
                    sentence = ""
            
            if sentence.strip():
                sentences.append(sentence.strip())
            
            return sentences if sentences else [cleaned.strip()]
        except Exception as e:
            return [text.strip()]