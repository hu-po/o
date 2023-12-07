import os
import time

LLM = str(os.getenv("O_LLM", "test llm"))
VLM = str(os.getenv("O_VLM", "test vlm"))
TTS = str(os.getenv("O_TTS", "test tts"))
STT = str(os.getenv("O_STT", "test stt"))


def llm(x):
    # print(f"🪲 test llm received {x}")
    time.sleep(1)
    return "MOVE,FORWARD"


def vlm(x, y):
    # print(f"🪲 test vlm received {x} and {y}")
    time.sleep(1)
    return "I see darkness"


def tts(x):
    # print(f"🪲 test tts received {x}")
    time.sleep(1)
    return None


def stt(x):
    # print(f"🪲 test stt received {x}")
    time.sleep(1)
    return "I hear nothing"
