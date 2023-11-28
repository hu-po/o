import base64
from datetime import datetime, timedelta
import hashlib
import os

from pydub import AudioSegment
from pydub.playback import play
from scipy.io.wavfile import write
import sounddevice as sd

EMOJIS = {
    "vlm": "👁️‍🗨️",
    "llm": "💬",
    "tts": "🗣️",
    "stt": "👂",
    "fail": "❌",
    "success": "✅",

    "brain": "🧠",
    "robot": "🤖",
    "state": "📄",
    "save": "💾",
    "born": "🐣",
    "forget": "🗑️",
    "time": "⏱️",
    "move": "🦿",
    "look": "📷",
    "perform": "🦾",
    "dead": "🪦",
    "poem": "📜",
    "plan": "🤔",
}
IMAGE_PATH = "/tmp/image.jpg"  # Image is constantly overwritten
AUDIO_RECORD_TIME: int = 3  # Duration for audio recording
AUDIO_SAMPLE_RATE: int = 16000  # Sample rate for speedy audio recording
AUDIO_CHANNELS: int = 1  # mono
AUDIO_OUTPUT_PATH: str = "/tmp/audio.wav"  # recorded audio is constantly overwritten
IMAGE_OUTPUT_PATH: str = "/tmp/image.jpg"  # captured image is constantly overwritten

def import_models(api: str = "test") -> dict:
    if api == "test":
        from test import llm, vlm, tts, stt
        from test import LLM, VLM, TTS, STT

    elif api == "gpt":
        from gpt import llm, vlm, tts, stt
        from gpt import LLM, VLM, TTS, STT

    elif api == "rep":
        from rep import llm, vlm, tts, stt
        from rep import LLM, VLM, TTS, STT

    else:
        raise Exception(f"Unknown model api {api}")
    print(f"@@@@@@@@@ Importing Models {api}")
    print(f"LLM {EMOJIS['llm']}: {LLM}")
    print(f"VLM {EMOJIS['vlm']}: {VLM}")
    print(f"TTS {EMOJIS['tts']}: {TTS}")
    print(f"STT {EMOJIS['stt']}: {STT}")
    print("@@@@@@@@@@@")

    async def async_llm(prompt: str) -> [str, str]:
        try:
            reply = llm(prompt)
        except Exception as e:
            # print(e)
            return (
                f"{EMOJIS['llm']}{EMOJIS['fail']} could not think, {e.__class__.__name__}"
            )
        return f"{EMOJIS['llm']}{EMOJIS['success']} {reply}", reply

    async def async_vlm(prompt: str) -> str:
        try:
            with open(IMAGE_PATH, "rb") as f:
                base64_image = base64.b64encode(f.read()).decode("utf-8")
                description = vlm(prompt, base64_image)
        except Exception as e:
            # print(e)
            return f"{EMOJIS['vlm']}{EMOJIS['fail']} could not see, {e.__class__.__name__}"
        return f"{EMOJIS['vlm']}{EMOJIS['success']} saw {description}"


    async def async_tts(text: str) -> str:
        try:
            file_name = f"/tmp/tmp{hashlib.sha256(text.encode()).hexdigest()[:10]}.mp3"
            if not os.path.exists(file_name):
                seg: AudioSegment = tts(text)
                seg.export(file_name, format="mp3")
            seg = AudioSegment.from_file(file_name, format="mp3")
            play(seg)
        except Exception as e:
            print(e)
            return (
                f"{EMOJIS['tts']}{EMOJIS['fail']} could not speak, {e.__class__.__name__}"
            )
        return f"{EMOJIS['tts']}{EMOJIS['success']} said '{text}'"


    async def async_stt() -> str:
        try:
            audio_data = sd.rec(
                int(AUDIO_RECORD_TIME * AUDIO_SAMPLE_RATE),
                samplerate=AUDIO_SAMPLE_RATE,
                channels=AUDIO_CHANNELS,
            )
            sd.wait()  # Wait until recording is finished
            write(AUDIO_OUTPUT_PATH, AUDIO_SAMPLE_RATE, audio_data)
            transcript = stt(AUDIO_OUTPUT_PATH)
        except Exception as e:
            # print(e)
            return f"{EMOJIS['stt']}{EMOJIS['fail']} could not hear, {e.__class__.__name__}"
        return f"{EMOJIS['stt']}{EMOJIS['success']} heard {transcript}"


    return {'llm': async_llm, 'vlm': async_vlm, 'tts': async_tts, 'stt': async_stt}

