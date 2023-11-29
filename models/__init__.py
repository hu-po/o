import asyncio
import base64
import hashlib
import os
from filelock import FileLock
import time

from pydub import AudioSegment
from pydub.playback import play
from scipy.io.wavfile import write
import sounddevice as sd


IMAGE_PATH = "/tmp/o.image.jpeg"  # Image is constantly overwritten
IMAGE_LOCK_PATH = "/tmp/o.image.lock" # Lock prevents reading while writing
AUDIO_RECORD_TIME: int = 3  # Duration for audio recording
AUDIO_SAMPLE_RATE: int = 16000  # Sample rate for speedy audio recording
AUDIO_CHANNELS: int = 1  # mono
AUDIO_OUTPUT_PATH: str = "/tmp/o.audio.wav"  # recorded audio is constantly overwritten


def import_models(api: str) -> dict:
    if api == "gpt":
        from models.gpt import llm, vlm, tts, stt
        from models.gpt import LLM, VLM, TTS, STT

    elif api == "rep":
        from models.rep import llm, vlm, tts, stt
        from models.rep import LLM, VLM, TTS, STT

    else:
        from models.test import llm, vlm, tts, stt
        from models.test import LLM, VLM, TTS, STT
    print(f"@@@@@@@@@ Importing Models {api}")
    print(f"LLM 💬: {LLM}")
    print(f"VLM 👁️‍🗨️: {VLM}")
    print(f"TTS 🗣️: {TTS}")
    print(f"STT 👂: {STT}")
    print("@@@@@@@@@@@")

    def timed(f: callable):
        async def _(*args, **kwargs):
            _s = time.time()
            log, result = await f(*args, **kwargs)
            log += f", took {time.time() - _s:.2f}s⏱️"
            return log, result

        return _

    async def async_llm(prompt: str) -> [str, str]:
        try:
            reply = llm(prompt)
        except Exception as e:
            print("@@@@@@@@@@@ Exception in LLM")
            print(e)
            print("@@@@@@@@@@@")
            return "💬❌ error with llm", None
        return f"💬✅ llm reply [{reply}]", reply

    async def async_vlm(prompt: str) -> str:
        try:
            if not os.path.exists(IMAGE_PATH):
                await asyncio.sleep(1.0)
                return "👁️‍🗨️❌ no image found", None
            with FileLock(IMAGE_LOCK_PATH):
                with open(IMAGE_PATH, "rb") as f:
                    base64_image = base64.b64encode(f.read()).decode("utf-8")
            description = vlm(prompt, base64_image)
        except Exception as e:
            print("@@@@@@@@@@@ Exception in VLM")
            print(e)
            print("@@@@@@@@@@@")
            return "👁️‍🗨️❌ error with vlm", None
        return f"👁️‍🗨️✅ vlm saw [{description}]", description

    async def async_tts(text: str) -> str:
        try:
            file_name = f"/tmp/o.audio.{hashlib.sha256(text.encode()).hexdigest()[:10]}.mp3"
            if not os.path.exists(file_name):
                seg: AudioSegment = tts(text)
                seg.export(file_name, format="mp3")
            seg = AudioSegment.from_file(file_name, format="mp3")
            play(seg)
        except Exception as e:
            print("@@@@@@@@@@@ Exception in TTS")
            print(e)
            print("@@@@@@@@@@@")
            return "🗣️❌ error with tts", None
        return f"🗣️✅ tts said {text}", text

    async def async_stt() -> (str, str):
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
            print("@@@@@@@@@@@ Exception in STT")
            print(e)
            print("@@@@@@@@@@@")
            return "👂❌ error with stt", ""
        return f"👂✅ stt heard {transcript}", transcript

    return {
        "llm": timed(async_llm),
        "vlm": timed(async_vlm),
        "tts": timed(async_tts),
        "stt": timed(async_stt),
    }
