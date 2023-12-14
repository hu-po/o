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

IMAGE_PATH = os.getenv('O_IMAGE_PATH', "/tmp/o.image.jpeg")
IMAGE_LOCK_PATH = os.getenv('O_IMAGE_LOCK_PATH', "/tmp/o.image.lock")
IMAGE_FREQ = int(os.getenv('O_IMAGE_FREQ', 1))
AUDIO_RECORD_TIME = int(os.getenv('O_AUDIO_RECORD_TIME', 3))
AUDIO_SAMPLE_RATE = int(os.getenv('O_AUDIO_SAMPLE_RATE', 16000))
AUDIO_CHANNELS = int(os.getenv('O_AUDIO_CHANNELS', 1))
AUDIO_OUTPUT_PATH = os.getenv('O_AUDIO_OUTPUT_PATH', "/tmp/o.audio.wav")
AUDIO_LOCK_PATH = os.getenv('O_SPEAKING_LOCK_PATH', "/tmp/o.audio.lock")

def import_models(api: str, node: str) -> dict:
    api = "test"
    from models.test import llm, vlm, tts, stt
    from models.test import LLM, VLM, TTS, STT
    if api == "gpt":
        from models.gpt import llm, vlm, tts, stt
        from models.gpt import LLM, VLM, TTS, STT

    elif api == "rep":
        from models.rep import llm, vlm, tts, stt
        from models.rep import LLM, VLM, TTS, STT

    elif api == "gem":
        from models.gem import llm, vlm
        from models.gem import LLM, VLM

    print(f"🖥️ {node} using model {api}")
    # print(f"   LLM 💬: {LLM}")
    # print(f"   VLM 👁️‍🗨️: {VLM}")
    # print(f"   TTS 🗣️: {TTS}")
    # print(f"   STT 👂: {STT}")

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
            print(f"\t{node}🖥️❌ exception in LLM: {e}")
            return f"{node}💬❌ error with llm", None
        return f"{node}💬✅ llm reply [{reply}]", reply

    async def async_vlm(prompt: str) -> str:
        try:
            if not os.path.exists(IMAGE_PATH):
                await asyncio.sleep(IMAGE_FREQ)
                return f"{node}👁️‍🗨️❌ no image found", None
            with FileLock(IMAGE_LOCK_PATH):
                with open(IMAGE_PATH, "rb") as f:
                    base64_image = base64.b64encode(f.read()).decode("utf-8")
            description = vlm(prompt, base64_image)
        except Exception as e:
            print(f"\t{node}🖥️❌ exception in VLM: {e}")
            return f"{node}👁️‍🗨️❌ error with vlm", None
        return f"{node}👁️‍🗨️✅ vlm saw [{description}]", description

    async def async_tts(text: str) -> str:
        try:
            file_name = (
                f"/tmp/o.audio.{hashlib.sha256(text.encode()).hexdigest()[:10]}.mp3"
            )
            if not os.path.exists(file_name):
                seg: AudioSegment = tts(text)
                if not seg:
                    return "🗣️✅ tts in test mode", None
                seg.export(file_name, format="mp3")
            seg = AudioSegment.from_file(file_name, format="mp3")
            with FileLock(AUDIO_LOCK_PATH):
                play(seg)
        except Exception as e:
            print(f"\t{node}🖥️❌ exception in TTS: {e}")
            return f"{node}🗣️❌ error with tts", None
        return f"{node}🗣️✅ tts said [{text}]", text

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
            print(f"\t{node}🖥️❌ exception in STT: {e}")
            return f"{node}👂❌ error with stt", ""
        return f"{node}👂✅ stt heard [{transcript}]", transcript

    return {
        "llm": timed(async_llm),
        "vlm": timed(async_vlm),
        "tts": timed(async_tts),
        "stt": timed(async_stt),
    }
