import base64
import time
import hashlib
import io
from pydub import AudioSegment

EMOJIS = {
    "robot": "🤖",
    "state": "📄",
    "fail": "❌",
    "success": "✅",
    "born": "🐣",
    "died": "🪦",
    "vlm": "👁️‍🗨️",
    "llm": "💬",
    "tts": "🗣️",
    "stt": "👂",
    "time": "⏱️",
    "move": "🦿",
    "look": "📷",
    "perform": "🦾",
}
IMAGE_PATH = "/tmp/image.jpg"  # Image is constantly overwritten


def timeit(f):
    def _(*args, **kwargs):
        start_time = time.time()
        print(f"----------- {f.__name__}")
        for key, value in kwargs.items():
            print(f"\t{key}={value}")
        result = f(*args, **kwargs)
        end_time = time.time()
        print(
            f"----------- {f.__name__} took {EMOJIS['time']}{end_time - start_time:.2f}s"
        )
        return result

    return _


@timeit
def encode_image(image_path: str = IMAGE_PATH):
    with open(image_path, "rb") as f:
        base64_image = base64.b64encode(f.read()).decode("utf-8")
    return base64_image


def make_tmp_audio_path(text: str):
    return f"/tmp/tmp{hashlib.sha256(text.encode()).hexdigest()[:10]}.mp3"


def bytes_to_audio(bytes: bytes, path: str):
    byte_stream = io.BytesIO(bytes)
    seg = AudioSegment.from_file(byte_stream, format="mp3")
    seg.export(path, format="mp3")
