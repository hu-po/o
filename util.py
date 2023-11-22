import base64
import time

EMOJIS = {
    "brain": "🧠",
    "robot": "🤖",
    "state": "📄",
    "fail": "❌",
    "success": "✅",
    "born": "🐣",
    "dead": "🪦",
    "forget": "🗑️",
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
            f"----------- {f.__name__} took {end_time - start_time:.2f}s{EMOJIS['time']}"
        )
        return result

    return _


def encode_image(image_path: str = IMAGE_PATH):
    with open(image_path, "rb") as f:
        base64_image = base64.b64encode(f.read()).decode("utf-8")
    return base64_image
