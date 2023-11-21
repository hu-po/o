import base64
import time

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
IMAGE_PATH = "/tmp/image.jpg" # Image is constantly overwritten


def timeit(f):
    def _(*args, **kwargs):
        start_time = time.time()
        print(f"----------- {f.__name__}")
        for key, value in kwargs.items():
            print(f"\t{key}={value}")
        result = f(*args, **kwargs)
        end_time = time.time()
        print(f"----------- {f.__name__} took {EMOJIS['time']}{end_time - start_time:.2f}s")
        return result

    return _

@timeit
def encode_image(image_path: str = IMAGE_PATH):
    with open(image_path, "rb") as f:
        base64_image = base64.b64encode(f).decode("utf-8")
    return base64_image