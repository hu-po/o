import time

EMOJIS = {
    "brain": "🧠",
    "robot": "🤖",
    "state": "📄",
    "fail": "❌",
    "success": "✅",
    "born": "🐣",
    "forget": "🗑️",
    "vlm": "👁️‍🗨️",
    "llm": "💬",
    "tts": "🗣️",
    "stt": "👂",
    "time": "⏱️",
    "move": "🦿",
    "look": "📷",
    "perform": "🦾",
    "dead": "🪦",
    "poem": "📜",
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