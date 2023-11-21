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
