EMOJIS = {
    "brain": "🧠",
    "robot": "🤖",
    "state": "📄",
    "save": "💾",
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
    "plan": "🤔",
}
IMAGE_PATH = "/tmp/image.jpg"  # Image is constantly overwritten

def import_models(api: str = "test") -> dict:
    if api == "test":
        LLM, VLM, TTS, STT = ["test"] * 4

        def llm(x):
            return "test llm reply,"

        def vlm(x, y):
            return "test vlm reply"

        def tts(x):
            return None

        def stt(x):
            return "test tts reply"
    elif api == "gpt":
        from models_gpt import llm, vlm, tts, stt
        from models_gpt import LLM, VLM, TTS, STT

    elif api == "rep":
        from models_rep import llm, vlm, tts, stt
        from models_rep import LLM, VLM, TTS, STT

    else:
        raise Exception(f"Unknown model api {api}")
    print(f"LLM {EMOJIS['llm']}: {LLM}")
    print(f"VLM {EMOJIS['vlm']}: {VLM}")
    print(f"TTS {EMOJIS['tts']}: {TTS}")
    print(f"STT {EMOJIS['stt']}: {STT}")
    return {'llm': llm, 'vlm': vlm, 'tts': tts, 'stt': stt}