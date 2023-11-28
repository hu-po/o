import time

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
        from gpt import llm, vlm, tts, stt
        from gpt import LLM, VLM, TTS, STT

    elif api == "rep":
        from rep import llm, vlm, tts, stt
        from rep import LLM, VLM, TTS, STT

    else:
        raise Exception(f"Unknown model api {api}")
    print(f"LLM {EMOJIS['llm']}: {LLM}")
    print(f"VLM {EMOJIS['vlm']}: {VLM}")
    print(f"TTS {EMOJIS['tts']}: {TTS}")
    print(f"STT {EMOJIS['stt']}: {STT}")
    return {'llm': llm, 'vlm': vlm, 'tts': tts, 'stt': stt}
    

def import_robot(robot: str = "test") -> dict:
    if robot == "nex":
        from nex import ROBOT_FUNC_PROMPT, ROBOT_EXAMPLE_PROMPT

        # robot commands are run in a subprocess
        ROBOT_FILENAME: str = "nex.py"
    elif robot == "test":
        ROBOT_FUNC_PROMPT = """
    MOVE(direction:str)
    direction must be one of ["FORWARD", "BACKWARD", "LEFT", "RIGHT"]
    """
        ROBOT_EXAMPLE_PROMPT = """
    MOVE,FORWARD
    MOVE,LEFT
    """
        ROBOT_FILENAME: str = "oop.py"
    else:
        raise Exception(f"Unknown robot {robot}")
    return {
        "functions": ROBOT_FUNC_PROMPT,
        "examples": ROBOT_EXAMPLE_PROMPT,
        "filename": ROBOT_FILENAME,
    }