import os
import subprocess

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

def import_robot(robot: str = "test") -> dict:
    if robot == "nex":
        from robots.ainex import FUNCTIONS, SUGGESTIONS
        from robots.ainex import __file__ as ROBOT_FILENAME

    elif robot == "test":
        from robots.test import FUNCTIONS, SUGGESTIONS
        from robots.test import __file__ as ROBOT_FILENAME

    elif robot == "igigi":
        from robots.igigi import FUNCTIONS, SUGGESTIONS
        from robots.igigi import __file__ as ROBOT_FILENAME

    else:
        raise Exception(f"Unknown robot {robot}")
    
    async def async_act(func: str, code: str) -> str:
        _path = os.path.join(os.path.dirname(os.path.realpath(__file__)), ROBOT_FILENAME)
        try:
            proc = subprocess.Popen(
                ["python3", _path, func, code],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            stdout, stderr = proc.communicate()
        except Exception as e:
            # print(f"{e}, {stderr}")
            return f"{EMOJIS['robot']}{EMOJIS['fail']} robot failed on {func} {code}"
        return stdout


    return {
        "functions": FUNCTIONS,
        "examples": SUGGESTIONS,
        "act": async_act,
    }