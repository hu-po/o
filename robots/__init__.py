import os
import subprocess
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

def import_robot(robot: str = "test") -> dict:
    if robot == "nex":
        from robots.ainex import FUNCTIONS, SUGGESTIONS
        from robots.ainex import __file__ as _file

    elif robot == "igigi":
        from robots.igigi import FUNCTIONS, SUGGESTIONS
        from robots.igigi import __file__ as _file

    else:
        from robots.test import FUNCTIONS, SUGGESTIONS
        from robots.test import __file__ as _file

    def timed(f: callable):
        async def _(*args, **kwargs):
            
            log, result = await f(*args, **kwargs)
            log += f", took {time.time() - _s:.2f}s⏱️"
            return log, result

        return _
        
    async def async_act(func: str, code: str) -> str:
        _s = time.time()
        _log = ""
        _path = os.path.join(os.path.dirname(os.path.realpath(__file__)), _file)
        try:
            proc = subprocess.Popen(
                ["python3", _path, func, code],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            log, stderr = proc.communicate()
        except Exception as e:
            # print(f"{e}, {stderr}")
            return f"{EMOJIS['robot']}{EMOJIS['fail']} robot failed on {func} {code}"
        
        log += f", took {time.time() - _s:.2f}s⏱️"
        return log


    return {
        "functions": FUNCTIONS,
        "examples": SUGGESTIONS,
        "act": async_act,
    }