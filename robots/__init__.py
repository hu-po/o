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
        from robots.nex import FUNCTIONS, SUGGESTIONS

        # robot commands are run in a subprocess
        ROBOT_FILENAME: str = "nex.py"
    elif robot == "test":
        FUNCTIONS = """
    MOVE(direction:str)
    direction must be one of ["FORWARD", "BACKWARD", "LEFT", "RIGHT"]
    """
        SUGGESTIONS = """
    MOVE,FORWARD
    MOVE,LEFT
    """
        ROBOT_FILENAME: str = "oop.py"
    else:
        raise Exception(f"Unknown robot {robot}")
    return {
        "functions": FUNCTIONS,
        "examples": SUGGESTIONS,
        "filename": ROBOT_FILENAME,
    }