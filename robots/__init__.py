import os
import subprocess
import time


def import_robot(robot: str) -> dict:
    if robot == "nex":
        from robots.nex import FUNCTIONS, SUGGESTIONS, DEFAULT_FUNC, DEFAULT_CODE
        from robots.nex import __file__ as _file

    elif robot == "iggi":
        from robots.iggi import FUNCTIONS, SUGGESTIONS, DEFAULT_FUNC, DEFAULT_CODE
        from robots.iggi import __file__ as _file

    else:
        from robots.test import FUNCTIONS, SUGGESTIONS, DEFAULT_FUNC, DEFAULT_CODE
        from robots.test import __file__ as _file

    async def async_act(func: str, code: str) -> str:
        _s = time.time()
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
            print("@@@@@@@@@@@ Exception in Robot")
            print(f"{e}, {stderr}")
            print("@@@@@@@@@@@")
            log = f"❌ failed on {func}({code})."
        return f"🤖{log[:-1]}, took {time.time() - _s:.2f}s⏱️"

    return {
        "act": async_act,
        "FUNCTIONS": FUNCTIONS,
        "SUGGESTIONS": SUGGESTIONS,
        "DEFAULT_FUNC": DEFAULT_FUNC,
        "DEFAULT_CODE": DEFAULT_CODE,
    }
