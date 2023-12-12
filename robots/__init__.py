import os
import subprocess
import time


def import_robot(robot: str, node: str) -> dict:
    if robot == "nex":
        from robots.nex import DEFAULT_FUNC, DEFAULT_CODE, DESCRIPTION
        from robots.nex import __file__ as _file

    elif robot == "igi":
        from robots.igi import DEFAULT_FUNC, DEFAULT_CODE, DESCRIPTION
        from robots.igi import __file__ as _file

    else:
        from robots.test import DEFAULT_FUNC, DEFAULT_CODE, DESCRIPTION
        from robots.test import __file__ as _file
    print(f"🖥️ {node} using robot {robot}")

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
            print(f"\t🖥️❌ exception in robot: {e}, {stderr}")
            log = f"❌ failed on {func}({code})."
        return f"🤖{log[:-1]}, took {time.time() - _s:.2f}s⏱️"

    return {
        "act": async_act,
        "DESCRIPTION": DESCRIPTION,
        "DEFAULT_FUNC": DEFAULT_FUNC,
        "DEFAULT_CODE": DEFAULT_CODE,
    }