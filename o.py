import os

from datetime import datetime, timedelta
from filelock import FileLock

O_START: datetime = os.getenv("O_START", datetime.utcnow())
O_DEATH: timedelta = timedelta(seconds=int(os.getenv("O_DEATH", 10)))
MEMORY_PATH = "/tmp/o.memory.txt"
MEMORY_LOCK_PATH = "/tmp/o.memory.lock"
MEMORY_MAX_SIZE = 4096  # bytes
MAX_STEPS = 100
STEPS = 0


def timestamp(log: str) -> str:
    return f"{datetime.utcnow().timestamp()}{log}"


def heartbeat(name: str) -> (str, bool):
    global STEPS
    log = timestamp(f"{name} step {STEPS} of {MAX_STEPS}")
    STEPS += 1
    if STEPS > MAX_STEPS:
        log += timestamp(f"{name} max steps {MAX_STEPS} exceeded")
        return log, False
    if datetime.utcnow() - O_START > O_DEATH:
        log += timestamp(f"{name} death {O_DEATH} exceeded")
        return log, False
    return log, True


async def check_memory() -> str:
    log = ""
    if not os.path.exists(MEMORY_PATH):
        log += timestamp("📜 Memory file not found, creating ...")
        with open(MEMORY_PATH, "w") as f:
            f.write("")
    else:
        mem_size = os.path.getsize(MEMORY_PATH)
        # log += timestamp(f"💾 Current memory size is {mem_size} bytes")
        if mem_size > MEMORY_MAX_SIZE:
            log += timestamp(f"🗑️ Memory limit {MEMORY_MAX_SIZE} exceeded, truncating past")
            with FileLock(MEMORY_LOCK_PATH):
                with open(MEMORY_PATH, "r") as file:
                    lines = file.readlines()
                half_index = len(lines) // 2
                with open(MEMORY_PATH, "w") as file:
                    file.writelines(lines[half_index:])
    # print(log)
    return log


async def get_memory() -> (str, str):
    log = await check_memory()
    with FileLock(MEMORY_LOCK_PATH):
        with open(MEMORY_PATH, "r") as f:
            memraw = f.read()
    return log, f"""
Each line in the robot memory is the output of one node.
There are many nodes running asynchronously.
Each line in the robot memory contains UTC timestamps.
<robot_memory>
{memraw}
</robot_memory>
"""


async def add_memory(txt: str) -> str:
    log = await check_memory()
    with FileLock(MEMORY_LOCK_PATH):
        with open(MEMORY_PATH, "a") as f:
            f.write(timestamp(txt))
            f.write("\n")
    return log


async def remember(lines: list) -> str:
    log = await check_memory()
    to_add = [] * len(lines)
    with FileLock(MEMORY_LOCK_PATH):
        with open(MEMORY_PATH, "r") as f:
            for i, line in f.readlines():
                if i in lines:
                    to_add.append(line)
    log += f"🧠 remembered {len(to_add)} lines"
    with FileLock(MEMORY_LOCK_PATH):
        with open(MEMORY_PATH, "a") as f:
            f.write("\n".join(to_add))
    return log
    