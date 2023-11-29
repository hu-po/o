import os

from datetime import datetime, timedelta
from filelock import FileLock

START: datetime = datetime.utcnow()
DEATH: timedelta = timedelta(seconds=int(os.getenv('DEATH', 6)))
MEMORY_PATH = "/tmp/o.memory.txt"
MEMORY_LOCK_PATH = "/tmp/o.memory.lock"
MEMORY_MAX_SIZE = 4096 # bytes


def check_alive():
    if datetime.utcnow() - START > DEATH:
        return False
    return True


def timestamp(log: str) -> str:
    return f"{datetime.utcnow().timestamp()} {log}"


async def check_memory() -> str:
    log = ""
    if not os.path.exists(MEMORY_PATH):
        # log += timestamp("📜 Memory file not found, creating ...")
        with open(MEMORY_PATH, "w") as f:
            f.write("")
    else:
        mem_size = os.path.getsize(MEMORY_PATH)
        # log += timestamp(f"💾 Current memory size is {mem_size} bytes")
        if mem_size > MEMORY_MAX_SIZE:
            # log += timestamp(f"🗑️ Memory limit {MEMORY_MAX_SIZE} exceeded, truncating past")
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
    return log, f"Here is the robot memory:\n<memory>\n{memraw}\n</memory"

async def add_memory(log: str) -> (str, str):
    log = await check_memory()
    with FileLock(MEMORY_LOCK_PATH):
        with open(MEMORY_PATH, "a") as f:
            f.write(timestamp(log))
            f.write("\n")
    return log, None
