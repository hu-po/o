import os

from datetime import datetime, timedelta
from filelock import FileLock

START_TIME: datetime = datetime.now()
DEATH_TIME: timedelta = timedelta(minutes=4)  # How long the robot will live

MEMORY_PATH = "/tmp/o.📜"
MEMORY_LOCK_PATH = "/tmp/o.memory.🔒"
MEMORY_MAX_SIZE = 1024 * 1024  # 1MB


def check_alive():
    if datetime.now() - START_TIME > DEATH_TIME:
        return False
    return True


def timestamp(log: str) -> str:
    return f"{datetime.utcnow().timestamp()} {log}"


async def get_memory() -> str:
    with FileLock(MEMORY_LOCK_PATH):
        mem_size = os.path.getsize(MEMORY_PATH)
        print(f"💾 Reading memory, current size {mem_size} bytes")
        if mem_size > MEMORY_MAX_SIZE:
            print(f"🗑️ Memory limit {MEMORY_MAX_SIZE} exceeded, truncating")
            with open(MEMORY_PATH, "r") as file:
                lines = file.readlines()
            half_index = len(lines) // 2
            with open(MEMORY_PATH, "w") as file:
                file.writelines(lines[half_index:])
        with open(MEMORY_PATH, "r") as f:
            log = f.read()
    return timestamp(f"Here is the robot memory:\n<memory>\n{log}\n</memory>")


async def add_memory(log: str) -> None:
    with FileLock(MEMORY_LOCK_PATH):
        with open(MEMORY_PATH, "a") as f:
            f.write(timestamp(log))
            f.write("\n")


async def forget() -> None:
    with FileLock(MEMORY_LOCK_PATH):
        with open(MEMORY_PATH, "w") as f:
            f.write("")
