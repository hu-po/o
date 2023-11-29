import os

from datetime import datetime, timedelta
from filelock import FileLock

STARTON: datetime = datetime.utcnow()
DEATH_TIME: timedelta = timedelta(seconds=int(os.getenv('DEATH_TIME', 6)))  # How long the robot will live
MEMORY_PATH = "/tmp/o.memory.txt"
MEMORY_LOCK_PATH = "/tmp/o.memory.lock"
MEMORY_MAX_SIZE = 1024  # 1KB


def check_alive():
    if datetime.now() - STARTON > DEATH_TIME:
        return False
    return True


def timestamp(log: str) -> str:
    return f"{datetime.utcnow().timestamp()} {log}"


def _check_memory_exists():
    if not os.path.exists(MEMORY_PATH):
        print("📜 Memory file not found, creating")
        with open(MEMORY_PATH, "w") as f:
            f.write("")


async def get_memory() -> str:
    _check_memory_exists()
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
    _check_memory_exists()
    with FileLock(MEMORY_LOCK_PATH):
        with open(MEMORY_PATH, "a") as f:
            f.write(timestamp(log))
            f.write("\n")
