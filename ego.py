import asyncio
import time
from collections import deque

from datetime import datetime, timedelta
from filelock import Timeout, FileLock

START_TIME: datetime = datetime.now()
DEATH_TIME: timedelta = timedelta(minutes=4)  # How long the robot will live
MEMORY: int = 32  # How many characters worth of state to keep in memory
FORGET: int = 8  # How many characters worth of state to forget

MEMORY_PATH = "/tmp/o.📜"
MEMORY_LOCK_PATH = "/tmp/o.log.lock"

async def check_alive():

    # if len(state) >= MEMORY:
    #     for _ in range(FORGET):
    #         state.popleft()
    #     state.appendleft(f"{EMOJIS['forget']} memory erased")

    if datetime.now() - START_TIME > DEATH_TIME:
        return False
    return True


async def get_memory() -> str:
    with FileLock(MEMORY_LOCK_PATH):
        with open(MEMORY_PATH, "r") as f:
            log = f.read()
    return log

async def add_memory(log: str) -> None:
    with FileLock(MEMORY_LOCK_PATH):
        with open(MEMORY_PATH, "a") as f:
            f.write(log)
            f.write("\n")

async def forget() -> None:
    
    with FileLock(MEMORY_LOCK_PATH):
        with open(MEMORY_PATH, "w") as f:
            f.write("")
