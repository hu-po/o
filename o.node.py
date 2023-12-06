import os
import argparse
import asyncio
from datetime import datetime, timedelta
from filelock import FileLock

from models import import_models
from robots import import_robot
from nodes import import_node

argparser = argparse.ArgumentParser()
argparser.add_argument("--model_api", type=str, default="test")
argparser.add_argument("--robot", type=str, default="test")
argparser.add_argument("--node", type=str, default="test")

O_START: datetime = os.getenv("O_START", datetime.utcnow())
O_DEATH: timedelta = timedelta(seconds=int(os.getenv("O_DEATH", 10)))
O_STEPS: int = os.getenv("O_STEPS", 0)
O_MAX_STEPS: int = os.getenv("O_MAX_STEPS", 40)

MEMORY_PATH = "/tmp/o.memory.txt"
MEMORY_LOCK_PATH = "/tmp/o.memory.lock"
MEMORY_MAX_SIZE = 4096  # bytes

def timestamp(log: str) -> str:
    elapsed_time = datetime.utcnow() - O_START
    minutes, seconds = divmod(elapsed_time.total_seconds(), 60)
    return f"⏱️{int(minutes)}m:{seconds:.3f}s {log}"

def heartbeat(name: str) -> (str, bool):
    global O_STEPS
    O_STEPS += 1
    log = timestamp(f"{name} step {O_STEPS} of {O_MAX_STEPS}")
    if O_STEPS > O_MAX_STEPS:
        log += timestamp(f"{name} max steps {O_MAX_STEPS} exceeded")
        return log, False
    if datetime.utcnow() - O_START > O_DEATH:
        log += timestamp(f"{name} death {O_DEATH}s exceeded")
        return log, False
    return log, True

# TODO: Make this a specific file, keep handles to uuid named tmp files in memory
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
Each line in the robot memory contains a time since the start of the node.
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

if __name__ == "__main__":
    args = argparser.parse_args()
    models: dict = import_models(args.model_api)
    robot: dict = import_robot(args.robot)
    node: dict = import_node(args.node)
    utils: dict = {
        "heartbeat": heartbeat,
        "get_memory": get_memory,
        "add_memory": add_memory,
        "remember": remember,
    }
    print(f"starting node: {node['emoji']}")
    asyncio.run(node['loop'](models, robot, utils))
    print(f"node {node['emoji']} completed")