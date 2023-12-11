import os
import argparse
import asyncio
from datetime import datetime, timedelta
from filelock import FileLock

from models import import_models
from robots import import_robot
from nodes import import_node

argparser = argparse.ArgumentParser()
argparser.add_argument("--node", type=str, default="test")
argparser.add_argument("--model_api", type=str, default="test")
argparser.add_argument("--robot", type=str, default="test")

START = datetime.now()
DEATH = timedelta(seconds=int(os.getenv("O_DEATH", 3)))
STEPS = int(os.getenv("O_STEPS", 0))
MAX_STEPS: int = int(os.getenv("O_MAX_STEPS", 1))
MEM_PATH = str(os.getenv("O_MEM_PATH", "/tmp/o.memory.txt"))
MEM_LOCK_PATH = str(os.getenv( "O_MEM_LOCK_PATH", "/tmp/o.memory.lock"))
MEM_MAX_SIZE = int(os.getenv("O_MEM_MAX_SIZE", 4096))

def timestamp(log: str) -> str:
    elapsed_time = datetime.now() - START
    minutes, seconds = divmod(elapsed_time.total_seconds(), 60)
    return f"⏱️  {int(minutes)}m:{seconds:.2f}s {log}"

def heartbeat(name: str) -> (str, bool):
    global STEPS
    STEPS += 1
    log = timestamp(f"{name} step {STEPS} of {MAX_STEPS}")
    if STEPS > MAX_STEPS:
        log += timestamp(f"{name} max steps {MAX_STEPS} exceeded")
        return log, False
    if datetime.now() - START > DEATH:
        log += timestamp(f"{name} death {DEATH}s exceeded")
        return log, False
    return log, True

# TODO: Make this a specific file, keep handles to uuid named tmp files in memory
async def check_memory() -> str:
    log = ""
    if not os.path.exists(MEM_PATH):
        log += timestamp("📜 Memory file not found, creating ...")
        with open(MEM_PATH, "w") as f:
            f.write("")
    else:
        mem_size = os.path.getsize(MEM_PATH)
        # log += timestamp(f"💾 Current memory size is {mem_size} bytes")
        if mem_size > MEM_MAX_SIZE:
            log += timestamp(f"🗑️ Memory limit {MEM_MAX_SIZE} exceeded, truncating past")
            with FileLock(MEM_LOCK_PATH):
                with open(MEM_PATH, "r") as file:
                    lines = file.readlines()
                half_index = len(lines) // 2
                with open(MEM_PATH, "w") as file:
                    file.writelines(lines[half_index:])
    # print(log)
    return log


async def get_memory() -> (str, str):
    log = await check_memory()
    with FileLock(MEM_LOCK_PATH):
        with open(MEM_PATH, "r") as f:
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
    with FileLock(MEM_LOCK_PATH):
        with open(MEM_PATH, "a") as f:
            f.write(timestamp(txt))
    return log


async def remember(lines: list) -> str:
    log = await check_memory()
    to_add = [] * len(lines)
    with FileLock(MEM_LOCK_PATH):
        with open(MEM_PATH, "r") as f:
            for i, line in f.readlines():
                if i in lines:
                    to_add.append(line)
    log += f"🧠 remembered {len(to_add)} lines"
    with FileLock(MEM_LOCK_PATH):
        with open(MEM_PATH, "a") as f:
            f.write("\n".join(to_add))
    return log

if __name__ == "__main__":
    args = argparser.parse_args()
    node: dict = import_node(args.node)
    robot: dict = import_robot(args.robot, node['emoji'])
    models: dict = import_models(args.model_api, node['emoji'])
    utils: dict = {
        "heartbeat": heartbeat,
        "get_memory": get_memory,
        "add_memory": add_memory,
        "remember": remember,
    }
    print(f"      {node['emoji']}  starting node")
    asyncio.run(node['loop'](models, robot, utils))
    print(f"      {node['emoji']}✅  node finished")