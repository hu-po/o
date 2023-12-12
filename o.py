import os
import argparse
import asyncio
import random
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

MEM_ID = int(os.getenv("O_MEM_ID", 0))
MEM_MAX_SIZE = int(os.getenv("O_MEM_MAX_SIZE", 4096))
MEM_MAX_NUM = int(os.getenv("O_MEM_MAX_NUM", 8))


def make_memory_paths(id: int = MEM_ID) -> str:
    return f"/tmp/o.memory.{id}.txt", f"/tmp/o.memory.{id}.lock"


def make_memory_prompt(memraw: str) -> str:
    return """
Each line in the robot memory is the output of one node.
There are many nodes running asynchronously.
Each line in the robot memory contains a time since the start of the node.
<robot_memory>
{memraw}
</robot_memory>
"""


def timestamp(log: str) -> str:
    elapsed_time = datetime.now() - START
    minutes, seconds = divmod(elapsed_time.total_seconds(), 60)
    return f"⏱️   time [{int(minutes)}m:{seconds:.2f}]{log}"


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


def make_memory(id: int) -> (str, int):
    mem_path, mem_lock_path = make_memory_paths(id)
    if not os.path.exists(mem_path):
        log = f"💾  created new memory {id} of {MEM_MAX_NUM}"
    else:
        log = f"💾  wiped reused memory {id} of {MEM_MAX_NUM}"
    with FileLock(mem_lock_path):
        with open(mem_path, "w") as f:
            f.write("")
    return log, id


async def select_memory(id: int = MEM_ID) -> (str, int):
    mem_path, _ = make_memory_paths(id)
    if not os.path.exists(mem_path):
        log, _ = make_memory(id)
        return log, id
    else:
        mem_size = os.path.getsize(mem_path)
        if mem_size < MEM_MAX_SIZE:
            log = f"💾  found memory {id} of {MEM_MAX_NUM} size {mem_size} bytes"
            return log, id
        else:
            log += f" max size {MEM_MAX_SIZE} bytes exceeded"
            # TODO: evolutionary algorithm
            new_id = random.randint(0, MEM_MAX_NUM)
            log += f" using new memory {new_id} of {MEM_MAX_NUM}"
            make_memory(new_id)
            return log, new_id


async def get_memory() -> (str, str):
    log, id = await select_memory()
    mem_path, mem_lock_path = make_memory_paths(id)
    with FileLock(mem_lock_path):
        with open(mem_path, "r") as f:
            memraw = f.read()
    return timestamp(log), memraw


async def add_memory(txt: str) -> str:
    log, id = await select_memory()
    mem_path, mem_lock_path = make_memory_paths(id)
    with FileLock(mem_lock_path):
        with open(mem_path, "a") as f:
            f.write(timestamp(txt))
    return timestamp(log)


if __name__ == "__main__":
    args = argparser.parse_args()
    node: dict = import_node(args.node)
    robot: dict = import_robot(args.robot, node["emoji"])
    models: dict = import_models(args.model_api, node["emoji"])
    utils: dict = {
        "heartbeat": heartbeat,
        "get_memory": get_memory,
        "add_memory": add_memory,
    }
    asyncio.run(node["loop"](models, robot, utils))
    print(f"🖥️ {node['emoji']} ✅ node finished")
