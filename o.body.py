import argparse
import asyncio

from o import heartbeat, get_memory, add_memory
from models import import_models
from robots import import_robot

argparser = argparse.ArgumentParser()
argparser.add_argument("--model_api", type=str, default="test")
argparser.add_argument("--robot", type=str, default="test")
args = argparser.parse_args()


async def loop(models: dict, robot: dict):
    func, code = robot["DEFAULT_FUNC"], robot["DEFAULT_CODE"]
    while True:
        log, is_alive = heartbeat('🤸')
        if not is_alive:
            break
        (_, memstr) = await get_memory()
        (llm_log, reply), robot_log, _ = await asyncio.gather(
            models["llm"](
                f"""
Pick a function based on the robot log.
Always pick a function and provide any args required.
Here are the functions:
{robot['FUNCTIONS']}
{memstr}
Pick one of the functions and the args.
Here are some example outputs:
{robot['SUGGESTIONS']}
Your response should be a single line with the chosen function code and arguments.
                """
            ),
            robot["act"](func, code),
            add_memory(log),
        )
        try:
            func, code = reply.split(",")
        except ValueError:
            func, code = robot["DEFAULT_FUNC"], robot["DEFAULT_CODE"]
        log += llm_log
        log += robot_log


if __name__ == "__main__":
    asyncio.run(loop(
        models=import_models(args.model_api),
        robot=import_robot(args.robot),
    ))
    print("o.body.py: done")