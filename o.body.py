import argparse
import asyncio

from mem import check_alive, get_memory, add_memory
from models import import_models
from robots import import_robot

argparser = argparse.ArgumentParser()
argparser.add_argument("--model_api", type=str, default="test")
argparser.add_argument("--robot", type=str, default="test")
args = argparser.parse_args()

MODELS: dict = import_models(args.model_api)
ROBOT: dict = import_robot(args.robot)


async def loop():
    log = "🤸 body started"
    func, code = ROBOT["DEFAULT_FUNC"], ROBOT["DEFAULT_CODE"]
    while check_alive('🤸'):
        (_, memstr) = await get_memory()
        (llm_log, reply), robot_log, _ = await asyncio.gather(
            MODELS["llm"](
                f"""
Pick a function based on the robot log.
Always pick a function and provide any args required.
Here are the functions:
{ROBOT['FUNCTIONS']}
{memstr}
Pick one of the functions and the args.
Here are some example outputs:
{ROBOT['SUGGESTIONS']}
Your response should be a single line with the chosen function code and arguments.
                """
            ),
            ROBOT["act"](func, code),
            add_memory(log),
        )
        func, code = reply.split(",")
        log = f"{llm_log}{robot_log}"


if __name__ == "__main__":
    print("🏁 body born")
    try:
        asyncio.run(loop())
    except KeyboardInterrupt:
        print("🪦 body interrupted by user")
        exit(0)
    print("🪦 body dead")