import argparse
import asyncio

from o import heartbeat, add_memory, get_memory, remember
from models import import_models

argparser = argparse.ArgumentParser()
argparser.add_argument("--model_api", type=str, default="test")
args = argparser.parse_args()

GOALS: list = [
    "visual exploration, looking before moving",
    "performing and entertaining the human",
    "making plans for the future to ensure efficient exploration",
    "avoid repetitive actions",
]
NUM_EDITS: int = 3


async def loop(models: dict):
    log = f"🧠🎯 goals are: {GOALS}"
    while True:
        log, is_alive = heartbeat("🧠")
        if not is_alive:
            break
        (_, memstr), _ = await asyncio.gather(get_memory(), add_memory(log))
        results = await asyncio.gather(
            *[
                models["llm"](
                    f"""
You are playing a board game, everyone is controlling a robot together.
The robot will choose an action based the robot memory.
Summarize the robot memory in a few lines.
These lines will be added as your contribution to the robot memory.
You will pick {NUM_EDITS} lines to add.
Other players will also be adding lines.
Choose wisely such that your goals are achieved.
Your goals are: [{goal}]
Return a comma separated list of line numbers to add and remove.
The line numbers start at 0 and represent the row index.
{memstr}
                """
                )
                for goal in GOALS
            ]
        )
        try:
            lines = []
            for _, reply in results:
                for x in reply.split(","):
                    lines.append(int(x))
            log = await remember(lines)
        except Exception as e:
            print("@@@@@@@@@@@ Exception in plan")
            print(e)
            print("@@@@@@@@@@@")
            log = "🗳️❌ planning failed to remember"
            
        


if __name__ == "__main__":
    asyncio.run(
        loop(
            models=import_models(args.model_api),
        )
    )
    print("o.plan.py: done")