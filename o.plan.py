import argparse
import asyncio

from o import heartbeat, add_memory, get_memory
from models import import_models

argparser = argparse.ArgumentParser()
argparser.add_argument("--model_api", type=str, default="test")
args = argparser.parse_args()

GOALS: dict = {
    "🔭": "visual exploration, looking before moving",
    "🤪": "performing and entertaining the human",
    "🤔": "making plans for the future",
}

async def loop(models: dict):
    log = "📝 plan started, my goals are:"
    log += "\n".join([f"{k} {v}" for k, v in GOALS.items()])
    while  True:
        log, is_alive = heartbeat('📝')
        if not is_alive:
            break
        (_, memstr) = await get_memory()
        tasks = []
        for name, opinion in GOALS.items():
            tasks.append(models["llm"](
                f"""
You prioritize robot goals based on a robot memory.
You are part of a collective with different goals.
Help vote on what the robot should do next based on the robot memory.
Reply with your opinion in a single line of text.
Keep it short.
Your emoji is {name}.
You represent the goal of {opinion}.
{memstr}
                """
            ))
        results = await asyncio.gather(*tasks)
        tasks = []
        for persona, result in zip(GOALS.keys(), results):
            _, reply = result
            tasks.append(add_memory(f"{persona} votes for [{reply}]"))
        await asyncio.gather(*tasks)

if __name__ == "__main__":
    asyncio.run(loop(
        models=import_models(args.model_api),
    ))
    print("o.plan.py: done")