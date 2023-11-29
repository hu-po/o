import argparse
import asyncio

from mem import check_alive, add_memory, get_memory
from models import import_models

argparser = argparse.ArgumentParser()
argparser.add_argument("--model_api", type=str, default="test")
args = argparser.parse_args()

MODELS: dict = import_models(args.model_api)

PERSONAS: dict = {
    "🔭": "visual exploration, looking before moving.",
    "📣": "performing and entertaining the human.",
    "📝": "making plans for the future.",
}

async def loop():
    memstr = await get_memory()
    while check_alive():
        results = await asyncio.gather(
            *[MODELS["llm"](
                f"""
Help vote on what the robot should do next based on the robot memory.
Reply with your opinion in a single line of text.
You are {name}, you prefer {opinion}
{memstr}
                """
            ) for name, opinion in PERSONAS.items()])
        for persona, result in zip(PERSONAS.keys(), results):
            _, reply = result
            await add_memory(f"{persona} votes for [{reply}]")


if __name__ == "__main__":
    print("🏁 plan born")
    try:
        asyncio.run(loop())
    except KeyboardInterrupt:
        print("🪦 plan interrupted by user")
        exit(0)
    print("🪦 plan dead")