import argparse
import asyncio

from ego import check_alive, add_memory, get_memory
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
    memory = await get_memory()
    while check_alive():
        results = asyncio.gather(
            *[MODELS["llm"](
                f"""
Help vote on what the robot should do next based on the robot log.
Reply with your opinion in a single line of text.
You are {name}, you prefer {opinion}
Here is the robot log:
<robotlog>
{memory}
</robotlog>
                """
            ) for name, opinion in PERSONAS.items()])
        for persona, result in zip(PERSONAS.keys(), results):
            llm_log, reply = result
            await add_memory(f"{persona} votes for [{reply}]")


if __name__ == "__main__":
    print("🏁 plan born")
    asyncio.run(loop())
    print("🪦 plan dead")
