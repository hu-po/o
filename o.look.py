import argparse
import asyncio

from mem import check_alive, add_memory
from models import import_models

argparser = argparse.ArgumentParser()
argparser.add_argument("--model_api", type=str, default="test")
args = argparser.parse_args()

MODELS: dict = import_models(args.model_api)


async def loop():
    log = "🧐 look started"
    while check_alive():
        (vlm_log, _), mem_log = await asyncio.gather(MODELS["vlm"](
            """
Describe the scene, objects, and characters
You are a robot vision module
You are small and only 20 centimeters off the ground
Focus on the most important things
If there are humans mention them and their relative position
Do not mention the image
Directly describe the scene
Be concise
Do not use punctuation
Your response will be read out by the robot speech module
Your reponse should not contain any special characters
"""
        ),
        add_memory(log))
        log = f"{vlm_log}\n{mem_log}"


if __name__ == "__main__":
    print("🏁 look born")
    try:
        asyncio.run(loop())
    except KeyboardInterrupt:
        print("🪦 look interrupted by user")
        exit(0)
    print("🪦 look dead")
