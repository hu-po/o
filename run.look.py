import argparse
import asyncio

from memory import check_alive, add_memory
from models import import_models

argparser = argparse.ArgumentParser()
argparser.add_argument("--model_api", type=str, default="test")
args = argparser.parse_args()

MODELS: dict = import_models(args.model_api)


async def loop():
    while check_alive():
        vlm_result = await MODELS["vlm"](
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
        )
        vlm_log, _ = vlm_result
        await add_memory(vlm_log)


if __name__ == "__main__":
    print("🏁 look born")
    asyncio.run(loop())
    print("🪦 look dead")
