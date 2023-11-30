import argparse
import asyncio

from mem import heartbeat, add_memory
from models import import_models
from robots import import_robot

argparser = argparse.ArgumentParser()
argparser.add_argument("--model_api", type=str, default="test")
argparser.add_argument("--robot", type=str, default="test")
args = argparser.parse_args()

async def loop(models: dict, robot: dict):
    vlm_log = "🧐 look started"
    while  True:
        log, is_alive = heartbeat('🧐')
        if not is_alive:
            break
        (vlm_log, _), _ = await asyncio.gather(models["vlm"](
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
        add_memory(vlm_log))


if __name__ == "__main__":
    asyncio.run(loop(
        models=import_models(args.model_api),
        robot=import_robot(args.robot),
    ))
    print("o.look.py: done")