import asyncio
import os

EMOJI = "🧐"
LOOK_PROMPT = os.environ.get("O_LOOK_PROMPT", """
Describe the scene, objects, and characters
Focus on the most important things
If there are humans mention them and their relative position
Do not mention the image
Directly describe the scene
Be concise
Do not use punctuation
Your response will be read out by the robot speech module
Your reponse should not contain any special characters
""")

async def loop(models: dict, robot: dict, utils: dict):
    vlm_log = "🧐 look started"
    while  True:
        log, is_alive = utils['heartbeat'](EMOJI)
        if not is_alive:
            break
        (vlm_log, _), _ = await asyncio.gather(models["vlm"](LOOK_PROMPT),
        utils['add_memory'](vlm_log))