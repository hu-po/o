import asyncio

EMOJI = "🧐"

async def loop(models: dict, robot: dict, utils: dict):
    vlm_log = "🧐 look started"
    while  True:
        log, is_alive = utils['heartbeat'](EMOJI)
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
        utils['add_memory'](vlm_log))