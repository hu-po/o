
import os
import time

EMOJI = "🎯"
GOAL = os.getenv("O_GOAL", "wave at the humans")
GOAL_FREQ = float(os.getenv("O_GOAL_FREQ", 1.0))

async def loop(models: dict, robot: dict, utils: dict):
    while  True:
        _, is_alive = utils['heartbeat'](EMOJI)
        if not is_alive:
            break
        time.sleep(1.0 / GOAL_FREQ)
        await utils['add_memory'](f"{EMOJI} my goal is {GOAL}")