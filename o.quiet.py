import argparse
import asyncio

from o import heartbeat, get_memory, add_memory
from models import import_models

argparser = argparse.ArgumentParser()
argparser.add_argument("--model_api", type=str, default="test")
args = argparser.parse_args()

models: dict = import_models(args.model_api)


async def loop(models: dict):
    speak = "hey there"
    heard = ""
    while  True:
        log, is_alive = heartbeat('🙊')
        if not is_alive:
            break
        (_, memstr), _, (stt_log, heard) = await asyncio.gather(
            get_memory(),
            add_memory(log)
            models["stt"](),
        )
        log = stt_log
        if heard:
            (llm_log, speak) = await models["llm"](
                    f"""
Reply to the human.
The human said: [{heard}].
Be short, laconic, and witty in your reply.
The human might mention the robot memory:
{memstr}
                """
                )
            log += llm_log
            (tts_log, _) = await models["tts"](speak)
            log += tts_log
        else:
            log += "🙊 heard nothing"
        await add_memory(log)


if __name__ == "__main__":
    asyncio.run(
        loop(
            models=import_models(args.model_api),
        )
    )
    print("o.talk.py: done")
