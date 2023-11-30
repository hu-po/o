import argparse
import asyncio

from mem import heartbeat, get_memory, add_memory
from models import import_models

argparser = argparse.ArgumentParser()
argparser.add_argument("--model_api", type=str, default="test")
args = argparser.parse_args()

models: dict = import_models(args.model_api)


async def loop(models: dict):
    log = "📣 talk started"
    speak = "hello world"
    heard = ""
    while  True:
        log, is_alive = heartbeat('📣')
        if not is_alive:
            break
        (_, memstr), (tts_log, _) = await asyncio.gather(
            get_memory(),
            models["tts"](speak),
        )
        tasks = [
            # Say "o" outloud so we know when listening
            models["tts"]("o"),
            models["stt"](),
            add_memory(log),
        ]
        if heard:
            tasks.append(
                models["llm"](
                    f"""
Reply to the human.
The human said: [{heard}].
Be short, laconic, and witty in your reply.
The human might mention the robot memory:
{memstr}
                """
                )
            )
        else:
            tasks.append(
                models["llm"](
                    f"""
Describe what the robot sees and any future plans.
Be short, laconic, and witty in your reply.
{memstr}
                """
                )
            )
        (tts_log, _), (stt_log, heard), _, (llm_log, speak) = await asyncio.gather(
            *tasks
        )
        log = f"{tts_log}{llm_log}{stt_log}"


if __name__ == "__main__":
    asyncio.run(
        loop(
            models=import_models(args.model_api),
        )
    )
    print("o.talk.py: done")
