import argparse
import asyncio

from mem import check_alive, get_memory, add_memory
from models import import_models

argparser = argparse.ArgumentParser()
argparser.add_argument("--model_api", type=str, default="test")
args = argparser.parse_args()

MODELS: dict = import_models(args.model_api)

async def loop():
    log = "📣 talk started"
    speak = "hello world"
    heard = ""
    while check_alive('📣'):
        (_, memstr), (tts_log, _) = await asyncio.gather(
            get_memory(),
            MODELS["tts"](speak),
        )
        tasks = [
            # Say "o" outloud so we know when listening
            MODELS["tts"]("o"),
            MODELS["stt"](),
            add_memory(log),
        ]
        if heard:
            tasks.append(MODELS["llm"](
                f"""
Reply to the human.
The human said: [{heard}].
Be short, laconic, and witty in your reply.
The human might mention the robot memory:
{memstr}
                """))
        else:
            tasks.append(MODELS["llm"](
                f"""
Summarize the more recent (bottom) robot memories.
Be short, laconic, and witty in your reply.
{memstr}
                """))   
        (tts_log, _), (stt_log, heard), _, (llm_log, speak)= await asyncio.gather(*tasks)
        log = f"{tts_log}{llm_log}{stt_log}"


if __name__ == "__main__":
    print("🏁 talk born")
    try:
        asyncio.run(loop())
    except KeyboardInterrupt:
        print("🪦 talk interrupted by user")
        exit(0)
    print("🪦 talk dead")
