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
        (llm_log, speak), (stt_log, heard), _ = await asyncio.gather(
            MODELS["llm"](f"""
Pick a reply to speak out based on the robot log.
Reply to the human if anything is heard with stt.
You most recently heard [{heard}].
Be short and minimal with your replies.
{memstr}
                """),
            MODELS["stt"](),
            add_memory(log),
        )
        log = f"{tts_log}{llm_log}{stt_log}"


if __name__ == "__main__":
    print("🏁 talk born")
    try:
        asyncio.run(loop())
    except KeyboardInterrupt:
        print("🪦 talk interrupted by user")
        exit(0)
    print("🪦 talk dead")
