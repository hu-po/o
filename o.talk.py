import argparse
import asyncio

from mem import check_alive, get_memory, add_memory
from models import import_models

argparser = argparse.ArgumentParser()
argparser.add_argument("--model_api", type=str, default="test")
args = argparser.parse_args()

MODELS: dict = import_models(args.model_api)

async def loop():
    speech = "hello world"
    while check_alive():
        (_, memstr), (tts_log, _) = await asyncio.gather(
            get_memory(),
            MODELS["tts"](speech),
        )
        (llm_log, speech), _ = await asyncio.gather(
            MODELS["llm"](f"""
Pick a reply to speak out based on the robot log.
Reply to the human if anything is heard with stt.
Be short and minimal with your replies.
{memstr}
                """),
                add_memory(tts_log),
        )
        await add_memory(llm_log)


if __name__ == "__main__":
    print("🏁 talk born")
    try:
        asyncio.run(loop())
    except KeyboardInterrupt:
        print("🪦 talk interrupted by user")
        exit(0)
    print("🪦 talk dead")
