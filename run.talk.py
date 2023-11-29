import argparse
import asyncio

from memory import check_alive, get_memory, add_memory
from models import import_models

argparser = argparse.ArgumentParser()
argparser.add_argument("--model_api", type=str, default="test")
args = argparser.parse_args()

MODELS: dict = import_models(args.model_api)

async def loop():
    speech = "hello world"
    while check_alive():
        memstr, tts_result = await asyncio.gather(
            get_memory(),
            MODELS["tts"](speech),
        )
        tts_log, _ = tts_result
        llm_result, _ = await asyncio.gather(
            MODELS["llm"](f"""
Pick a reply to speak out based on the robot log.
Reply to the human if anything is heard with stt.
Be short and minimal with your replies.
{memstr}
                """),
                add_memory(tts_log),
        )
        llm_log, speech = llm_result
        await add_memory(llm_log)


if __name__ == "__main__":
    print("🏁 talk born")
    asyncio.run(loop())
    print("🪦 talk dead")
