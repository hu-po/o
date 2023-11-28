import argparse
import asyncio

from ego import is_alive, get_memory, add_memory
from models import import_models
from robots import import_robot

argparser = argparse.ArgumentParser()
argparser.add_argument("--model_api", type=str, default="test")
args = argparser.parse_args()

MODELS: dict = import_models(args.model_api)

async def loop():
    state = await get_memory()
    speech = "hello world"
    while is_alive():
        llm_result, tts_result = asyncio.gather(
            *[
                MODELS["llm"](
                    f"""
    Pick a reply to speak out based on the robot log.
    Be short and minimal.
    Reply to the human if anything is heard.
    Here is the robot log:
    <robotlog>
    {state}
    </robotlog>
    """
                ),
                MODELS["tts"](speech),
            ]
        )
        llm_log, speech = llm_result
        tts_log, _ = tts_result
        await add_memory(llm_log)
        await add_memory(tts_log)


if __name__ == "__main__":
    print("🏁 talk born")
    asyncio.run(loop())
    print("🪦 talk dead")
