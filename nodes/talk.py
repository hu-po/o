import asyncio
import os

EMOJI = "📢"
TALK_PROMPT = os.environ.get("O_TALK_PROMPT", """
Respond to the human.
Be short, laconic, and witty in your reply.
""")

async def loop(models: dict, robot: dict, utils: dict):
    speak = "hey there"
    heard = ""
    while  True:
        log, is_alive = utils['heartbeat'](EMOJI)
        if not is_alive:
            break
        (mem_log, memstr), _, (stt_log, heard) = await asyncio.gather(
            utils['get_memory'](),
            utils['add_memory'](log),
            models["stt"](),
        )
        log = mem_log + stt_log
        if heard:
            (llm_log, speak) = await models["llm"](f"""
{TALK_PROMPT}
You just heard [{heard}]
{memstr}
""")
            log += llm_log
            (tts_log, _) = await models["tts"](speak)
            log += tts_log
        else:
            log += f"{EMOJI} heard nothing"
        await utils['add_memory'](log)