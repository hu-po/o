import asyncio

EMOJI = "🙊"

async def loop(models: dict, robot: dict, utils: dict):
    speak = "hey there"
    heard = ""
    while  True:
        log, is_alive = utils['heartbeat'](EMOJI)
        if not is_alive:
            break
        (_, memstr), _, (stt_log, heard) = await asyncio.gather(
            utils['get_memory'](),
            utils['add_memory'](log),
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
        await utils['add_memory'](log)