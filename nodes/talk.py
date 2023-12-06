import asyncio

EMOJI = "📣"

async def loop(models: dict, robot: dict, utils: dict):
    log = "📣 talk started (chatty, talks about everything)"
    speak = "hello world"
    heard = ""
    while  True:
        log, is_alive = utils['heartbeat'](EMOJI)
        if not is_alive:
            break
        (_, memstr), (tts_log, _) = await asyncio.gather(
            utils['get_memory'](),
            models["tts"](speak),
        )
        tasks = [
            # Say "o" outloud so we know when listening
            models["tts"]("o"),
            models["stt"](),
            utils['add_memory'](log),
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
