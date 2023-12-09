import asyncio

EMOJI = "🎧"

async def loop(models: dict, robot: dict, utils: dict):
    heard = ""
    while  True:
        log, is_alive = utils['heartbeat'](EMOJI)
        if not is_alive:
            break
        _, (stt_log, heard) = await asyncio.gather(
            utils['add_memory'](log),
            models["stt"](),
        )
        if heard:
            log += f"🎧 heard [{heard}]"
            _, (stt_log, heard), (llm_log, _) = await asyncio.gather(
                utils['add_memory'](log),
                models["stt"](),
                models["llm"](
                    f"""
Interpret this transcript: [{heard}].
Does it contain any direct commands intended for a robot?
Condense the most important concepts into a single sentence.
                """
                ))
            log += f"🎧 heard [{heard}]"
            log += f"🧠 thought [{llm_log}]"