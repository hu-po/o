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
Interpret this audio transcript: [{heard}].
It might have errors with clipping around the edges, or missing sections.
Try to reply with a restoration of the full transcript:
                """
                ))
            log += f"🎧 heard [{heard}]"
            log += f"🧠 thought [{llm_log}]"