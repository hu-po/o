import asyncio

EMOJI = "🧪"

async def loop(models: dict, robot: dict, utils: dict):
    while  True:
        log, is_alive = utils['heartbeat'](EMOJI)
        if not is_alive:
            break
        (_, memstr) = await utils['get_memory']()
        (llm_log, _), (vlm_log, _), (tts_log, _), (stt_log, _), robot_log = await asyncio.gather(
            models["llm"]("mock prompt"),
            models["vlm"]("mock prompt"),
            models["tts"]("mock text"),
            models["stt"](),
            robot["act"]("MOVE", "FORWARD"),
        )
        log = f"{log} {llm_log} {vlm_log} {tts_log} {stt_log} {robot_log}"
        await utils['add_memory'](log)
