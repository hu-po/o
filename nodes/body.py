import asyncio

EMOJI = "🤸"

async def loop(models: dict, robot: dict, utils: dict):
    func, code = robot["DEFAULT_FUNC"], robot["DEFAULT_CODE"]
    while True:
        log, is_alive = utils['heartbeat'](EMOJI)
        if not is_alive:
            break
        (_, memstr) = await utils['get_memory']()
        (llm_log, reply), robot_log, _ = await asyncio.gather(
            models["llm"](f"{memstr}{robot['DESCRIPTION']}"),
            robot["act"](func, code),
            utils['add_memory'](log),
        )
        log = llm_log + robot_log
        try:
            func, code = reply.split(",")
        except ValueError:
            func, code = robot["DEFAULT_FUNC"], robot["DEFAULT_CODE"]
        await utils['add_memory'](log)