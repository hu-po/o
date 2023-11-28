import argparse
import asyncio
from collections import deque


from ego import is_alive, get_log, write_log
from models import import_models
from robots import import_robot

argparser = argparse.ArgumentParser()
argparser.add_argument("--model_api", type=str, default="test")
argparser.add_argument("--robot", type=str, default="test")
args = argparser.parse_args()

MODELS: dict = import_models(args.model_api)
ROBOT: dict = import_robot(args.robot)

EMOJIS = {
    "brain": "🧠",
    "robot": "🤖",
    "state": "📄",
    "save": "💾",
    "fail": "❌",
    "success": "✅",
    "born": "🐣",
    "forget": "🗑️",
    "vlm": "👁️‍🗨️",
    "llm": "💬",
    "tts": "🗣️",
    "stt": "👂",
    "time": "⏱️",
    "move": "🦿",
    "look": "📷",
    "perform": "🦾",
    "dead": "🪦",
    "poem": "📜",
    "plan": "🤔",
}



async def sense() -> list:
    return await asyncio.gather(_vlm(), _stt(), _tts("observing"))


async def act(func: str, code: str, speech: str) -> list:
    return await asyncio.gather(_act(func, code), _tts(speech))


async def plan(state: str) -> [str, str, str]:
    results = await asyncio.gather(
        *[
            _llm(
                f"""
Pick a function based on the robot log. Always pick a function and provide any args required. Here are the functions:
{ROBOT['functions']}
Here is the robot log
<robotlog>
{state}
</robotlog>
Pick one of the functions and the args. Here are some example outputs:
{ROBOT['examples']}
Your response should be a single line with the chosen function code and arguments.
"""
            ),
            _llm(
                f"""
Summarize the robot log in a couple clever words, be brief but precise
Here is the robot log
<robotlog>
{state}
</robotlog>
"""
            ),
            _tts("deciding"),
        ]
    )
    func, code = results[0][1].split(",")
    speech = results[1][1]
    return func, code, speech


while is_alive():

    for s in asyncio.run(MODELS['llm']
        state.append(s)
    state_str = "\n".join([str(item) for item in state])
    print(f"*********** {EMOJIS['state']} age {datetime.now() - BIRTHDAY}")
    print(state_str)
    print(f"*********** {EMOJIS['state']}")
    func, code, speech = asyncio.run(plan(state_str))
    state.append(f"{EMOJIS['llm']} choosing function {func} {code}")
    print(f"___________{EMOJIS['plan']}")
    print(speech)
    print(func, code)
    print(f"___________{EMOJIS['plan']}")
    for s in asyncio.run(act(func, code, speech)):
        state.append(s)
state.append(f"{EMOJIS['dead']} robot is dead, lived for {LIFESPAN}")
poem = MODELS["llm"](
    f"""
Write a short eulogy poem for a robot. Here is the robot log:
<robotlog>
{state}
</robotlog>
"""
)
print(f"~~~~~~~~~~~ {EMOJIS['poem']}")
print(poem)
print(f"~~~~~~~~~~~ {EMOJIS['poem']}")
MODELS["tts"](poem)
