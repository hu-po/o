import argparse
import asyncio

from util import EMOJIS, import_models, import_robot

argparser = argparse.ArgumentParser()
argparser.add_argument("--model_api", type=str, default="test")
argparser.add_argument("--robot", type=str, default="test")
args = argparser.parse_args()

MODELS: dict = import_models(args.model_api)
ROBOT: dict = import_robot(args.robot)

PERSONAS: list = [
    "You are the robot vision module. You prioritize visual exploration. Your vote is for looking at objects before moving.",
    "You are the robot movement module. You prioritize moving to new locations. Your vote is for moving before looking.",
    "You are the robot language module. You prioritize talking to humans. Your vote is for talking before moving.",
    "You are the robot action module. You prioritize doing things. Your vote is for doing things before talking.",
    "You are the robot planning module. You prioritize planning. Your vote is for planning before doing.",
    "You are the robot critique module. You prioritize critiquing. Your vote is for critiquing before planning.",
    "You are the robot compliment module. You prioritize complimenting. Your vote is for complimenting before critiquing.",
]

CRITIQUE: str = """
Give a critique on the previous action choice.
"""
COMPLIMENT: str = """
Give compliment on the previous action choice.
"""

async def _llm(prompt: str) -> [str, str]:
    try:
        reply = MODELS['llm'](prompt)
    except Exception as e:
        # print(e)
        return (
            f"{EMOJIS['llm']}❌ could not think, {e.__class__.__name__}"
        )
    return f"{EMOJIS['llm']}✅ {reply}", reply

async def moe(state: str) -> [str, str, str]:
    results = await asyncio.gather(
        *[
            _llm(
                f"""
{persona}
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
            ) for persona in PERSONAS
        ]
    )
    func, code = results[0][1].split(",")
    speech = results[1][1]
    return func, code, speech
