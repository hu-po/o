import argparse
import asyncio

from util import EMOJIS

argparser = argparse.ArgumentParser()
argparser.add_argument("--model_api", type=str, default="rep")
argparser.add_argument("--robot", type=str, default="test")
args = argparser.parse_args()

if args.model_api == "gpt":
    from gpt import llm, vlm, tts, stt
    from gpt import LLM, VLM, TTS, STT
elif args.model_api == "rep":
    from rep import llm, vlm, tts, stt
    from rep import LLM, VLM, TTS, STT
elif args.model_api == "test":
    LLM, VLM, TTS, STT = ["test"] * 4

    def llm(x):
        return "test llm reply,"

    def vlm(x, y):
        return "test vlm reply"

    def tts(x):
        return "test tts reply"

    def stt(x):
        return None

if args.robot == "nex":
    from nex import ROBOT_FUNC_PROMPT, ROBOT_EXAMPLE_PROMPT

    # robot commands are run in a subprocess
    ROBOT_FILENAME: str = "nex.py"
elif args.robot == "test":
    ROBOT_FUNC_PROMPT = """
MOVE(direction:str)
direction must be one of ["FORWARD", "BACKWARD", "LEFT", "RIGHT"]
"""
    ROBOT_EXAMPLE_PROMPT = """
MOVE,FORWARD
MOVE,LEFT
"""
    ROBOT_FILENAME: str = "oop.py"

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
        reply = llm(prompt)
    except Exception as e:
        # print(e)
        return (
            f"{EMOJIS['llm']}{EMOJIS['fail']} could not think, {e.__class__.__name__}"
        )
    return f"{EMOJIS['llm']}{EMOJIS['success']} {reply}", reply

async def moe(state: str) -> [str, str, str]:
    results = await asyncio.gather(
        *[
            _llm(
                f"""

{ROBOT_FUNC_PROMPT}
Here is the robot log
<robotlog>
{state}
</robotlog>
Pick one of the functions and the args. Here are some example outputs:
{ROBOT_EXAMPLE_PROMPT}
Your response should be a single line with the chosen function code and arguments.
"""
            ),
            _llm(
                f"""
Pick a function based on the robot log. Always pick a function and provide any args required. Here are the functions:
{ROBOT_FUNC_PROMPT}
Here is the robot log
<robotlog>
{state}
</robotlog>
Pick one of the functions and the args. Here are some example outputs:
{ROBOT_EXAMPLE_PROMPT}
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
        ]
    )
    func, code = results[0][1].split(",")
    speech = results[1][1]
    return func, code, speech
