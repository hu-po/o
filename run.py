import argparse
import asyncio
import base64
from collections import deque
from datetime import datetime, timedelta
import hashlib
import os
import subprocess

from pydub import AudioSegment
from pydub.playback import play
from scipy.io.wavfile import write
import sounddevice as sd

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
    from nex import ROBOT_FUNC_PROMPT

    # robot commands are run in a subprocess
    ROBOT_FILENAME: str = "nex.py"
elif args.robot == "test":
    ROBOT_FUNC_PROMPT = """
MOVE(direction:str)
direction must be one of ["FORWARD", "BACKWARD", "LEFT", "RIGHT"]
LOOK(direction:str)
direction must be one of ["UP", "DOWN", "LEFT", "RIGHT"]
"""
    ROBOT_FILENAME: str = "oop.py"

BIRTHDAY: datetime = datetime.now()
LIFESPAN: timedelta = timedelta(minutes=4)  # How long the robot will live
MEMORY: int = 32  # How many characters worth of state to keep in memory
FORGET: int = 8  # How many characters worth of state to forget

BLIND: bool = True  # Do not use see (VLM)
MUTE: bool = False  # Do not speak (TTS)
DEAF: bool = False  # Do not listen for audio input
LAME: bool = False  # Do not use robot commands (move, look, perform)

AUDIO_RECORD_TIME: int = 3  # Duration for audio recording
AUDIO_SAMPLE_RATE: int = 16000  # Sample rate for speedy audio recording
AUDIO_CHANNELS: int = 1  # mono
AUDIO_OUTPUT_PATH: str = "/tmp/audio.wav"  # recorded audio is constantly overwritten
IMAGE_OUTPUT_PATH: str = "/tmp/image.jpg"  # captured image is constantly overwritten


async def _llm(prompt: str) -> [str, str]:
    try:
        reply = llm(prompt)
    except Exception as e:
        # print(e)
        return (
            f"{EMOJIS['llm']}{EMOJIS['fail']} could not think, {e.__class__.__name__}"
        )
    return f"{EMOJIS['llm']}{EMOJIS['success']} {reply}", reply


async def _vlm() -> str:
    if BLIND:
        return f"{EMOJIS['vlm']}{EMOJIS['fail']} could not see, robot is blind"
    try:
        prompt = """
Describe the scene, objects, and characters
You are a robot vision module
You are small and only 20 centimeters off the ground
Focus on the most important things
If there are humans mention them and their relative position
Do not mention the image
Directly describe the scene
Be concise
Do not use punctuation
Your response will be read out by the robot speech module
Your reponse should not contain any special characters
"""
        with open(IMAGE_OUTPUT_PATH, "rb") as f:
            base64_image = base64.b64encode(f.read()).decode("utf-8")
            description = vlm(prompt, base64_image)
    except Exception as e:
        # print(e)
        return f"{EMOJIS['vlm']}{EMOJIS['fail']} could not see, {e.__class__.__name__}"
    return f"{EMOJIS['vlm']}{EMOJIS['success']} saw {description}"


async def _tts(text: str) -> str:
    if MUTE:
        return f"{EMOJIS['tts']}{EMOJIS['fail']} could not speak, robot is on mute"
    try:
        file_name = f"/tmp/tmp{hashlib.sha256(text.encode()).hexdigest()[:10]}.mp3"
        if not os.path.exists(file_name):
            seg = tts(text)
            seg.export(file_name, format="mp3")
        seg = AudioSegment.from_file(file_name, format="mp3")
        play(seg)
    except Exception as e:
        # print(e)
        return (
            f"{EMOJIS['tts']}{EMOJIS['fail']} could not speak, {e.__class__.__name__}"
        )
    return f"{EMOJIS['tts']}{EMOJIS['success']} said '{text}'"


async def _stt() -> str:
    if DEAF:
        return f"{EMOJIS['stt']}{EMOJIS['fail']} could not hear, robot is deaf"
    try:
        audio_data = sd.rec(
            int(AUDIO_RECORD_TIME * AUDIO_SAMPLE_RATE),
            samplerate=AUDIO_SAMPLE_RATE,
            channels=AUDIO_CHANNELS,
        )
        sd.wait()  # Wait until recording is finished
        write(AUDIO_OUTPUT_PATH, AUDIO_SAMPLE_RATE, audio_data)
        transcript = stt(AUDIO_OUTPUT_PATH)
    except Exception as e:
        # print(e)
        return f"{EMOJIS['stt']}{EMOJIS['fail']} could not hear, {e.__class__.__name__}"
    return f"{EMOJIS['stt']}{EMOJIS['success']} heard {transcript}"


async def _act(func: str, code: str) -> str:
    if LAME:
        return f"{EMOJIS['robot']}{EMOJIS['fail']} cannot act, robot is lame"
    _path = os.path.join(os.path.dirname(os.path.realpath(__file__)), ROBOT_FILENAME)
    try:
        proc = subprocess.Popen(
            ["python3", _path, func, code],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        stdout, stderr = proc.communicate()
    except Exception as e:
        # print(f"{e}, {stderr}")
        return f"{EMOJIS['robot']}{EMOJIS['fail']} robot failed on {func} {code}"
    return stdout


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
{ROBOT_FUNC_PROMPT}
Here is the robot log
<robotlog>
{state}
</robotlog>
Pick one of the functions and the args. Here are some example outputs:
PLAY,greet
LOOK,up
MOVE,forward
Your response should be a single line with the chosen function code and arguments.
"""
            ),
            _llm(
                f"""
Summarize the robot log in a couple clever words, be brief but precise, like yoda
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


print(f"LLM {EMOJIS['llm']}: {LLM}")
print(f"VLM {EMOJIS['vlm']}: {VLM}")
print(f"TTS {EMOJIS['tts']}: {TTS}")
print(f"STT {EMOJIS['stt']}: {STT}")
state = deque([f"{EMOJIS['born']} robot is alive"], maxlen=MEMORY)
while datetime.now() - BIRTHDAY < LIFESPAN:
    if len(state) >= MEMORY:
        for _ in range(FORGET):
            state.popleft()
        state.appendleft(f"{EMOJIS['forget']} memory erased")
    for s in asyncio.run(sense()):
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
poem = llm(
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
tts(poem)
