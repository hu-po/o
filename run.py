import argparse
import asyncio
import os
import hashlib
from datetime import datetime, timedelta
import subprocess
from collections import deque

from pydub import AudioSegment
from pydub.playback import play
import sounddevice as sd
from scipy.io.wavfile import write


from util import EMOJIS

argparser = argparse.ArgumentParser()
argparser.add_argument("--model_api", type=str, default="test")
argparser.add_argument("--robot", type=str, default="test")
args = argparser.parse_args()
if args.model_api == "gpt":
    from gpt import llm, vlm, tts, stt
    from gpt import LLM_MODEL, VLM_MODEL, TTS_MODEL, STT_MODEL
elif args.model_api == "rep":
    from rep import llm, vlm, tts, stt
    from rep import LLM_MODEL, VLM_MODEL, TTS_MODEL, STT_MODEL
elif args.model_api == "test":
    LLM_MODEL, VLM_MODEL, TTS_MODEL, STT_MODEL = ["test"]*4
    def llm(x):
        return "test llm reply,"
    def vlm(x):
        return "test vlm reply"
    def tts(x):
        return "test tts reply"
    def stt(x):
        return None

print(f"########### {EMOJIS['brain']}")
print(f"{EMOJIS['llm']} {LLM_MODEL}")
print(f"{EMOJIS['vlm']} {VLM_MODEL}")
print(f"{EMOJIS['tts']} {TTS_MODEL}")
print(f"{EMOJIS['stt']} {STT_MODEL}")
print(f"########### {EMOJIS['brain']}")
if args.robot == "nex":
    from nex import LOOKS, MOVES, ACTIONS

    ROBOT_FILENAME: str = "nex.py"  # robot commands are run in a subprocess
elif args.robot == "test":
    LOOKS = ["up", "down"]
    MOVES = ["forward", "backward"]
    ACTIONS = ["greet"]
    ROBOT_FILENAME: str = "oop.py"


BIRTHDAY: datetime = datetime.now()
LIFESPAN: timedelta = timedelta(minutes=5)  # How long the robot will live
MEMORY: int = 32  # How many characters worth of state to keep in memory
FORGET: int = 8  # How many characters worth of state to forget 
BLIND: bool = True  # Do not use vision module
MUTE: bool = True  # Mute audio output
DEAF: bool = False  # Do not listen for audio input
CRIP: bool = False  # Do not move
GREETING: str = "hello there"  # Greeting is spoken on start
AUDIO_RECORD_TIME: int = 3  # Duration for audio recording
AUDIO_SAMPLE_RATE: int = 16000  # Sample rate for speedy audio recording
AUDIO_CHANNELS: int = 1  # mono
AUDIO_OUTPUT_PATH: str = "/tmp/audio.wav"  # recorded audio is constantly overwritten


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
        return f"{EMOJIS['tts']}{EMOJIS['fail']} could not speak, ERROR"
    return f"{EMOJIS['tts']}{EMOJIS['success']} said '{text}'"


async def _vlm() -> str:
    if BLIND:
        return f"{EMOJIS['vlm']}{EMOJIS['fail']} could not see, robot is blind"
    try:
        description = vlm()
    except Exception as e:
        # print(e)
        return f"{EMOJIS['vlm']}{EMOJIS['fail']} could not see, ERROR"
    return f"{EMOJIS['vlm']}{EMOJIS['success']} saw {description}"


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
        return f"{EMOJIS['stt']}{EMOJIS['fail']} could not hear, ERROR"
    return f"{EMOJIS['stt']}{EMOJIS['success']} heard {transcript}"


async def _llm(prompt: str) -> [str, str]:
    try:
        reply = llm(prompt)
    except Exception as e:
        # print(e)
        return f"{EMOJIS['llm']}{EMOJIS['fail']} could not think, ERROR"
    return f"{EMOJIS['llm']}{EMOJIS['success']} picked {reply}", reply


async def robot(mode: str, name: str) -> str:
    if CRIP:
        return f"{EMOJIS['robot']}{EMOJIS['fail']} cannot act, robot is crippled"
    _path = os.path.join(os.path.dirname(os.path.realpath(__file__)), ROBOT_FILENAME)
    try:
        proc = subprocess.Popen(
            ["python3", _path, mode, name],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        stdout, stderr = proc.communicate()
    except Exception as e:
        print(f"Exception on robot {e}, {stderr}")
        return f"{EMOJIS['robot']}{EMOJIS['fail']} failed trying to {mode} {name}"
    return stdout


async def sense() -> str:
    return await asyncio.gather(_vlm(), _stt(), _tts("observing"))


async def act(mode: str, name: str, speech: str) -> str:
    return await asyncio.gather(robot(mode, name), _tts(speech))


async def plan(state: str) -> [str, str, str]:
    results = await asyncio.gather(
        *[
            _llm(
                f"""
Pick a function based on the robot log. Always pick a function and provide any args required. Here are the functions:
MOVE(direction:str)
  direction must be one of {MOVES}
PERFORM(action:str)
  action must be one of {ACTIONS}
LOOK(direction:str)
  direction must be one of {LOOKS}
Here is the robot log
<robotlog>
{state}
</robotlog>
Pick one of the functions and the args. Here are some example outputs:
PERFORM,greet
LOOK,up
MOVE,forward
Your response should be a single line with the chosen function name and arguments.
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
    mode, name = results[0][1].split(",")
    speech = results[1][1]
    return mode, name, speech


state = deque([f"{EMOJIS['born']} robot is alive"], maxlen=MEMORY)
while datetime.now() - BIRTHDAY < LIFESPAN:
    if len(state) >= MEMORY:
        for _ in range(FORGET):
            state.popleft()
        state.appendleft(f"{EMOJIS['forget']} memory erased")
    for s in asyncio.run(sense()):
        state.append(s)
    print(f"*********** {EMOJIS['state']} age {datetime.now() - BIRTHDAY}")
    state_str = "\n".join([str(item) for item in state])
    print(state_str)
    print(f"*********** {EMOJIS['state']}")
    mode, name, speech = asyncio.run(plan(state_str))
    print(f"___________{EMOJIS['llm']}")
    print(speech)
    print(mode, name)
    state.append(f"{EMOJIS['robot']} using function {mode} {name}")
    print(f"___________{EMOJIS['llm']}")
    for s in asyncio.run(act(mode, name, speech)):
        state.append(s)
state.append(f"{EMOJIS['dead']} robot is dead, lived for {LIFESPAN}")
poem = llm(f"""
Here is the robot log
<robotlog>
{state}
</robotlog>
""")
print(f"~~~~~~~~~~~ {EMOJIS['poem']}")
print(poem)
print(f"~~~~~~~~~~~ {EMOJIS['poem']}")