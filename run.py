import argparse
import asyncio
import os
import hashlib
from datetime import datetime, timedelta
import subprocess

from pydub import AudioSegment
from pydub.playback import play
import sounddevice as sd
from scipy.io.wavfile import write

from nex import LOOKS, MOVES, ACTIONS
from util import EMOJIS

argparser = argparse.ArgumentParser()
argparser.add_argument("--mode", type=str, default="gpt")
args = argparser.parse_args()
if args.mode == "gpt":
    from gpt import llm, vlm, tts, stt
    from gpt import MODELS, LLM_MODEL, VLM_MODEL, TTS_MODEL, STT_MODEL
elif args.mode == "rep":
    from rep import llm, vlm, tts, stt
    from rep import MODELS, LLM_MODEL, VLM_MODEL, TTS_MODEL, STT_MODEL
print(f"########### {EMOJIS['brain']}")
print(f"{EMOJIS['llm']} {LLM_MODEL}")
print(f"{EMOJIS['vlm']} {VLM_MODEL}")
print(f"{EMOJIS['tts']} {TTS_MODEL}")
print(f"{EMOJIS['stt']} {STT_MODEL}")
print(f"########### {EMOJIS['brain']}")

LIFESPAN: timedelta = timedelta(minutes=1)  # How long the robot will live
STATE_SIZE: int = 256 # How many characters worth of state to keep
BLIND: bool = False  # Do not use vision module
MUTE: bool = True  # Mute audio output
DEAF: bool = False  # Do not listen for audio input
CRIP: bool = False  # Do not move
GREETING: str = "hello there"  # Greeting is spoken on start
AUDIO_RECORD_TIME: int = 5  # Duration for audio recording
AUDIO_SAMPLE_RATE: int = 16000  # Sample rate for speedy audio recording
AUDIO_CHANNELS: int = 1  # mono
AUDIO_OUTPUT_PATH: str = "/tmp/audio.wav"  # recorded audio is constantly overwritten
ROBOT_FILENAME: str = "nex.py"  # robot commands are run in a subprocess


async def _tts(text: str) -> str:
    if MUTE:
        return f"{EMOJIS['tts']}{EMOJIS['fail']} could not speak, robot is mute"
    file_name = f"/tmp/tmp{hashlib.sha256(text.encode()).hexdigest()[:10]}.mp3"
    if not os.path.exists(file_name):
        seg = tts(text)
        seg.export(file_name, format="mp3")
    seg = AudioSegment.from_file(file_name, format="mp3")
    play(seg)
    return f"{EMOJIS['tts']}{EMOJIS['success']} said '{text}'"


async def _vlm() -> str:
    if BLIND:
        return f"{EMOJIS['vlm']}{EMOJIS['fail']} could not see, robot is blind"
    try:
        description = vlm()
    except Exception as e:
        print(e)
        return f"{EMOJIS['vlm']}{EMOJIS['fail']} could not see"
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
        print(e)
        return f"{EMOJIS['stt']}{EMOJIS['fail']} could not hear"
    return f"{EMOJIS['stt']}{EMOJIS['success']} heard {transcript}"


async def _llm(prompt: str) -> [str, str]:
    try:
        reply = llm(prompt)
    except Exception as e:
        print(e)
        return f"{EMOJIS['llm']}{EMOJIS['fail']} could not think"
    return f"{EMOJIS['llm']}{EMOJIS['success']} picked {reply}", reply


async def robot(mode:str, name: str) -> str:
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
    results = await asyncio.gather(_vlm(), _stt(), _tts("observing"))
    return "\n".join(results)


async def act(mode: str, name: str, speech: str) -> str:
    results = await asyncio.gather(robot(mode, name), _tts(speech))
    return "\n".join(results)


async def plan(state: str) -> [str, str]:
    results = await asyncio.gather(*[
        _llm(f"""
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
"""),
_llm(f"""
Summarize the robot log in a couple clever words, be brief but precise
Here is the robot log
<robotlog>
{state}
</robotlog>
"""),
        _tts("deciding"),
    ])
    mode_full = results[0]
    print(f"___________{EMOJIS['llm']}")
    print(mode_full)
    print(f"___________{EMOJIS['llm']}")
    speech = results[1]
    print(f"___________{EMOJIS['llm']}")
    print(speech)
    print(f"___________{EMOJIS['llm']}")
    return mode_full, speech


def autonomous_loop(
    models: dict,
    lifespan: timedelta = LIFESPAN,
) -> None:
    birthday = datetime.now()
    print(f"{EMOJIS['born']} robot is alive")
    state = ""
    num_steps = 0
    while datetime.now() - birthday < lifespan:
        num_steps += 1
        if len(state) >= STATE_SIZE:
            state_as_list = state.splitlines()
            split_in_half = STATE_SIZE // 2
            state = "\n".join(state_as_list[-split_in_half:])
        state += asyncio.run(sense(models))
        print(f"*********** {EMOJIS['state']}")
        print(state)
        print(f"*********** {EMOJIS['state']} at step {num_steps}")
        mode_full, speech = asyncio.run(plan(state))
        mode, name = mode_full.split(",")
        state += act(mode, name, speech)
    print(f"{EMOJIS['died']} robot is dead")


autonomous_loop(MODELS)