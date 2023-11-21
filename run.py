import argparse
import asyncio
import os
import hashlib
from datetime import datetime, timedelta
import functools

from pydub import AudioSegment
from pydub.playback import play
import sounddevice as sd
from scipy.io.wavfile import write

from bot_look import look_at, LOOKS
from bot_move import move, MOVES
from bot_perform import perform, ACTIONS
from util import timeit, EMOJIS

argparser = argparse.ArgumentParser()
argparser.add_argument("--mode", type=str, required=True)
args = argparser.parse_args()

LIFESPAN: timedelta = timedelta(minutes=1)  # How long the robot will live
STATE_SIZE: int = 2  # Number of observations to keep in the state
BLIND: bool = False  # Do not use vision module
MUTE: bool = False  # Mute audio output
DEAF: bool = False  # Do not listen for audio input
GREETING: str = "hello there"  # Greeting is spoken on start
AUDIO_RECORD_TIME: int = 3  # Duration for audio recording
AUDIO_SAMPLE_RATE: int = 16000  # Sample rate for speedy audio recording
AUDIO_CHANNELS: int = 1  # mono
AUDIO_OUTPUT_PATH: str = "/tmp/audio.wav"  # recorded audio is constantly overwritten


@timeit
def speak(
    tts: callable,
    text: str = GREETING,
    mute: bool = MUTE,
) -> str:
    if mute:
        return f"{EMOJIS['tts']}{EMOJIS['fail']} could not speak, robot is mute"
    file_name = f"/tmp/tmp{hashlib.sha256(text.encode()).hexdigest()[:10]}.mp3"
    if not os.path.exists(file_name):
        seg = tts(text)
        seg.export(file_name, format="mp3")
    seg = AudioSegment.from_file(file_name, format="mp3")
    play(seg)
    return f"{EMOJIS['tts']}{EMOJIS['success']} said '{text}'"


async def _vlm(
    vlm: callable,
    blind: bool = BLIND,
) -> str:
    if blind:
        return f"{EMOJIS['vlm']}{EMOJIS['fail']} could not see, robot is blind"
    try:
        description = vlm()
    except Exception as e:
        print(e)
        return f"{EMOJIS['vlm']}{EMOJIS['fail']} could not see"
    return f"{EMOJIS['vlm']}{EMOJIS['success']} saw {description}"


async def _stt(
    stt: callable,
    duration: int = AUDIO_RECORD_TIME,
    sample_rate: int = AUDIO_SAMPLE_RATE,
    channels: int = AUDIO_CHANNELS,
    output_path: str = AUDIO_OUTPUT_PATH,
    deaf: bool = DEAF,
) -> str:
    if deaf:
        return f"{EMOJIS['stt']}{EMOJIS['fail']} could not hear, robot is deaf"
    try:
        audio_data = sd.rec(
            int(duration * sample_rate),
            samplerate=sample_rate,
            channels=channels,
        )
        sd.wait()  # Wait until recording is finished
        write(output_path, sample_rate, audio_data)
        transcript = stt(output_path)
    except Exception as e:
        print(e)
        return f"{EMOJIS['stt']}{EMOJIS['fail']} could not hear"
    return f"{EMOJIS['stt']}{EMOJIS['success']} heard {transcript}"


async def observe(vlm: callable, stt: callable):
    vlm_result, stt_result = await asyncio.gather(_vlm(vlm), _stt(stt))
    return f"{vlm_result}\n{stt_result}"


REPERTOIRE = {
    "SPEAK": speak,
    "MOVE": move,
    "PERFORM": perform,
    "LOOK": look_at,
}


@timeit
def choose_action(
    models: dict,
    repertoire: dict,
    state: list,
) -> str:
    prompt = f"""
Pick a function based on the robot log. Always pick a function and provide any args required. Here are the functions:
SPEAK(text: str)
  text should be short and relevant to what the robot sees and hears
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
SPEAK,hello my name is robot
PERFORM,greet
MOVE,forward
LOOK,up
Your response should be a single line with the chosen function name and arguments.
"""
    choice = models['llm'](prompt)
    func_name, args = choice.split(",")
    if func_name in repertoire:
        _msg = f"{EMOJIS['llm']}{EMOJIS['success']} running {func_name}({args})\n"
        if func_name == "MOVE":
            models["speak"]("moving")
        if func_name == "PERFORM":
            models["speak"](f"performing {args}")
        _msg += repertoire[func_name](args)
        return _msg
    else:
        return f"{EMOJIS['llm']}{EMOJIS['fail']} unknown function {func_name}"


def autonomous_loop(
    models: dict,
    repertoire: dict,
    lifespan: timedelta = LIFESPAN,
) -> None:
    birthday = datetime.now()
    print(f"{EMOJIS['born']} robot is alive")
    state = []
    num_steps = 0
    while datetime.now() - birthday < lifespan:
        num_steps += 1
        obs = asyncio.run(observe(models["vlm"], models["stt"]))
        state.append(obs)
        if len(state) >= STATE_SIZE:
            state.pop(0)
        _state = "\n".join(state)
        print(f"*********** {EMOJIS['state']} at step {num_steps}")
        print(_state)
        print(f"*********** {EMOJIS['state']}")
        act = choose_action(models["llm"], repertoire, _state)
        state.append(act)
    print(f"{EMOJIS['died']} robot is dead")


if __name__ == "__main__":
    if args.mode == "gpt":
        from gpt import MODELS
    elif args.mode == "rep":
        from rep import MODELS

    REPERTOIRE["SPEAK"] = functools.partial(speak, tts=MODELS["tts"])
    autonomous_loop(MODELS, REPERTOIRE)
