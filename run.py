import argparse
import asyncio
import os
import hashlib
from datetime import datetime, timedelta

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
BLIND: bool = True  # Do not use vision module
MUTE: bool = True  # Mute audio output
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
    description = vlm()
    return f"{EMOJIS['vlm']}{EMOJIS['success']} saw '{description}'"


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
    audio_data = sd.rec(
        int(duration * sample_rate),
        samplerate=sample_rate,
        channels=channels,
    )
    sd.wait()  # Wait until recording is finished
    write(output_path, sample_rate, audio_data)
    transcript = stt(output_path)
    return f"{EMOJIS['stt']}{EMOJIS['success']} heard '{transcript}'"


@timeit
async def observe(vlm: callable, stt: callable):
    vlm_result, stt_result = await asyncio.gather(_vlm(vlm), _stt(stt))
    return f"{vlm_result}\n{stt_result}"


REPERTOIRE = {
    "look": look_at,
    "move": move,
    "perform": perform,
    "speak": speak,
}


@timeit
def act(
    llm: callable,
    state: list,
) -> str:
    prompt = '\n'.join(state)
    system = "You are the function master node in a robot control system. "
    system += "You decide when to run robot functions on behalf of the other robot nodes. "
    system += "The other robot nodes depend on you. "
    system += "The user will provide the log containing previous function calls. "
    system += "You can move to explore and understand the environment. "
    system += "If a human is visible, perform the greet action or speak to them. "
    system += "If you hear a human, respond to them by speaking. "
    system += "Try to move towards interesting things. "
    system += "Always pick a function to run. "
    system += "Return the name of the function you decide to run and any arguments. "
    system += "The available functions are:"
    system += "LOOK\n"
    system += "\t param str: direction, one of {LOOKS}\n"
    system += "MOVE\n"
    system += "\t param str: direction, one of {MOVES}\n"
    system += "PERFORM\n"
    system += "\t param str: action, one of {ACTIONS}\n"
    system += "SPEAK\n"
    system += "\t param str: text, the text to speak, keep it short\n"
    choice = llm(system, prompt)
    print(f"Output from act: {choice}")
    if choice.upper() in REPERTOIRE:
        return REPERTOIRE[choice](llm)
    else:
        return f"{EMOJIS['llm']}{EMOJIS['fail']} could not run unknown function {choice}"


def autonomous_loop(
    models,
    lifespan: timedelta = LIFESPAN,
) -> None:
    birthday = datetime.now()
    print(f"{EMOJIS['born']} robot is alive")
    state = []
    num_steps = 0
    while datetime.now() - birthday < lifespan:
        num_steps += 1
        obs = asyncio.run(observe(models["vlm"], models["stt"]))
        if len(state) >= STATE_SIZE:
            state.pop(0)
        state.append(obs)
        print(f"*********** {EMOJIS['state']} at step {num_steps}")
        for s in state:
            print(s)
        print(f"*********** {EMOJIS['state']}")
        act(models["llm"], state)
    print(f"{EMOJIS['died']} robot is dead")


if __name__ == "__main__":
    if args.mode == "gpt":
        from gpt import MODELS
    elif args.mode == "rep":
        from rep import MODELS

    autonomous_loop(MODELS)
