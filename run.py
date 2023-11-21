import argparse
import asyncio
import base64
import hashlib
import os
from datetime import datetime, timedelta

import cv2
from pydub import AudioSegment
from pydub.playback import play
import sounddevice as sd
from scipy.io.wavfile import write

from bot import move, perform, look_at
from util import timeit, EMOJIS

argparser = argparse.ArgumentParser()
argparser.add_argument("--mode", type=str, required=True)
args = argparser.parse_args()

LIFESPAN: timedelta = timedelta(minutes=1) # How long the robot will live
DATE_FORMAT: str = "%d.%m.%Y"

BLIND: bool = True  # Do not use vision module
IMAGE_OUTPUT_PATH: str = "/tmp/image.jpg"  # Image is constantly overwritten

MUTE: bool = True  # Mute audio output
DEAF: bool = False  # Do not listen for audio input
GREETING: str = "hello there"  # Greeting is spoken on start
AUDIO_RECORD_TIME: int = 3  # Duration for audio recording
AUDIO_SAMPLE_RATE: int = 16000  # Sample rate for speedy audio recording
AUDIO_CHANNELS: int = 1  # mono
AUDIO_OUTPUT_PATH: str = "/tmp/audio.wav"  # recorded audio is constantly overwritten

STATE_SIZE: int = 2  # Number of observations to keep in the state


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
        tts(text, file_name)
    seg = AudioSegment.from_file(file_name, format="mp3")
    play(seg)
    return f"{EMOJIS['tts']}{EMOJIS['success']} said '{text}'"


async def _vlm(
    vlm: callable,
    image_path: str = IMAGE_OUTPUT_PATH,
    blind: bool = BLIND,
) -> str:
    if blind:
        return f"{EMOJIS['vlm']}{EMOJIS['fail']} could not see, robot is blind"
    frame = cv2.imread(image_path)
    if frame is None:
        return f"{EMOJIS['vlm']}{EMOJIS['fail']} could not see, image is empty"
    _, buffer = cv2.imencode(".jpg", frame)
    base64_image = base64.b64encode(buffer).decode("utf-8")
    description = vlm(base64_image)
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
def observe(
    vlm: callable,
    stt: callable,
):
    output = asyncio.run([_vlm(vlm), _stt(stt)])
    return output


REPERTOIRE = {
    "look": look_at,
    "move": move,
    "perform": perform,
    "speak": speak,
}


@timeit
def act(
    llm: callable,
    state: str,
    tools: dict = REPERTOIRE,
) -> str:
    return llm(state, tools)


def autonomous_loop(
    models,
    lifespan: timedelta = LIFESPAN,
) -> None:
    birthday = datetime.now()
    print(f"{EMOJIS['born']} born on {datetime.now().strftime(DATE_FORMAT)}")
    state = []
    num_steps = 0
    while datetime.now() - birthday < lifespan:
        num_steps += 1
        obs = observe(models["vlm"], models["stt"])
        if len(state) >= STATE_SIZE:
            state.pop(0)
        state.append(obs)
        print(f"*********** {EMOJIS['state']} at step {num_steps}")
        for s in state:
            print(s)
        print(f"*********** {EMOJIS['state']}")
        act(state, models["llm"])
    print(f"{EMOJIS['died']} died on {datetime.now().strftime(DATE_FORMAT)}")


if __name__ == "__main__":
    if args.mode == "gpt":
        from gpt import MODELS
    elif args.mode == "rep":
        from rep import MODELS

    autonomous_loop(MODELS)
