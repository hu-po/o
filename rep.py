import replicate
import requests
from pydub import AudioSegment

from util import timeit, encode_image

VLM_MODEL: str = "yorickvp/llava-13b:2facb4a474a0462c15041b78b1ad70952ea46b5ec6ad29583c0b29dbd4249591"
VLM_PROMPT: str = ". ".join(
    [
        "Describe the scene, objects, and characters",
        "You are a robot vision module",
        "You are small and only 20 centimeters off the ground",
        "Focus on the most important things",
        "If there are humans mention them and their relative position",
        "Do not mention the image",
        "Directly describe the scene",
        "Be concise",
        "Do not use punctuation",
        "Your response will be read out by the robot speech module",
        "Your reponse should not contain any special characters",
    ]
)
VLM_MAX_TOKENS: int = 24  # max tokens for reply
LLM_MODEL: str = "meta/llama-2-13b-chat:f4e2de70d66816a838a89eeeb621910adffb0dd0baba3976c96980970978018d"
# LLM_MODEL: str = "mistralai/mistral-7b-instruct-v0.1:83b6a56e7c828e667f21fd596c338fd4f0039b46bcfa18d973e8e70e455fda70"
LLM_MAX_TOKENS: int = 16
LLM_TEMPERATURE: float = 0.01
# Text-to-Speech
TTS_MODEL: str = (
    "suno-ai/bark:b76242b40d67c76ab6742e987628a2a9ac019e11d56ab96c4e91ce03b79b2787"
)
VOICE: str = "en_speaker_0"  # 9 total english speakers available
TTS_AUDIO_PATH: str = "/tmp/audio.wav"  # audio is constantly overwritten
# Speech-to-Text
STT_MODEL: str = (
    "openai/whisper:4d50797290df275329f202e48c76360b3f22b08d28c196cbc54600319435f8d2"
)


@timeit
def llm(
    prompt: str,
    model: str = LLM_MODEL,
    max_tokens: int = LLM_MAX_TOKENS,
    temperature: float = LLM_TEMPERATURE,
) -> str:
    # https://replicate.com/meta/llama-2-13b-chat
    output = replicate.run(
        model,
        input={
            "prompt": prompt,
            "max_new_tokens": max_tokens,
            "temperature": temperature,
        },
    )
    return "".join(output)


@timeit
def vlm(
    prompt: str = VLM_PROMPT,
    model: str = VLM_MODEL,
    max_tokens: int = VLM_MAX_TOKENS,
) -> str:
    # https://replicate.com/yorickvp/llava-13b
    base64_image = encode_image()
    output = replicate.run(
        model,
        input={
            "image": f"data:image/jpeg;base64,{base64_image}",
            "prompt": prompt,
            "max_tokens": max_tokens,
        },
    )
    return "".join(output)


@timeit
def tts(
    text: str,
    model: str = TTS_MODEL,
    voice: str = VOICE,
    tmp_path: str = TTS_AUDIO_PATH,
) -> AudioSegment:
    # https://replicate.com/suno-ai/bark
    output = replicate.run(
        model,
        input={
            "prompt": text,
            "history_prompt": voice,
        },
    )
    with open(tmp_path, "wb") as file:
        file.write(requests.get(output["audio_out"]).content)
    return AudioSegment.from_wav(tmp_path)


@timeit
def stt(audio_path: str, model: str = STT_MODEL) -> str:
    # https://replicate.com/openai/whisper
    output = replicate.run(
        model,
        input={"audio": open(audio_path, "rb")},
    )
    return output["transcription"]


MODELS = {
    "llm": llm,
    "vlm": vlm,
    "tts": tts,
    "stt": stt,
}

if __name__ == "__main__":
    seg = tts("hello world")
    seg.export("/tmp/test.mp3", format="mp3")
    print(stt("/tmp/test.mp3"))
    print(llm("hello"))
    print(vlm())
