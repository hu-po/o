import os
import requests

from pydub import AudioSegment
import replicate

LLM = str(os.getenv("O_LLM", "meta/llama-2-13b-chat:f4e2de70d66816a838a89eeeb621910adffb0dd0baba3976c96980970978018d"))
LLM_MAX_TOKENS = int(os.getenv("O_LLM_MAX_TOKENS", 16))
LLM_TEMPERATURE = float(os.getenv("O_LLM_TEMPERATURE", 0.4))
VLM = str(os.getenv("O_VLM", "yorickvp/llava-13b:2facb4a474a0462c15041b78b1ad70952ea46b5ec6ad29583c0b29dbd4249591"))
VLM_MAX_TOKENS = int(os.getenv("O_VLM_MAX_TOKENS", 24))
TTS = str(os.getenv("O_TTS", "suno-ai/bark:b76242b40d67c76ab6742e987628a2a9ac019e11d56ab96c4e91ce03b79b2787"))
VOICE = str(os.getenv("O_VOICE", "en_speaker_3"))
TTS_AUDIO_PATH: str = os.getenv("O_TTS_AUDIO_PATH", "/tmp/audio.wav")
STT = str(os.getenv("O_STT", "openai/whisper:4d50797290df275329f202e48c76360b3f22b08d28c196cbc54600319435f8d2"))

def llm(prompt: str) -> str:
    # https://replicate.com/meta/llama-2-13b-chat
    output = replicate.run(
        LLM,
        input={
            "prompt": prompt,
            "max_new_tokens": LLM_MAX_TOKENS,
            "temperature": LLM_TEMPERATURE,
        },
    )
    return "".join(output)


def vlm(prompt: str, base64_image: str) -> str:
    # https://replicate.com/yorickvp/llava-13b
    output = replicate.run(
        VLM,
        input={
            "image": f"data:image/jpeg;base64,{base64_image}",
            "prompt": prompt,
            "max_tokens": VLM_MAX_TOKENS,
        },
    )
    return "".join(output)


def tts(text: str) -> AudioSegment:
    # https://replicate.com/suno-ai/bark
    output = replicate.run(
        TTS,
        input={
            "prompt": text,
            "history_prompt": VOICE,
        },
    )
    with open(TTS_AUDIO_PATH, "wb") as file:
        file.write(requests.get(output["audio_out"]).content)
    return AudioSegment.from_wav(TTS_AUDIO_PATH)


def stt(audio_path: str) -> str:
    # https://replicate.com/openai/whisper
    output = replicate.run(
        STT,
        input={"audio": open(audio_path, "rb")},
    )
    return output["transcription"]


if __name__ == "__main__":
    seg = tts("hello world")
    seg.export("/tmp/test.mp3", format="mp3")
    print(stt("/tmp/test.mp3"))
    print(llm("hello"))

    import base64

    with open("/tmp/image.jpg", "rb") as f:
        base64_image = base64.b64encode(f.read()).decode("utf-8")
    print(vlm("what do you see?"))
