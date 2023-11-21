import io
import os
import requests

from pydub import AudioSegment
from openai import OpenAI

from util import timeit

client = OpenAI()

VLM_MODEL: str = "gpt-4-vision-preview"
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
MAX_TOKENS_VISION: int = 16  # max tokens for reply
LLM_MODEL: str = "gpt-4-1106-preview"
LLM_SYSTEM_PROMPT: str = ". ".join(
    [
        "You are the function master node in a robot control system",
        "You decide when to run robot functions on behalf of the other robot nodes",
        "Use the log to avoid previous functions",
        "Do not use the same functions as before",
        "You can move to explore and understand the environment",
        # "The robot can observe the world through sight",
        # "The robot can observe the world through sound",
        "Make sure to often listen",
        "A good default is to listen",
        "If a human is visible, perform the greet action or speak to them",
        "If you hear a human, respond to them by speaking",
        "Try to move towards interesting things",
        "Always pick a function to run",
        "The other robot nodes depend on you",
        "Do not repeat functions",
    ]
)
LLM_MAX_TOKENS: int = 16
LLM_TEMPERATURE: float = 0.0
TTS_MODEL: str = "tts-1"  # Text-to-speech model
VOICE: str = "echo"  # (alloy, echo, fable, onyx, nova, and shimmer)
STT_MODEL: str = "whisper-1"  # Speech-to-text model


@timeit
def llm(
    prompt: str,
    language_model: str = LLM_MODEL,
    max_tokens: int = LLM_MAX_TOKENS,
    temperature: float = LLM_TEMPERATURE,
) -> str:
    response = "asd"
    return response


@timeit
def vlm(
    base64_image: str,
    prompt: str = VLM_PROMPT,
    vision_model: str = VLM_MODEL,
    max_tokens: int = MAX_TOKENS_VISION,
) -> str:
    headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {os.environ['OPENAI_API_KEY']}",
    }
    payload = {
        "model": vision_model,
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"},
                    },
                ],
            }
        ],
        "max_tokens": max_tokens,
    }
    response = requests.post(
        "https://api.openai.com/v1/chat/completions", headers=headers, json=payload
    )
    content = response.json()["choices"][0]["message"]["content"]
    return content


@timeit
def tts(text: str, file_name: str, model: str = TTS_MODEL, voice: str = VOICE):
    response = client.audio.speech.create(model=model, voice=voice, input=text)
    byte_stream = io.BytesIO(response.content)
    seg = AudioSegment.from_file(byte_stream, format="mp3")
    seg.export(file_name, format="mp3")


@timeit
def stt(audio_path: str) -> str:
    with open(audio_path, "rb") as f:
        transcript = client.audio.transcriptions.create(
            model=STT_MODEL, file=f, response_format="text"
        )
    return transcript


MODELS = {
    "llm": llm,
    "vlm": vlm,
    "tts": tts,
    "stt": stt,
}
