import io
import os
import requests

from pydub import AudioSegment
from openai import OpenAI


client = OpenAI()

LLM: str = "gpt-4-1106-preview"
LLM_MAX_TOKENS: int = 16
LLM_TEMPERATURE: float = 0.0
VLM: str = "gpt-4-vision-preview"
VLM_MAX_TOKENS: int = 24
TTS: str = "tts-1"
VOICE: str = "onyx"  # (alloy, echo, fable, onyx, nova, and shimmer)
STT: str = "whisper-1"


def llm(
    prompt: str,
    model: str = LLM,
    max_tokens: int = LLM_MAX_TOKENS,
    temperature: float = LLM_TEMPERATURE,
) -> str:
    # https://platform.openai.com/docs/guides/text-generation/chat-completions-api
    response = client.chat.completions.create(
        model=model,
        max_tokens=max_tokens,
        temperature=temperature,
        messages=[
            {"role": "user", "content": prompt},
        ],
    )
    reply = response.choices[0].message.content
    return reply


def vlm(
    prompt: str,
    base64_image: str,
    model: str = VLM,
    max_tokens: int = VLM_MAX_TOKENS,
) -> str:
    headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {os.environ['OPENAI_API_KEY']}",
    }
    payload = {
        "model": model,
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


def tts(text: str, model: str = TTS, voice: str = VOICE) -> AudioSegment:
    response = client.audio.speech.create(model=model, voice=voice, input=text)
    byte_stream = io.BytesIO(response.content)
    return AudioSegment.from_file(byte_stream, format="mp3")


def stt(audio_path: str, model: str = STT) -> str:
    with open(audio_path, "rb") as f:
        transcript = client.audio.transcriptions.create(
            model=model, file=f, response_format="text"
        )
    return transcript


if __name__ == "__main__":
    seg = tts("hello world")
    seg.export("/tmp/test.mp3", format="mp3")
    print(stt("/tmp/test.mp3"))
    print(llm("hello"))

    import base64

    with open("/tmp/o.image.jpg", "rb") as f:
        base64_image = base64.b64encode(f.read()).decode("utf-8")
    print(vlm("what do you see?"))
