import io
import os
import requests

from pydub import AudioSegment
from openai import OpenAI


client = OpenAI()

LLM = str(os.getenv("O_LLM", "gpt-4-1106-preview"))
LLM_MAX_TOKENS = int(os.getenv("O_LLM_MAX_TOKENS", 16))
LLM_TEMPERATURE = float(os.getenv("O_LLM_TEMPERATURE", 0.4))
VLM = str(os.getenv("O_VLM", "gpt-4-vision-preview"))
VLM_MAX_TOKENS = int(os.getenv("O_VLM_MAX_TOKENS", 24))
TTS = str(os.getenv("O_TTS", "tts-1"))
VOICE = str(os.getenv("O_VOICE", "onyx"))  # (alloy, echo, fable, onyx, nova, and shimmer)
STT = str(os.getenv("O_STT", "whisper-1"))


def llm(prompt: str, model: str = LLM) -> str:
    # https://platform.openai.com/docs/guides/text-generation/chat-completions-api
    response = client.chat.completions.create(
        model=model,
        max_tokens=LLM_MAX_TOKENS,
        temperature=LLM_TEMPERATURE,
        messages=[
            {"role": "user", "content": prompt},
        ],
    )
    reply = response.choices[0].message.content
    return reply


def vlm(prompt: str, base64_image: str) -> str:
    headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {os.environ['OPENAI_API_KEY']}",
    }
    payload = {
        "model": VLM,
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
        "max_tokens": VLM_MAX_TOKENS,
    }
    response = requests.post(
        "https://api.openai.com/v1/chat/completions", headers=headers, json=payload
    )
    content = response.json()["choices"][0]["message"]["content"]
    return content


def tts(text: str) -> AudioSegment:
    response = client.audio.speech.create(model=TTS, voice=VOICE, input=text)
    byte_stream = io.BytesIO(response.content)
    return AudioSegment.from_file(byte_stream, format="mp3")


def stt(audio_path: str) -> str:
    with open(audio_path, "rb") as f:
        transcript = client.audio.transcriptions.create(
            model=STT, file=f, response_format="text"
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
