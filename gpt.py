import os
import requests

from openai import OpenAI

from util import timeit, encode_image

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
VLM_MAX_TOKENS: int = 16  # max tokens for reply
LLM_MODEL: str = "gpt-4-1106-preview"
LLM_MAX_TOKENS: int = 16
LLM_TEMPERATURE: float = 0.0
# Text-to-Speech
TTS_MODEL: str = "tts-1"
VOICE: str = "echo"  # (alloy, echo, fable, onyx, nova, and shimmer)
# Speech-to-Text
STT_MODEL: str = "whisper-1"  # Speech-to-text model


@timeit
def llm(
    prompt: str,
    system: str,
    model: str = LLM_MODEL,
    max_tokens: int = LLM_MAX_TOKENS,
    temperature: float = LLM_TEMPERATURE,
) -> str:
    # https://platform.openai.com/docs/guides/text-generation/chat-completions-api
    response = client.chat.completions.create(
        model=model,
        max_tokens=max_tokens,
        temperature=temperature,
        messages=[
            {"role": "system", "content": system},
            {"role": "user", "content": prompt},
        ],
    )
    reply = response.choices[0].message.content
    return reply


@timeit
def vlm(
    prompt: str = VLM_PROMPT,
    model: str = VLM_MODEL,
    max_tokens: int = VLM_MAX_TOKENS,
) -> str:
    base64_image = encode_image()
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


@timeit
def tts(text: str, model: str = TTS_MODEL, voice: str = VOICE) -> bytes:
    response = client.audio.speech.create(model=model, voice=voice, input=text)
    return response.content


@timeit
def stt(audio_path: str, model: str = STT_MODEL) -> str:
    with open(audio_path, "rb") as f:
        transcript = client.audio.transcriptions.create(
            model=model, file=f, response_format="text"
        )
    return transcript


MODELS = {
    "llm": llm,
    "vlm": vlm,
    "tts": tts,
    "stt": stt,
}

if __name__ == "__main__":
    from util import bytes_to_audio

    _bytes = tts("hello world")
    bytes_to_audio(_bytes, "/tmp/test.mp3")
    print(stt("/tmp/test.mp3"))
    print(llm("you are a robot", "hello"))
    print(vlm())
