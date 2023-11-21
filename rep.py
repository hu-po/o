import replicate

from util import timeit

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
    output = replicate.run(
        "meta/llama-2-13b-chat:f4e2de70d66816a838a89eeeb621910adffb0dd0baba3976c96980970978018d",
        input={"prompt": ...},
    )
    # The meta/llama-2-13b-chat model can stream output as it's running.
    # The predict method returns an iterator, and you can iterate over that output.
    for item in output:
        # https://replicate.com/meta/llama-2-13b-chat/versions/f4e2de70d66816a838a89eeeb621910adffb0dd0baba3976c96980970978018d/api#output-schema
        print(item, end="")


@timeit
def vlm(
    base64_image: str,
    prompt: str = VLM_PROMPT,
    vision_model: str = VLM_MODEL,
    max_tokens: int = MAX_TOKENS_VISION,
) -> str:
    output = replicate.run(
        "yorickvp/llava-13b:2facb4a474a0462c15041b78b1ad70952ea46b5ec6ad29583c0b29dbd4249591",
        input={"image": open("path/to/file", "rb")},
    )
    # The yorickvp/llava-13b model can stream output as it's running.
    # The predict method returns an iterator, and you can iterate over that output.
    for item in output:
        # https://replicate.com/yorickvp/llava-13b/versions/2facb4a474a0462c15041b78b1ad70952ea46b5ec6ad29583c0b29dbd4249591/api#output-schema
        print(item, end="")


@timeit
def tts(text: str, file_name: str, model: str = TTS_MODEL, voice: str = VOICE):
    output = replicate.run(
        "suno-ai/bark:b76242b40d67c76ab6742e987628a2a9ac019e11d56ab96c4e91ce03b79b2787",
        input={
            "prompt": "Hello, my name is Suno. And, uh \u2014 and I like pizza. [laughs] But I also have other interests such as playing tic tac toe."
        },
    )
    print(output)


@timeit
def stt(audio_path: str) -> str:
    output = replicate.run(
        "openai/whisper:4d50797290df275329f202e48c76360b3f22b08d28c196cbc54600319435f8d2",
        input={"audio": open("path/to/file", "rb")},
    )
    print(output)


MODELS = {
    "llm": llm,
    "vlm": vlm,
    "tts": tts,
    "stt": stt,
}
