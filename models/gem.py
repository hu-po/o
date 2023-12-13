import os

import google.generativeai as genai


# https://ai.google.dev/tutorials/python_quickstart
genai.configure(api_key=os.environ["GOOGLE_API_KEY"])

LLM = str(os.getenv("O_LLM", "gemini-pro"))
LLM_MAX_TOKENS = int(os.getenv("O_LLM_MAX_TOKENS", 16))
LLM_TEMPERATURE = float(os.getenv("O_LLM_TEMPERATURE", 0.4))
VLM = str(os.getenv("O_VLM", "gemini-pro-vision"))
VLM_MAX_TOKENS = int(os.getenv("O_VLM_MAX_TOKENS", 24))


def llm(prompt: str, model: str = LLM) -> str:
    model = genai.GenerativeModel(model)
    response = model.generate(prompt, max_length=LLM_MAX_TOKENS, temperature=LLM_TEMPERATURE)
    return response.text


def vlm(prompt: str, base64_image: str) -> str:
    model = genai.GenerativeModel(VLM)
    # TODO: base64 image to PIL image
    response = model.generate_content([prompt, img], stream=True)
    response.resolve()
    return response.text


if __name__ == "__main__":
    print(llm("hello"))

    import base64

    with open("/tmp/o.image.jpg", "rb") as f:
        base64_image = base64.b64encode(f.read()).decode("utf-8")
    print(vlm("what do you see?"))
