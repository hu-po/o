# Models

You can optionally use different model backends with the `--model` flag.

## OpenAI `--model gpt`

- **LLM:** `gpt-4-1106-preview`
- **VLM:** `gpt-4-vision-preview`
- **TTS:** `tts-1`
- **STT:** `whisper-1`

```
pip install openai==1.1.1
export OPENAI_API_KEY=...
```

## Replicate (Open Source) `--model rep`

- **LLM:** `meta/llama-2-13b-chat`
- **VLM:** `yorickvp/llava-13b`
- **TTS:** `suno-ai/bark`
- **STT:** `openai/whisper`

```
pip install replicate==0.20.0
export REPLICATE_API_TOKEN=...
```