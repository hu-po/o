# o
autonomous humanoid robot

### VLM (Video Language Model)

LLaVA 13B running locally at 8bit on a docker container (14G VRAM)

```
# Download original model from HuggingFace https://huggingface.co/liuhaotian/llava-v1.5-13b/tree/main
# Use replicate docker container: https://replicate.com/yorickvp/llava-13b
docker run --name llava13b r8.im/yorickvp/llava-13b@sha256:2facb4a474a0462c15041b78b1ad70952ea46b5ec6ad29583c0b29dbd4249591
docker commit llava13b llava13b

# Use mounted volume for locally stored weights (will also download clip336)
docker run -v /home/oop/dev/LLaVA/llava-v1.5-13b:/src/liuhaotian/llava-v1.5-13b --gpus=all llava13b
docker commit FOOO llava13b

# Inside src/predict.py you have to change line 85 "load_8bit=True"
docker run -it -v /home/oop/dev/LLaVA/llava-v1.5-13b:/src/liuhaotian/llava-v1.5-13b --gpus=all llava13b
docker cp 68f397c04dc0:/src/predict.py predict.py
docker cp predict.py 68f397c04dc0:/src/predict.py
docker commit 68f397c04dc0 llava13b
```

### STT (Speech To Text)

https://replicate.com/openai/whisper/api?tab=docker

docker run -d -p 5000:5000 --gpus=all r8.im/openai/whisper@sha256:4d50797290df275329f202e48c76360b3f22b08d28c196cbc54600319435f8d2
curl http://localhost:5000/predictions -X POST \
-H "Content-Type: application/json" \
-d '{"input": {
  "audio": "https://url/to/file",
    "model": "...",
    "transcription": "...",
    "translate": "...",
    "language": "...",
    "temperature": "...",
    "patience": "...",
    "suppress_tokens": "...",
    "initial_prompt": "...",
    "condition_on_previous_text": "...",
    "temperature_increment_on_fallback": "...",
    "compression_ratio_threshold": "...",
    "logprob_threshold": "...",
    "no_speech_threshold": "..."
  }}'

docker run --name whisper --gpus=all r8.im/openai/whisper@sha256:4d50797290df275329f202e48c76360b3f22b08d28c196cbc54600319435f8d2
docker commit whisper whisper
docker run -it --rm -p 5000:5000 --gpus=all whisper

### TTS (Text To Speech)

https://replicate.com/suno-ai/bark/api?tab=docker

docker run -d -p 5000:5000 --gpus=all r8.im/suno-ai/bark@sha256:b76242b40d67c76ab6742e987628a2a9ac019e11d56ab96c4e91ce03b79b2787
curl http://localhost:5000/predictions -X POST \
-H "Content-Type: application/json" \
-d '{"input": {
  "prompt": "...",
    "history_prompt": "...",
    "custom_history_prompt": "https://url/to/file",
    "text_temp": "...",
    "waveform_temp": "...",
    "output_full": "..."
  }}'

docker run --name bark r8.im/suno-ai/bark@sha256:b76242b40d67c76ab6742e987628a2a9ac019e11d56ab96c4e91ce03b79b2787
docker commit bark bark
docker run -it --rm -p 5000:5000 --gpus=all bark

### LLM (Language Language Model)

https://replicate.com/meta/llama-2-13b-chat/api?tab=docker

docker run -d -p 5000:5000 --gpus=all r8.im/meta/llama-2-13b-chat@sha256:f4e2de70d66816a838a89eeeb621910adffb0dd0baba3976c96980970978018d
curl http://localhost:5000/predictions -X POST \
-H "Content-Type: application/json" \
-d '{"input": {
  "prompt": "...",
    "system_prompt": "...",
    "max_new_tokens": "...",
    "min_new_tokens": "...",
    "temperature": "...",
    "top_p": "...",
    "top_k": "...",
    "stop_sequences": "...",
    "seed": "...",
    "debug": "..."
  }}'

docker run --name llama2-13b r8.im/meta/llama-2-13b-chat@sha256:f4e2de70d66816a838a89eeeb621910adffb0dd0baba3976c96980970978018d
docker commit llama2-13b llama2-13b
docker run -it --rm -p 5000:5000 --gpus=all llama2-13b