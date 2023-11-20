# o
autonomous humanoid robot

### LLM (Language Language Model)

https://replicate.com/andreasjansson/llama-2-13b-chat-gguf/api?tab=docker

docker run -d -p 5000:5000 --gpus=all r8.im/andreasjansson/llama-2-13b-chat-gguf@sha256:60ec5dda9ff9ee0b6f786c9d1157842e6ab3cc931139ad98fe99e08a35c5d4d4
curl http://localhost:5000/predictions -X POST \
-H "Content-Type: application/json" \
-d '{"input": {
  "prompt": "...",
    "grammar": "...",
    "jsonschema": "...",
    "max_tokens": "...",
    "temperature": "...",
    "top_p": "...",
    "top_k": "...",
    "frequency_penalty": "...",
    "presence_penalty": "...",
    "repeat_penalty": "...",
    "mirostat_mode": "...",
    "mirostat_learning_rate": "...",
    "mirostat_entropy": "..."
  }}'

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

soundevice requires pyaudio which requires portaudio
```
sudo apt-get install libportaudio2
sudo apt-get install python3-pyaudio
pip install sounddevice==0.4.6
pip install pydub==0.25.1
```