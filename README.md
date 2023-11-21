# o

autonomous humanoid robot, uses APIs for vlm, tts, stt, llm

## OpenAI API

- **LLM:** `gpt-4-1106-preview`
- **VLM:** `gpt-4-vision-preview`
- **TTS:** `tts-1`
- **STT:** `whisper-1`

```
pip install openai==1.1.1
export OPENAI_API_KEY=...
python run.py --mode gpt
```

## Open Source API

- **LLM:** `meta/llama-2-13b-chat`
- **VLM:** `yorickvp/llava-13b`
- **TTS:** `suno-ai/bark`
- **STT:** `openai/whisper`

```
pip install replicate==0.20.0
export REPLICATE_API_TOKEN=...
python run.py --mode rep
```

## Robot

Robot is a HiWonder AiNex humanoid running ROS on a Raspberry Pi 4B

Some generic dependencies:

```
pip install opencv-python==4.8.1.78
pip install scipy==1.10.1
```

Audio dependency soundevice requires pyaudio which requires portaudio

```
sudo apt-get install libportaudio2
sudo apt-get install python3-pyaudio
pip install sounddevice==0.4.6
pip install pydub==0.25.1
```