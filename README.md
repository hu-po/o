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
python run.py --model_api gpt
```

## Replicate API (Open Source)

- **LLM:** `meta/llama-2-13b-chat`
- **VLM:** `yorickvp/llava-13b`
- **TTS:** `suno-ai/bark`
- **STT:** `openai/whisper`

```
pip install replicate==0.20.0
export REPLICATE_API_TOKEN=...
python run.py --model_api rep
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

To autostart script on robot boot use crontab (sleep for some time to allow ROS to spin up):

```
crontab -e
@reboot sleep 30 && sh /home/ubuntu/o/launch.sh
```

## Reddit Post

[r/Robotics](https://www.reddit.com/r/robotics/comments/1818x5t/zeroshot_autonomous_humanoid/)
[r/OpenAI](https://www.reddit.com/r/OpenAI/comments/1818yeg/humanoid_robot_with_gpt4v/)

I created a humanoid robot that can see, hear, listen, and speak all in real time. I am using a VLM (vision language model) to interpret images, TTS and STT (Speech-to-Text and Text-to-Speech) for the listening and speaking, and a LLM (language language model) to decide what to do and generate the speech text. All the model inference is through API because the robot is too tiny to perform the compute itself. The robot is a HiWonder AiNex running ROS (Robot Operating System) on a Raspberry Pi 4B.

I implemented a toggle between two different modes:

Open Source Mode:
- LLM: llama-2-13b-chat
- VLM: llava-13b
- TTS: bark
- STT: whisper

OpenAI Mode:
- LLM: gpt-4-1106-preview
- VLM: gpt-4-vision-preview
- TTS: tts-1
- STT: whisper-1

The robot runs a sense-plan-act loop where the observation (VLM and STT) is used by the LLM to determine what actions to take (moving, talking, performing a greet, etc). I open sourced (MIT) the code here: https://github.com/hu-po/o

Thanks for watching let me know what you think, I plan on working on this little buddy more in the future.

## Twitter (X) Post

[Post](https://x.com/hupobuboo/status/1727316167138886118)

Zero-Shot Autonomous Humanoid Robot using @replicate and @OpenAI

Open Source Mode (via Replicate API):
- LLM: llama-2-13b-chat
- VLM: llava-13b
- TTS: bark
- STT: whisper

OpenAI Mode:
- LLM: gpt-4-1106-preview
- VLM: gpt-4-vision-preview
- TTS: tts-1
- STT: whisper-1
