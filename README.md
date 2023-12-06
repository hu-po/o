# o

**Zero-Shot Autonomous Humanoid Robot**

Talks with TTS and STT, sees with VLM, and thinks with LLM.

## What is this repo structure?

Scripts inteded to be run are called `s.*.sh`, so for example `s.test.sh` is for testing, `s.watch.sh` is a live barebones visualizer, and `s.nex.gpt.sh` runs the AiNex robot with the OpenAI API.

Python scripts `o.*.py` are repeating asynchronous loops, kind of like mini ROS nodes. They run in parallel and communicate via a shared files in `/tmp/`. File locking ensures no corrupted reads and writes. It isn't efficient but because the API calls are the bottleneck it doesnt matter.

`models` contains model api code, `models/rep.py` is for the open source Replicate API, and `models/gpt.py` is for the OpenAI API.

`robots` contains robot code, `robots/nex.py` is for the HiWonder AiNex humanoid robot.

## Emojis

Great emoji database https://emojidb.org/

## Citation

```
@misc{zero-shot-humanoid-2023,
  title={Zero-Shot Autonomous Humanoid Robot},
  author={Hugo Ponte},
  year={2023},
  url={https://github.com/hu-po/o}
}
```