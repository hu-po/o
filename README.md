# o

**o** stands for Zer**o**-Sh**o**t Aut**o**n**o**m**o**us R**o**b**o**ts.

![o](docs/cover.jpeg)

This repo uses model APIs to create a Zero-Shot Autonomous Robot. Individual robot behaviors are wrapped in asynchronous nodes (python) which are launched via scripts (bash). It's kind of like a more minimalist and simpler ROS. Four main types of models are used:

- **LLM** (Language Language Model) a `text2text` model used for planning, reasoning, dialogue, and more!
- **VLM** (Vision Language Model) - a `image2text` model used for scene understanding, object detection, and more!
- **TTS** (Text-to-Speech) a `text2audio` model used for speech synthesis so the robot can talk.
- **STT** (Speech-to-Text) a `audio2text` model used for speech recognition so the robot can listen.

To get started [follow the setup guide](docs/setup.md).

The `models` module contains code for different model apis. For example `models/rep.py` is for the open source Replicate API, and `models/gpt.py` is for the OpenAI API. [More info on models](docs/models.md).

The `robots` module contains code for different robots. For example `robots/nex.py` is for the HiWonder AiNex Humanoid. [More info on robots](docs/robots.md).

The `nodes` module contains code for different nodes. For example `nodes/look.py` contains the loop used vision with a Vision Language Model. [More info on nodes](docs/nodes.md).

The `params` module contains code for different parameters. For example `params/default.sh` will load environment variables (params) that contain default values. [More info on params](docs/params.md).

If you are interested in contributing, please read the [contributing guide](docs/contributing.md).

## Video

[![YouTube Video](https://img.youtube.com/vi/bN9_ml4f05M/0.jpg)](https://www.youtube.com/watch?v=bN9_ml4f05M)

## Citation

```
@misc{zero-shot-robot-2023,
  title={Zero-Shot Autonomous Robots},
  author={Hugo Ponte},
  year={2023},
  url={https://github.com/hu-po/o}
}
```
