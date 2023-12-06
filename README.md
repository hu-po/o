# o

**o** stands for Zer**o**-Sh**o**t Aut**o**n**o**m**o**us R**o**b**o**ts.

Talks with TTS and STT, sees with VLM, and thinks with LLM.

To get started [follow the setup guide](docs/setup.md).

The `models` module contains code for different model apis. For example `models/rep.py` is for the open source Replicate API, and `models/gpt.py` is for the OpenAI API. [More info on models](docs/models.md).

The `robots` module contains code for different robots. For example `robots/nex.py` is for the HiWonder AiNex Humanoid. [More info on robots](docs/robots.md).

The `nodes` module contains code for different nodes. For example `nodes/look.py` contains the loop used vision with the VLM. [More info on nodes](docs/nodes.md).

If you are interested in contributing, please read the [contributing guide](docs/contributing.md).

## Citation

```
@misc{zero-shot-humanoid-2023,
  title={Zero-Shot Autonomous Robots},
  author={Hugo Ponte},
  year={2023},
  url={https://github.com/hu-po/o}
}
```