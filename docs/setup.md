# Setup Instructions

Install some generic dependencies:

```
pip install opencv-python==4.8.1.78
pip install scipy==1.11.4
pip install filelock==3.13.1
pip install requests==2.25.1
```

Test your dependencies with:

```
python o.py
```

Test the model apis with:

```
./scripts/test.sh
```

The basic workflow is to use scripts in the `scripts` submodule. For example `scripts/test.sh` will test the model apis, and `nex.gpt.sh` runs the AiNex robot with the OpenAI API.

### Visualizer

The visualizer will display local system information related to your `o` runtimes, nodes, robots, and more. Since state is stored in `/tmp/o.*` files the visualizer just uses terminal commands (you can use it through ssh).

```
./scripts/viz.sh
```