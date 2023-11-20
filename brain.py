import asyncio
import os

from hparams import HPARAMS, Task
from utils import find_file, send_file, task_batch, write_log, clear_data
from replicate_container import VLMDocker, run_vlm


def _loop():
    docker_proc = VLMDocker()
    tasks = [Task("clear_data", clear_data("brain"))]
    while True:
        state = asyncio.run(task_batch(tasks, "brain"))
        # Reset tasks
        tasks = []
        # if log hasn't been saved in a while
        if state.get("brainlog", None) is None:
            _clean = (state.get("brainlog_age", 0) > HPARAMS["brainlog_max_age"])
            tasks.append(Task("write_log", write_log(state["log"], "brain", clean=_clean)))
        # always check for brainlog
        tasks.append(Task("find_file", find_file("brainlog", "brain", read=True)))
        # if there is an image, run VLM
        if state.get("image_path", None) is not None:
            tasks.append(Task("run_vlm", run_vlm(), HPARAMS["vlm_timeout"]))
        else:
            tasks.append(Task("find_file", find_file("image", "brain")))
        # if there is a VLM output, write and send
        if state.get("reply", None) is not None:
            _path = os.path.join(HPARAMS["brain_data_dir"], HPARAMS["vlmout_filename"])
            # TODO: check if path exists, if it is abve a certain size, delete it
            with open(_path, "w") as f:
                f.write(state["reply"])
            tasks.append(Task("send_file", send_file("vlmout", "brain", "robot")))


if __name__ == "__main__":
    print("Starting brain main loop.")
    _loop()

