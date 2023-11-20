import os
import subprocess
import time
import base64
from typing import Any, Dict

from hparams import HPARAMS
import requests

class VLMDocker:

    def __init__(self, name: str = 'llava13b', port: str = '5000', warmup: int = 25):
        self.nuke()
        self.name, self.port, self.warmup = name, port, warmup
        self.proc = subprocess.Popen([
            "docker", "run", "--rm", 
            "-v", "/home/oop/dev/LLaVA/llava-v1.5-13b:/src/liuhaotian/llava-v1.5-13b", 
            "-p", f"{port}:{port}", 
            "--gpus=all", 
            name
        ])
        time.sleep(self.warmup)

    def nuke(self):
        containers = os.popen("docker ps -aq").read().strip()
        if containers:
            os.system(f"docker stop {containers}")
            os.system(f"docker kill {containers}")
            os.system(f"docker rm {containers}")
        os.system("docker container prune -f")

    def __del__(self):
        self.proc.terminate()
        self.nuke()

async def run_vlm(
    prompt: str = HPARAMS["vlm_prompt"],
    docker_url: str = HPARAMS["vlm_docker_url"],
) -> Dict[str, Any]:
    log: str = f"{HPARAMS['vlm_token']} VLM using PROMPT: {prompt}"
    _path = os.path.join(HPARAMS["brain_data_dir"], HPARAMS["image_filename"])
    with open(_path, "rb") as img_file:
        response = requests.post(
            docker_url,
            headers={"Content-Type": "application/json"},
            json={
                "input": {
                    "image": f"data:image/png;base64,{base64.b64encode(img_file.read()).decode('utf-8')}",
                    "prompt": prompt,
                },
            },
        )
    reply = ''.join(response.json()["output"])
    log += f" REPLY: {reply}"
    print(f"\n{HPARAMS['vlm_token']} {log}\n")
    return {"log": log, "reply": reply}
