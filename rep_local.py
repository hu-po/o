import os
import subprocess
import time
import base64
import requests

class ReplicateContainer:

    def __init__(self,
                 name: str = 'llava13b',
                 port: str = '5000',
                 warmup: int = 25,
        ):
        self.nuke()
        self.name, self.port, self.warmup = name, port, warmup
        self.proc = subprocess.Popen([
            "docker", "run", "--rm", 
            "-p", f"{port}:{port}", 
            "--gpus=all", name
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

def send_request(
    filepath: str = '/tmp/image.jpg',
    docker_url: str = '',
    **kwargs,
) -> str:
    with open(filepath, "rb") as img_file:
        response = requests.post(
            docker_url,
            headers={"Content-Type": "application/json"},
            json={
                "input": {
                    "image": f"data:image/png;base64,{base64.b64encode(img_file.read()).decode('utf-8')}",
                    **kwargs,
                },
            },
        )
    reply = ''.join(response.json()["output"])
    return reply

def llm_request():
    pass

def vlm_request():
    pass

def tts_request():
    pass

def stt_request():
    pass

if __name__ == "__main__":
    pass