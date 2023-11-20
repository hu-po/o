# o
autonomous humanoid robot

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

### TTS (Text To Speech)

### STT (Speech To Text)

soundevice requires pyaudio which requires portaudio
```
sudo apt-get install libportaudio2
sudo apt-get install python3-pyaudio
pip install sounddevice==0.4.6
pip install pydub==0.25.1
```