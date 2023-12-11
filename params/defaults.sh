echo "🔠 loading test params"
export O_DEATH=6
export O_STEPS=0
export O_MAX_STEPS=3

export O_MEM_ID=0
export O_MEM_MAX_NUM=0
export O_MEM_MAX_SIZE=4096

export O_LLM="test llm"
export O_VLM="test vlm"
export O_TTS="test tts"
export O_STT="test stt"

export O_DESCRIPTION="You are a test robot."
export O_FUNCTIONS="MOVE(direction:str)
    direction must be one of [FORWARD, BACKWARD, LEFT, RIGHT]
    🦿📷"
export O_SUGGESTIONS="MOVE,FORWARD
MOVE,LEFT"
export O_DEFAULT_FUNC="MOVE"
export O_DEFAULT_CODE="FORWARD"

export O_VIDEO_DEVICE="/dev/video0"
export O_IMAGE_PATH="/tmp/o.image.jpeg"
export O_IMAGE_LOCK_PATH="/tmp/o.image.lock"

export O_AUDIO_RECORD_TIME=3
export O_AUDIO_SAMPLE_RATE=16000
export O_AUDIO_CHANNELS=1
export O_AUDIO_OUTPUT_PATH="/tmp/o.audio.wav"
export O_AUDIO_LOCK_PATH="/tmp/o.audio.lock"