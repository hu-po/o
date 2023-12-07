echo "🔠 loading test params"
export O_DEATH=6
export O_STEPS=0
export O_MAX_STEPS=3
export O_MEMORY_PATH="/tmp/o.memory.txt"
export O_MEMORY_LOCK_PATH="/tmp/o.memory.lock"
export O_MEMORY_MAX_SIZE=4096
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
export O_VIDEO_DEVICE=0
export O_IMAGE_PATH="/tmp/o.image.jpeg"
export O_IMAGE_LOCK_PATH="/tmp/o.image.lock"