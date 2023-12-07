echo "🔠 loading model_api rep test params"
export O_LLM="meta/llama-2-13b-chat:f4e2de70d66816a838a89eeeb621910adffb0dd0baba3976c96980970978018d"
export O_LLM_MAX_TOKENS=16
export O_LLM_TEMPERATURE=0.4
export O_VLM="yorickvp/llava-13b:2facb4a474a0462c15041b78b1ad70952ea46b5ec6ad29583c0b29dbd4249591"
export O_VLM_MAX_TOKENS=24
export O_TTS="suno-ai/bark:b76242b40d67c76ab6742e987628a2a9ac019e11d56ab96c4e91ce03b79b2787"
export O_VOICE="en_speaker_3"
export O_TTS_AUDIO_PATH="/tmp/audio.wav"
export O_STT="openai/whisper:4d50797290df275329f202e48c76360b3f22b08d28c196cbc54600319435f8d2"