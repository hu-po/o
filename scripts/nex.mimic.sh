echo "  🖥️    running robot nex with model_api gpt"
source scripts/nuke.sh
source params/nex.sh
source params/gpt.sh
export O_DESCRIPTION="You are a small humanoid robot with a monocular camera.
You are small and only 20cm off the ground"
export O_FUNCTIONS="
PLAY(action:str)
    action must be one of [ARM_WAVE_GREET, GRAB_LEFT_HAND, GRAB_RIGHT_HAND, CROUCH]
    🦾📷
LOOK(direction:str)
    direction must be one of [FORWARD, LEFT, RIGHT, UP, DOWN]
    👀📷"
export O_SUGGESTIONS="LOOK,RIGHT
PLAY,ARM_WAVE_GREET"
export O_DEFAULT_FUNC="LOOK"
export O_DEFAULT_CODE="FORWARD"
export O_DEATH=60
export O_MAX_STEPS=12
python3 o.py --node body --model_api gpt --robot nex &
python3 o.py --node look --model_api gpt --robot nex &
python3 o.py --node listen --model_api gpt --robot nex