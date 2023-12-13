echo "🖥️   running robot nex with model_api gpt"
source scripts/nuke.sh
source params/gpt.sh
export O_DESCRIPTION="You are a small humanoid robot with a monocular camera
You are small and only 20cm off the ground
Pick a function based on the robot log.
Always pick a function and provide any args required.
Here are the functions:
PLAY(action:str)
    action must be one of [ARM_WAVE_GREET, GRAB_LEFT_HAND, GRAB_RIGHT_HAND, CROUCH]
    🦾📷
LOOK(direction:str)
    direction must be one of [FORWARD, LEFT, RIGHT, UP, DOWN]
    👀📷
Pick one of the functions and the args.
Here are some example outputs:
PLAY,CROUCH
LOOK,FORWARD"
export O_DEFAULT_FUNC="LOOK"
export O_DEFAULT_CODE="FORWARD"
export O_VIDEO_DEVICE=0
export O_DEATH=120
export O_MAX_STEPS=12
export O_MUTE_MODE=1
python3 o.py --node body --model_api gpt --robot nex &
python3 o.py --node look --model_api gpt --robot nex &
python3 o.py --node talk --model_api gpt --robot nex