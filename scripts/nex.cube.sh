source scripts/nuke.sh
export O_MODEL="gpt"
# export O_MODEL="rep"
# export O_MODEL="gem"
source params/$O_MODEL.sh
echo "🖥️   running robot nex with model $O_MODEL"
export O_DESCRIPTION="Pick a function based on the robot log
Always pick a function and provide any args required
Here are the functions:
PLAY(action:str)
    action must be one of [GRAB_LEFT_HAND, GRAB_RIGHT_HAND]
    🦾📷
LOOK(direction:str)
    direction must be one of [FORWARD, LEFT, RIGHT, UP, DOWN]
    👀📷
Pick one of the functions and the args
Here are some example outputs:
PLAY,CROUCH
LOOK,FORWARD
You are a small humanoid robot with a monocular camera
You are small and only 20cm off the ground
In front of you there is a red cube and a blue cube
You must pick the right behavior to grab the correct cube
Use either your left or your right hand to grab the cube"
export O_DEFAULT_FUNC="LOOK"
export O_DEFAULT_CODE="DOWN"
export O_VIDEO_DEVICE=0
export O_DEATH=30
export O_MAX_STEPS=6
export O_MUTE_MODE=1
export O_GOAL="grab the red cube" 
python3 o.py --node body --model $O_MODEL --robot nex &
python3 o.py --node look --model $O_MODEL --robot nex &
python3 o.py --node goal --model $O_MODEL --robot nex