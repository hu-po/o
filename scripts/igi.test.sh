source scripts/nuke.sh
export O_MODEL="gpt"
# export O_MODEL="rep"
# export O_MODEL="gem"
source params/$O_MODEL.sh
echo "🖥️   running robot igi with model $O_MODEL"
export DISPLAY=:0
xdotool key shift
source params/defaults.sh
export O_DESCRIPTION="You are a small robot with a stereo camera vision module
Your vision module uses two cameras to infer 3d
Pick a function based on the robot log.
Always pick a function and provide any args required.
Here are the functions:
LOOK(direction:str)
    direction must be one of ["FORWARD", "LEFT", "RIGHT"]
    📷👀
Pick one of the functions and the args.
Here are some example outputs:
LOOK,LEFT
LOOK,FORWARD
Your response should be a single line with the chosen function code and arguments."
export O_DEFAULT_FUNC="LOOK"
export O_DEFAULT_CODE="FORWARD"
export O_LOOK_PROMPT="Describe the scene, objects, and characters
Focus on the most important things
If there are humans mention them and their relative position
Do not mention the image
Directly describe the scene
Be concise
Do not use punctuation
Your response will be read out by the robot speech module
Your reponse should not contain any special characters"
export O_TALK_PROMPT="Respond to the human in a few words.
Be short, laconic, and witty in your reply."
export O_MUTE_MODE=0
export O_DEATH=120
export O_MAX_STEPS=12
python3 o.py --node body --model $O_MODEL --robot igi &
python3 o.py --node look --model $O_MODEL --robot igi &
python3 o.py --node talk --model $O_MODEL --robot igi