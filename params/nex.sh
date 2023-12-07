echo "🔠 loading robot nex test params"
export O_DESCRIPTION="You are a small robot with a stereo camera vision module
Your vision module uses two cameras to infer 3d"
export O_FUNCTIONS="LOOK(direction:str)
  direction must be one of [FORWARD, LEFT, RIGHT, UP, DOWN]
  📷👀"
export O_SUGGESTIONS="LOOK,LEFT
LOOK,UP
LOOK,FORWARD"
export O_DEFAULT_FUNC="LOOK"
export O_DEFAULT_CODE="FORWARD"