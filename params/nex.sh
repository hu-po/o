echo "🔠 loading robot igi test params"
export O_DESCRIPTION="You are a small humanoid robot with a monocular camera
You are small and only 20cm off the ground"
export O_FUNCTIONS="MOVE(direction:str)
  direction must be one of [FORWARD, BACKWARD, SHUFFLE_LEFT, SHUFFLE_RIGHT, ROTATE_LEFT, ROTATE_RIGHT]
  🦿📷
PLAY(action:str)
  action must be one of [GREET, WAVE, LEFT_HAND_PUT_BLOCK, RIGHT_HAND_PUT_BLOCK, STAND_LOW, GO_FORWARD_LOW, LEFT_SHOT, RIGHT_SHOT, CLIMB_STAIRS, DESCEND_STAIRS, LIE_TO_STAND]
  🦾📷
LOOK(direction:str)
  direction must be one of [FORWARD, LEFT, RIGHT, UP, DOWN]
  👀📷"
export O_SUGGESTIONS="PLAY,GREET
LOOK,UP
MOVE,FORWARD"
export O_DEFAULT_FUNC="LOOK"
export O_DEFAULT_CODE="FORWARD"