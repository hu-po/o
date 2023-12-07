#!/usr/bin/env python3
# encoding: utf-8
import argparse
import os
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from filelock import FileLock

from ainex_kinematics.gait_manager import GaitManager
from ainex_kinematics.motion_manager import MotionManager

argparser = argparse.ArgumentParser()
argparser.add_argument("func", type=str)
argparser.add_argument("code", type=str)

# Servo 23 is the rotation/pan neck servo range [300, 600]
# Servo 24 is the tilt/vertical neck servo range [260, 650]
LOOK_DIRECTIONS: dict = {
    "FORWARD": [[23, 500], [24, 500]],
    "LEFT": [[23, 650], [24, 500]],
    "RIGHT": [[23, 350], [24, 500]],
    "UP": [[23, 500], [24, 650]],
    "DOWN": [[23, 500], [24, 400]],
}
DEFAULT_LOOK_DIRECTION: str = "FORWARD"
IMAGE_SLEEP: float = 0.1  # Sleep while saving image to preven motion blur
IMAGE_PATH = "/tmp/o.image.jpeg"  # Image is constantly overwritten
IMAGE_LOCK_PATH = "/tmp/o.image.lock " # Lock prevents reading while writing
CAMERA_ROS_TOPIC: str = "/camera/image_rect_color"


# Key-frame-y animations in a custom .d6a format
ACTION_NAMES: dict = {
    "ARM_WAVE_GREET": "greet",
    "WAGGLE_DANCE": "wave",
    "GRAB_LEFT_HAND": "left_hand_put_block",
    "GRAB_RIGHT_HAND": "right_hand_put_block",
    "CROUCH": "stand_low",
    "CROUCH_FORWARD": "go_forward_low",
    "KICK_LEFT_FOOT": "left_shot",
    "KICK_RIGHT_FOOT": "right_shot",
    "CLIMB_UP": "climb_stairs",
    "CLIMB_DOWN": "descend_stairs",
    "GET_UP_FROM_GROUND": "lie_to_stand",
}
DEFAULT_ACTION_NAME: str = "GREET"

# These multipliers modify the x_amp, y_amp, and rotation_angle
MOVE_DIRECTIONS = {
    "FORWARD": [1, 0, 0],
    "BACKWARD": [-1, 0, 0],
    "SHUFFLE_LEFT": [0, 1, 0],
    "SHUFFLE_RIGHT": [0, -1, 0],
    "ROTATE_LEFT": [0, 0, 1],
    "ROTATE_RIGHT": [0, 0, -1],
}
DEFAULT_MOVE_DIRECTION: str = "FORWARD"
O_NEX_MOVE_ENABLED: bool = os.getenv("O_NEX_MOVE_ENABLED", 1)
# Speed selection has three levels: 1, 2, 3, and 4, with speed decreasing from fast to slow.
SPEED: int = 3  # Integer in range [1, 4] slow to fast
# Step stride in the x direction (meters).
X_AMPLITUDE: float = 0.02  # Highest I see in examples is 0.02
# Step stride in the y direction (meters).
Y_AMPLITUDE: float = 0.01  # Highest I see in examples is 0.01
# Rotation extent (degrees).
ROTATION_ANGLE: int = 12  # Highest I see in examples is 5, 8
# Arm swing extent (degrees), default is 30. When it is 0, no commands will be sent to the arms.
ARM_SWING_DEGREE: int = 24  # Highest I see in examples is 30
# Number of steps to take in each movement, default is 1.
STEP_NUM: int = 4  # I see numbers like 0 and 3 in the code

# These are used inside prompts, so make them llm friendly
DESCRIPTION = """
You are a small humanoid robot with a monocular camera
You are small and only 20cm off the ground
"""
FUNCTIONS = f"""
MOVE(direction:str)
  direction must be one of [{','.join(MOVE_DIRECTIONS.keys())}]
  🦿📷
PLAY(action:str)
  action must be one of [{','.join(ACTION_NAMES.keys())}]
  🦾📷
LOOK(direction:str)
  direction must be one of [{','.join(LOOK_DIRECTIONS.keys())}]
  👀📷
"""
SUGGESTIONS = """
PLAY,GREET
LOOK,UP
MOVE,FORWARD
"""
DEFAULT_FUNC: str = "LOOK"
DEFAULT_CODE: str = "FORWARD"


def image_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    with FileLock(IMAGE_LOCK_PATH):
        cv2.imwrite(IMAGE_PATH, cv_image)
    rospy.signal_shutdown("Image saved")


def save_one_image():
    rospy.sleep(IMAGE_SLEEP)
    rospy.init_node("save_one_image")
    rospy.Subscriber(CAMERA_ROS_TOPIC, Image, image_callback)
    rospy.spin()


def move(
    direction: str = DEFAULT_MOVE_DIRECTION,
    directions: dict = MOVE_DIRECTIONS,
    speed: int = SPEED,
    x_amplitude: float = X_AMPLITUDE,
    y_amplitude: float = Y_AMPLITUDE,
    rotation_angle: int = ROTATION_ANGLE,
    arm_swing_degree: int = ARM_SWING_DEGREE,
    step_num: int = STEP_NUM,
) -> str:
    if not O_NEX_MOVE_ENABLED:
        return "🦿❌ move is disabled"
    rospy.init_node("simple_gait_control_demo")
    gait_manager = GaitManager()
    modifiers = directions.get(direction.upper(), None)
    if modifiers:
        x_amplitude *= modifiers[0]
        y_amplitude *= modifiers[1]
        rotation_angle *= modifiers[2]
        gait_manager.move(
            speed,
            x_amplitude,
            y_amplitude,
            rotation_angle,
            arm_swing_degree,
            step_num=step_num,
        )
        gait_manager.stop()
        save_one_image()
        return f"🦿✅ moved {direction}"
    else:
        return f"🦿❌ unknown move direction {direction}"


def play(
    action: str = DEFAULT_ACTION_NAME,
    actions: dict = ACTION_NAMES,
) -> str:
    motion_manager = MotionManager()
    action = actions.get(action.upper(), None)
    if action:
        motion_manager.run_action(action)
        save_one_image()
        return f"🦾✅ performed {action}"
    else:
        return f"🦾❌ unknown action {action}"


def look(
    direction: str = DEFAULT_LOOK_DIRECTION,
    directions: list = LOOK_DIRECTIONS,
) -> str:
    motion_manager = MotionManager()
    servo_pos = directions.get(direction.upper(), None)
    if servo_pos:
        motion_manager.set_servos_position(500, servo_pos)
        save_one_image()
        return f"👀✅ looked {direction}"
    else:
        return f"👀❌ unknown look direction {direction}"


if __name__ == "__main__":
    args = argparser.parse_args()
    if args.func.upper() == "LOOK":
        print(look(direction=args.code))
    elif args.func.upper() == "PLAY":
        print(play(action=args.code))
    elif args.func.upper() == "MOVE":
        print(move(direction=args.code))
    else:
        raise ValueError(f" unknown func {args.func} code {args.code}")
