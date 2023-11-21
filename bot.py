#!/usr/bin/env python3
# encoding: utf-8
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from ainex_kinematics.gait_manager import GaitManager
from ainex_kinematics.motion_manager import MotionManager
from util import timeit, EMOJIS


DEFAULT_ACTION_NAME: str = "greet"
# Key-frame-y animations in a custom .d6a format
ACTIONS = [
    "hurdles",
    # 'turn_right',
    # 'go_turn_right_20',
    "left_hand_put_block",
    "lie_to_stand",
    "go_turn_left",
    "forward",
    "hand_back",
    "descend_stairs",
    "go_forward_low",
    "right_shot",
    "clamp_right",
    "clamp_left",
    "wave",
    "raise_right_hand",
    "crawl_right",
    "crawl_left",
    "twist",
    "recline_to_stand",
    "calib",
    # 'turn_right_30',
    # 'turn_left_30',
    "forward_one_step",
    "right_hand_put_block",
    "left_shot",
    # 'go_turn_left_20',
    "place_block",
    "stand_low",
    "stair_down",
    "climb_stairs",
    "back",
    "stand",
    # 'back_step',
    "walk_ready",
    "stair_up",
    # 'go_turn_left_low',
    "greet",
    "hand_open",
    # 'turn_left',
    "temp",
    "put_down",
    # 'go_turn_right',
    # 'move_left',
    # 'move_right',
    # 'move_up',
    # 'go_turn_right_low',
    # 'forward_step',
]
DEFAULT_MOVE_DIRECTION: str = "forward"
# These multipliers modify the x_amp, y_amp, and rotation_angle
MOVE_DIRECTIONS = {
    "FORWARD": [1, 0, 0],
    "BACKWARD": [-1, 0, 0],
    "LEFT": [0, 1, 0],
    "RIGHT": [0, -1, 0],
    "ROTATE_LEFT": [0, 0, 1],
    "ROTATE_RIGHT": [0, 0, -1],
}
# Speed selection has three levels: 1, 2, 3, and 4, with speed decreasing from fast to slow.
SPEED: int = 1  # Integer in range [1, 4] slow to fast
# Step stride in the x direction (meters).
X_AMPLITUDE: float = 0.02  # Highest I see in examples is 0.02
# Step stride in the y direction (meters).
Y_AMPLITUDE: float = 0.01  # Highest I see in examples is 0.01
# Rotation extent (degrees).
ROTATION_ANGLE: int = 8  # Highest I see in examples is 5, 8
# Arm swing extent (degrees), default is 30. When it is 0, no commands will be sent to the arms.
ARM_SWING_DEGREE: int = 30  # Highest I see in examples is 30
# Number of steps to take in each movement, default is 1.
STEP_NUM: int = 2  # I see numbers like 0 and 3 in the code
# Image is constantly overwritten
IMAGE_OUTPUT_FILENAME: str = "/tmp/image.jpg"
CAMERA_ROS_TOPIC: str = "/camera/image_rect_color"
# Servo 23 is the rotation/pan neck servo range [300, 600]
# Servo 24 is the tilt/vertical neck servo range [260, 650]
DEFAULT_LOOK_DIRECTION: str = "FORWARD"
LOOK_DIRECTIONS = {
    "FORWARD": [[23, 500], [24, 500]],
    "LEFT": [[23, 650], [24, 500]],
    "RIGHT": [[23, 350], [24, 500]],
    "UP": [[23, 500], [24, 650]],
    "DOWN": [[23, 500], [24, 400]],
}


def image_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    cv2.imwrite(IMAGE_OUTPUT_FILENAME, cv_image)
    rospy.signal_shutdown("Image saved")


def save_one_image(rostopic: str = CAMERA_ROS_TOPIC):
    rospy.init_node("save_one_image")
    rospy.Subscriber(rostopic, Image, image_callback)
    rospy.spin()


@timeit
def look_at(
    direction: str = DEFAULT_LOOK_DIRECTION,
    directions: list = LOOK_DIRECTIONS,
) -> str:
    motion_manager = MotionManager()
    servo_pos = directions.get(direction, None)
    if servo_pos:
        motion_manager.set_servos_position(500, servo_pos)
        rospy.sleep(0.2)
        save_one_image()
        return (
            f"{EMOJIS['robot']}{EMOJIS['look']}{EMOJIS['success']} looked {direction}"
        )
    else:
        return f"{EMOJIS['robot']}{EMOJIS['look']}{EMOJIS['fail']} could not look in unknown direction {direction}"


@timeit
def move(
    direction: str = DEFAULT_MOVE_DIRECTION,
    directions: list = MOVE_DIRECTIONS,
    speed: int = SPEED,
    x_amplitude: float = X_AMPLITUDE,
    y_amplitude: float = Y_AMPLITUDE,
    rotation_angle: int = ROTATION_ANGLE,
    arm_swing_degree: int = ARM_SWING_DEGREE,
    step_num: int = STEP_NUM,
) -> str:
    rospy.init_node("simple_gait_control_demo")
    gait_manager = GaitManager()
    modifiers = directions.get(direction, None)
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
        return f"{EMOJIS['robot']}{EMOJIS['move']}{EMOJIS['success']} moved {direction}"
    else:
        return f"{EMOJIS['robot']}{EMOJIS['move']}{EMOJIS['fail']} could not move in unknown direction {direction}"


@timeit
def perform(
    action: str = DEFAULT_ACTION_NAME,
    actions: list = ACTIONS,
) -> str:
    motion_manager = MotionManager()
    if action in actions:
        motion_manager.run_action(action)
        return f"{EMOJIS['robot']}{EMOJIS['perform']}{EMOJIS['success']} performed {action}"
    else:
        return f"{EMOJIS['robot']}{EMOJIS['perform']}{EMOJIS['fail']} could not perform unknown action {action}"


if __name__ == "__main__":
    import random

    while True:
        print(perform(random.choice(ACTIONS)))
        print(move(random.choice(MOVE_DIRECTIONS.keys())))
        print(look_at(random.choice(LOOK_DIRECTIONS.keys())))
