#!/usr/bin/env python3
# encoding: utf-8
import rospy

from ainex_kinematics.gait_manager import GaitManager
from util import timeit, EMOJIS

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
        rospy.signal_shutdown("Movement complete")
        return f"{EMOJIS['robot']}{EMOJIS['move']}{EMOJIS['success']} moved {direction}"
    else:
        return f"{EMOJIS['robot']}{EMOJIS['move']}{EMOJIS['fail']} could not move in unknown direction {direction}"


if __name__ == "__main__":
    import random

    for _ in range(3):
        print(move(random.choice(list(MOVE_DIRECTIONS.keys()))))

