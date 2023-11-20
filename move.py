#!/usr/bin/env python3
# encoding: utf-8
import argparse
import rospy
from ainex_kinematics.gait_manager import GaitManager

argparser = argparse.ArgumentParser()
argparser.add_argument("--command", type=str, required=True)
args = argparser.parse_args()

# param step_velocity: Speed selection has three levels: 1, 2, 3, and 4, with speed decreasing from fast to slow.
SPEED: int = 1  # Integer in range [1, 4] slow to fast
# param x_amplitude: Step stride in the x direction (meters).
X_AMPLITUDE: float = 0.02  # Highest I see in examples is 0.02
# param y_amplitude: Step stride in the y direction (meters).
Y_AMPLITUDE: float = 0.01  # Highest I see in examples is 0.01
# param rotation_angle: Rotation extent (degrees).
ROTATION_ANGLE: int = 8  # Highest I see in examples is 5, 8
# param arm_swap: Arm swing extent (degrees), default is 30. When it is 0, no commands will be sent to the arms.
ARM_SWING_DEGREE: int = 30  # Highest I see in examples is 30
# param step_num: Number of steps to take, default is 1.
STEP_NUM: int = 2  # I see numbers like 0 and 3 in the code


def move(
    command: str,
    speed: int = SPEED,
    x_amplitude: float = X_AMPLITUDE,
    y_amplitude: float = Y_AMPLITUDE,
    rotation_angle: int = ROTATION_ANGLE,
    arm_swing_degree: int = ARM_SWING_DEGREE,
    step_num: int = STEP_NUM,
) -> str:
    rospy.init_node("simple_gait_control_demo")
    gait_manager = GaitManager()
    # rospy.sleep(0.2)
    print("GaitManager initialized.")
    assert command in [
        "forward",
        "backward",
        "left",
        "right",
        "rotate left",
        "rotate right",
    ], f"Unknown move name: {command}"
    if command == "forward":
        y_amplitude = 0.0
        rotation_angle = 0
    elif command == "backward":
        x_amplitude = -x_amplitude
        y_amplitude = 0.0
        rotation_angle = 0
    elif command == "left":
        x_amplitude = 0.0
        rotation_angle = 0
    elif command == "right":
        x_amplitude = 0.0
        y_amplitude = -y_amplitude
        rotation_angle = 0
    elif command == "rotate left":
        x_amplitude = 0.0
        y_amplitude = 0.0
    elif command == "rotate right":
        x_amplitude = 0.0
        y_amplitude = 0.0
        rotation_angle = -rotation_angle
    print(f"speed set to {speed}")
    print(f"x_amplitude set to {x_amplitude}")
    print(f"y_amplitude set to {y_amplitude}")
    print(f"rotation_angle set to {rotation_angle}")
    print(f"Number of steps {step_num}")
    print(f"Moving {command}...")
    gait_manager.move(
        speed,
        x_amplitude,
        y_amplitude,
        rotation_angle,
        arm_swing_degree,
        step_num=step_num,
    )
    print("Movement completed.")
    # rospy.sleep(0.2)
    print("Stopping GaitManager...")
    gait_manager.stop()
    print("GaitManager stopped.")


if __name__ == "__main__":
    move(args.command)
