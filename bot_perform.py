#!/usr/bin/env python3
# encoding: utf-8
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

    for _ in range(3):
        print(perform(random.choice(ACTIONS)))
