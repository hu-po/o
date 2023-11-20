#!/usr/bin/env python3
# encoding: utf-8
# import time
import argparse
from ainex_kinematics.motion_manager import MotionManager

argparser = argparse.ArgumentParser()
argparser.add_argument('--command', type=str, required=True)
args = argparser.parse_args()

def perform(command: str) -> str:
    print(f"Performing action: {command}")
    motion_manager = MotionManager('/home/ubuntu/software/ainex_controller/ActionGroups')
    print("MotionManager initialized.")
    assert command in [
        'left_shot',
        'right_shot',
        'stand',
        'walk_ready',
        'twist',
        'three',
        'four',
        'hand_back',
        'greet',
    ], f"Unknown action: {command}"
    motion_manager.run_action(command)
    print(f"Action {command} completed.")

if __name__ == '__main__':
    perform(args.command)