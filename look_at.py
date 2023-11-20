#!/usr/bin/env python3
# encoding: utf-8
# import time
import argparse
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from ainex_kinematics.motion_manager import MotionManager

argparser = argparse.ArgumentParser()
argparser.add_argument("--command", type=str, required=True)
args = argparser.parse_args()

# Image is constantly overwritten
IMAGE_OUTPUT_FILENAME: str = "/tmp/image.jpg"

# Servo 23 is the rotation/pan neck servo range [300, 600]
# Servo 24 is the tilt/vertical neck servo range [260, 650]
FORWARD_SERVO_POSITIONS = [[23, 500], [24, 500]]
LEFT_SERVO_POSITIONS = [[23, 650], [24, 500]]
RIGHT_SERVO_POSITIONS = [[23, 350], [24, 500]]
UP_SERVO_POSITIONS = [[23, 500], [24, 650]]
DOWN_SERVO_POSITIONS = [[23, 500], [24, 400]]


def image_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    cv2.imwrite(IMAGE_OUTPUT_FILENAME, cv_image)
    rospy.signal_shutdown("Image saved")


def save_one_image():
    rospy.init_node("save_one_image")
    rospy.Subscriber("/camera/image_rect_color", Image, image_callback)
    rospy.spin()


def look_at(command: str) -> str:
    print(f"Look_at {command}")
    motion_manager = MotionManager(
        "/home/ubuntu/software/ainex_controller/ActionGroups"
    )
    print("MotionManager initialized.")
    assert command in [
        "forward",
        "left",
        "right",
        "up",
        "down",
    ], f"Unknown look_at command: {command}"
    if command == "forward":
        servo_pos = FORWARD_SERVO_POSITIONS
    elif command == "left":
        servo_pos = LEFT_SERVO_POSITIONS
    elif command == "right":
        servo_pos = RIGHT_SERVO_POSITIONS
    elif command == "up":
        servo_pos = UP_SERVO_POSITIONS
    elif command == "down":
        servo_pos = DOWN_SERVO_POSITIONS
    motion_manager.set_servos_position(500, servo_pos)
    print(f"Look_at {command} completed.")
    rospy.sleep(0.2)
    save_one_image()
    print("Image saved.")


if __name__ == "__main__":
    look_at(args.command)
