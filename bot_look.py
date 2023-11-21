#!/usr/bin/env python3
# encoding: utf-8
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from ainex_kinematics.motion_manager import MotionManager
from util import timeit, EMOJIS


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
LOOKS = list(LOOK_DIRECTIONS.keys())

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


if __name__ == "__main__":
    import random

    for _ in range(3):
        print(look_at(random.choice(LOOKS)))
