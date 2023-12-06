import argparse
from filelock import FileLock

import cv2

argparser = argparse.ArgumentParser()
argparser.add_argument("func", type=str)
argparser.add_argument("code", type=str)

FUNCTIONS = """
MOVE(direction:str)
  direction must be one of ["FORWARD", "BACKWARD", "LEFT", "RIGHT"]
  🦿📷
"""
SUGGESTIONS = """
MOVE,FORWARD
MOVE,LEFT
"""
DEFAULT_FUNC: str = "MOVE"
DEFAULT_CODE: str = "FORWARD"
VIDEO_DEVICE = 0
IMAGE_PATH = "/tmp/o.image.jpeg"  # Image is constantly overwritten
IMAGE_LOCK_PATH = "/tmp/o.image.lock "  # Lock prevents reading while writing


def capture_and_save_image() -> str:
    cap = cv2.VideoCapture(VIDEO_DEVICE)
    if not cap.isOpened():
        return "📷❌ cv2 error: no video device"
    ret, frame = cap.read()
    if not ret:
        cap.release()
        return "📷❌ cv2 error: on read"
    with FileLock(IMAGE_LOCK_PATH):
        cv2.imwrite(IMAGE_PATH, frame)
    cap.release()
    return "📷✅ new image"


def move(direction: str) -> str:
    log = capture_and_save_image()
    return f"🦿✅ moved {direction}{log}"


if __name__ == "__main__":
    args = argparser.parse_args()
    if args.func.upper() == "MOVE":
        print(move(args.code))
    else:
        raise ValueError(f"unknown func {args.func} code {args.code}")
