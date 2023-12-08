import argparse
from filelock import FileLock
import os
import time

import cv2
from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    GroupBulkWrite,
    GroupBulkRead,
    COMM_SUCCESS,
    DXL_LOBYTE,
    DXL_LOWORD,
    DXL_HIBYTE,
    DXL_HIWORD,
)

argparser = argparse.ArgumentParser()
argparser.add_argument("func", type=str)
argparser.add_argument("code", type=str)

DESCRIPTION = os.getenv("O_DESCRIPTION", """
You are a small robot with a stereo camera vision module
Your vision module uses two cameras to infer 3d
""")
FUNCTIONS = os.getenv("O_FUNCTIONS", """
LOOK(direction:str)
    direction must be one of ["FORWARD", "LEFT", "RIGHT"]
    📷👀
""")
SUGGESTIONS = os.getenv("O_SUGGESTIONS", """
LOOK,LEFT
LOOK,FORWARD
""")
DEFAULT_FUNC: str = os.getenv("O_DEFAULT_FUNC", "LOOK")
DEFAULT_CODE: str = os.getenv("O_DEFAULT_CODE", "FORWARD")

VIDEO_DEVICE = str(os.getenv("O_VIDEO_DEVICE", "/dev/video0"))
IMAGE_PATH = os.getenv("O_IMAGE_PATH", "/tmp/o.image.jpeg")  # Image is constantly overwritten
IMAGE_LOCK_PATH = os.getenv("O_IMAGE_LOCK_PATH", "/tmp/o.image.lock")  # Lock prevents reading while writing

LOOK_DIRECTIONS = {
    "FORWARD": [3000, 2850, 1800],
    "LEFT": [3000, 2850, 1640],
    "RIGHT": [3000, 2850, 2000],
}
SERVOS: list = [
    (1, (2825, 3536)),  # Servo ID 1
    (2, (2827, 2850)),  # Servo ID 2
    (3, (1648, 2333)),  # Servo ID 3
]

# Convert servo units into degrees for readability
# Max for units is 4095, which is 360 degrees
DEGREE_TO_UNIT: float = 4095 / 360.0


def degrees_to_units(degree: int) -> int:
    return int(degree * DEGREE_TO_UNIT)


def units_to_degrees(position: int) -> int:
    return int(position / DEGREE_TO_UNIT)


class Servos:
    def __init__(
        self,
        servos: list = SERVOS,
        protocol_version: float = 2.0,  # DYNAMIXEL Protocol version (1.0 or 2.0)
        baudrate: int = 57600,  # Baudrate for DYNAMIXEL communication
        device_name: str = "/dev/ttyUSB0",  # Name of the device (port) where DYNAMIXELs are connected
        addr_torque_enable: int = 64,  # Address for Torque Enable control table in DYNAMIXEL
        addr_goal_position: int = 116,  # Address for Goal Position control table in DYNAMIXEL
        addr_present_position: int = 132,  # Address for Present Position control table in DYNAMIXEL
        torque_enable: int = 1,  # Value to enable the torque
        torque_disable: int = 0,  # Value to disable the torque
    ):
        self.num_servos: int = len(servos)
        self.servo_ids: list = []
        self.servo_ranges: list = []
        for id, range in servos:
            self.servo_ids.append(id)
            self.servo_ranges.append(range)
        self.protocol_version = protocol_version
        self.baudrate = baudrate
        self.device_name = device_name
        self.addr_torque_enable = addr_torque_enable
        self.addr_goal_position = addr_goal_position
        self.addr_present_position = addr_present_position
        self.torque_enable = torque_enable
        self.torque_disable = torque_disable
        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(self.protocol_version)
        if not self.port_handler.openPort():
            raise Exception("Failed to open the port")
        if not self.port_handler.setBaudRate(self.baudrate):
            raise Exception("Failed to change the baudrate")
        self.group_bulk_write = GroupBulkWrite(self.port_handler, self.packet_handler)
        self.group_bulk_read = GroupBulkRead(self.port_handler, self.packet_handler)

    def _write_position(self, positions: list):
        for i, pos in enumerate(positions):
            dxl_id = self.servo_ids[i]
            clipped = min(max(pos, self.servo_ranges[i][0]), self.servo_ranges[i][1])
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, dxl_id, self.addr_torque_enable, self.torque_enable
            )
            if dxl_comm_result != COMM_SUCCESS:
                raise Exception(self.packet_handler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                raise Exception(self.packet_handler.getRxPacketError(dxl_error))
            self.group_bulk_write.addParam(
                dxl_id,
                self.addr_goal_position,
                4,
                [
                    DXL_LOBYTE(DXL_LOWORD(clipped)),
                    DXL_HIBYTE(DXL_LOWORD(clipped)),
                    DXL_LOBYTE(DXL_HIWORD(clipped)),
                    DXL_HIBYTE(DXL_HIWORD(clipped)),
                ],
            )
        dxl_comm_result = self.group_bulk_write.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            raise Exception(self.packet_handler.getTxRxResult(dxl_comm_result))
        self.group_bulk_write.clearParam()

    def _read_pos(self) -> list:
        for i in range(self.num_servos):
            dxl_id = self.servo_ids[i]
            dxl_addparam_result = self.group_bulk_read.addParam(
                dxl_id, self.addr_present_position, 4
            )
            if not dxl_addparam_result:
                raise Exception(f"[ID:{dxl_id}] groupBulkRead addparam failed")
        # Read present position
        dxl_comm_result = self.group_bulk_read.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            raise Exception(self.packet_handler.getTxRxResult(dxl_comm_result))
        # Get present position value
        positions = []
        for i in range(self.num_servos):
            dxl_id = self.servo_ids[i]
            dxl_present_position = self.group_bulk_read.getData(
                dxl_id, self.addr_present_position, 4
            )
            positions.append(units_to_degrees(dxl_present_position))
        # Clear bulk read parameter storage
        self.group_bulk_read.clearParam()
        return positions

    def _disable_torque(self) -> None:
        for id in self.servo_ids:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler,
                id,
                self.addr_torque_enable,
                self.torque_disable,
            )
            if dxl_comm_result != COMM_SUCCESS:
                print(f"ERROR: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                print(f"ERROR: {self.packet_handler.getRxPacketError(dxl_error)}")

    def __del__(self, *args, **kwargs) -> None:
        self._disable_torque()
        self.port_handler.closePort()

def capture_and_save_image() -> str:
    cap = cv2.VideoCapture(VIDEO_DEVICE)
    if not cap.isOpened():
        return "📷❌ cv2 error: no video device"
    ret, frame = cap.read()
    if not ret:
        cap.release()
        return "📷❌ cv2 error: on read"
    frame = cv2.flip(frame, 0)  # Flip the image vertically
    with FileLock(IMAGE_LOCK_PATH):
        cv2.imwrite(IMAGE_PATH, frame)
    cap.release()
    return "📷✅ new image"


def look(
    direction: str = DEFAULT_CODE,
    directions: list = LOOK_DIRECTIONS,
) -> str:
    servos = Servos()
    servo_pos = directions.get(direction.upper(), None)
    if servo_pos:
        servos._write_position(servo_pos)
        log = capture_and_save_image()
        return f"{log}👀✅ looked {direction}"
    else:
        return f"👀❌ unknown look direction {direction}"


def test_mode() -> None:
    print("Entering limp mode")
    servos = Servos()
    servos._disable_torque()
    for _ in range(10):
        time.sleep(0.5)
        print(servos._read_pos())


if __name__ == "__main__":
    args = argparser.parse_args()
    if args.func.upper() == "LOOK":
        print(look(args.code))
    elif args.func.upper() == "TEST":
        test_mode()
    else:
        raise ValueError(f"unknown func {args.func} code {args.code}")
