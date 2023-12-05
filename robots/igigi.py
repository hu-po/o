import argparse
import time
from dataclasses import dataclass

from .test import capture_and_save_image
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

FUNCTIONS = """
LOOK(direction:str)
direction must be one of ["FORWARD", "LEFT", "RIGHT", "UP", "DOWN"]
"""
SUGGESTIONS = """
LOOK,LEFT
LOOK,UP
LOOK,FORWARD
"""
DEFAULT_FUNC: str = "LOOK"
DEFAULT_CODE: str = "FORWARD"
LOOK_DIRECTIONS = {
    "FORWARD": [0, 0, 0],
    "LEFT": [0, 0, 0],
    "RIGHT": [0, 0, 0],
    "UP": [0, 0, 0],
    "DOWN": [0, 0, 0],
}


@dataclass
class Servo:
    id: int  # dynamixel id for servo
    range: [int, int]  # (min, max) position values for servos (0, 4095)
    desc: str  # description of servo for llm use


SERVOS = {
    "roll": Servo(1, (1761, 2499), "rolls the neck left and right, roll"),
    "tilt": Servo(2, (979, 2223), "tilts the head up and down vertically, pitch"),
    "pan": Servo(3, (988, 3007), "pans the head side to side horizontally, yaw"),
}


@dataclass
class Pose:
    angles: list  # list of int angles in degrees (0, 360) describing static pose
    desc: str  # description of static pose for llm use


set_servo_speed: int = 64  # degrees per move duration
set_servo_timeout: float = 3  # seconds
set_servo_sleep: float = 0.01  # seconds

# Raw servo parameters
protocol_version: float = 2.0
baudrate: int = 57600
device_name: str = "/dev/ttyUSB0"
addr_torque_enable: int = 64
addr_goal_position: int = 116
addr_present_position: int = 132
torque_enable: int = 1
torque_disable: int = 0


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
        servos: dict = SERVOS,
        protocol_version: float = 2.0,
        baudrate: int = 57600,
        device_name: str = "/dev/ttyUSB0",
        addr_torque_enable: int = 64,
        addr_goal_position: int = 116,
        addr_present_position: int = 132,
        torque_enable: int = 1,
        torque_disable: int = 0,
    ):
        self.servos: list = []
        for name, servo in servos.items():
            print(f"---- Initialize servo {name} ----")
            print(f"id: {servo.id}")
            print(f"range: {servo.range}")
            print(f"description: {servo.desc}")
            self.servos.append(servo)
        self.num_servos: int = len(self.servos)  # Number of servos to control

        # Dynamixel communication parameters
        self.protocol_version = (
            protocol_version  # DYNAMIXEL Protocol version (1.0 or 2.0)
        )
        self.baudrate = baudrate  # Baudrate for DYNAMIXEL communication
        self.device_name = (
            device_name  # Name of the device (port) where DYNAMIXELs are connected
        )
        self.addr_torque_enable = (
            addr_torque_enable  # Address for Torque Enable control table in DYNAMIXEL
        )
        self.addr_goal_position = (
            addr_goal_position  # Address for Goal Position control table in DYNAMIXEL
        )
        self.addr_present_position = addr_present_position  # Address for Present Position control table in DYNAMIXEL
        self.torque_enable = torque_enable  # Value to enable the torque
        self.torque_disable = torque_disable  # Value to disable the torque

        # Initialize DYNAMIXEL communication
        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(self.protocol_version)
        if not self.port_handler.openPort():
            print("Failed to open the port")
            exit()
        if not self.port_handler.setBaudRate(self.baudrate):
            print("Failed to change the baudrate")
            exit()
        self.group_bulk_write = GroupBulkWrite(self.port_handler, self.packet_handler)
        self.group_bulk_read = GroupBulkRead(self.port_handler, self.packet_handler)

    def _write_position(self, positions: list) -> str:
        msg: str = ""
        # Enable torque for all servos and add goal position to the bulk write parameter storage
        for i, pos in enumerate(positions):
            pos = degrees_to_units(pos)
            dxl_id = self.servos[i].id
            clipped = min(max(pos, self.servos[i].range[0]), self.servos[i].range[1])

            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, dxl_id, self.addr_torque_enable, self.torque_enable
            )
            if dxl_comm_result != COMM_SUCCESS:
                msg += f"ERROR: {self.packet_handler.getTxRxResult(dxl_comm_result)}"
                raise Exception(msg)
            elif dxl_error != 0:
                msg += f"ERROR: {self.packet_handler.getRxPacketError(dxl_error)}"
                raise Exception(msg)

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

        # Write goal position
        dxl_comm_result = self.group_bulk_write.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            msg += f"ERROR: {self.packet_handler.getTxRxResult(dxl_comm_result)}"
            raise Exception(msg)

        # Clear bulk write parameter storage
        self.group_bulk_write.clearParam()

    def _read_pos(self) -> list:
        msg: str = ""
        # Add present position value to the bulk read parameter storage
        for i in range(self.num_servos):
            dxl_id = self.servos[i].id
            dxl_addparam_result = self.group_bulk_read.addParam(
                dxl_id, self.addr_present_position, 4
            )
            if not dxl_addparam_result:
                msg += f"ERROR: [ID:{dxl_id}] groupBulkRead addparam failed\n"
                raise Exception(msg)

        # Read present position
        dxl_comm_result = self.group_bulk_read.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            msg += f"ERROR: {self.packet_handler.getTxRxResult(dxl_comm_result)}\n"
            raise Exception(msg)

        # Get present position value
        positions = []
        for i in range(self.num_servos):
            dxl_id = self.servos[i].id
            dxl_present_position = self.group_bulk_read.getData(
                dxl_id, self.addr_present_position, 4
            )
            positions.append(units_to_degrees(dxl_present_position))

        # Clear bulk read parameter storage
        self.group_bulk_read.clearParam()

        return positions

    def _disable_torque(self) -> None:
        for servo in self.servos:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler,
                servo.id,
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
