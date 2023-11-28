from datetime import datetime, timedelta
from filelock import Timeout, FileLock

BIRTHDAY: datetime = datetime.now()
LIFESPAN: timedelta = timedelta(minutes=4)  # How long the robot will live
MEMORY: int = 32  # How many characters worth of state to keep in memory
FORGET: int = 8  # How many characters worth of state to forget

lock_path = "my_lock.lock"
shared_file_path = "shared_data.txt"

# To write data
with FileLock(lock_path):
    with open(shared_file_path, "w") as file:
        file.write("Some data")

# To read data
with FileLock(lock_path):
    with open(shared_file_path, "r") as file:
        data = file.read()


def read_log():
    pass

def write_log():
    pass

def is_alive():
    pass