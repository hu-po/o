import time

from datetime import datetime, timedelta
from filelock import Timeout, FileLock

BIRTHDAY: datetime = datetime.now()
LIFESPAN: timedelta = timedelta(minutes=4)  # How long the robot will live
MEMORY: int = 32  # How many characters worth of state to keep in memory
FORGET: int = 8  # How many characters worth of state to forget


MEMORY_LOCK_PATH = "/tmp/o.log.lock"
MEMORY_PATH = "/tmp/o.log.txt"

COMMAND_LOCK_PATH = "/tmp/o.command.lock"
COMMAND_PATH = "/tmp/o.command.txt"

# Series of images, more like an Image Directory with some depth
# Use time to prevent image locking without explicit need for lock
IMAGE_LOCK_PATH = 
IMAGE_PATH = 

def is_alive():

    # if len(state) >= MEMORY:
    #     for _ in range(FORGET):
    #         state.popleft()
    #     state.appendleft(f"{EMOJIS['forget']} memory erased")

    if datetime.now() - BIRTHDAY > LIFESPAN:
        return False
    return True

def timeit(f):
    def _(*args, **kwargs):
        start_time = time.time()
        print(f"----------- {f.__name__}")
        for key, value in kwargs.items():
            print(f"\t{key}={value}")
        result = f(*args, **kwargs)
        end_time = time.time()
        print(
            f"----------- {f.__name__} took {end_time - start_time:.2f}s{EMOJIS['time']}"
        )
        return result

    return _

def get_image():
    with FileLock("/tmp/o.image.lock"):
        with open("/tmp/o.image.png", "rb") as f:
            base64_image = base64.b64encode(f.read()).decode("utf-8")
    return base64_image

def get_log():
    with FileLock(LOG_LOCK_PATH):
        with open(LOG_PATH, "r") as f:
            log = f.read()
    return log

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

