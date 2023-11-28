from filelock import Timeout, FileLock

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
