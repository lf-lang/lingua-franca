import ray
from filelock import FileLock
import os
import tempfile

# The code is modeled after the file sync example at https://docs.ray.io/en/latest/advanced.html#inter-process-synchronization-using-filelock
# The purpose of the code is to demo Ray's ability to use a file lock for synchronization.
# However, this example shows non-deterministic results across multiple runs.

''' The original code from the example:
import ray
from filelock import FileLock

@ray.remote
def write_to_file(text):
    # Create a filelock object. Consider using an absolute tmpfile for the lock.
    with FileLock("my_data.txt.lock"):
        with open("my_data.txt","a") as f:
            f.write(text)

ray.init()
ray.get([write_to_file.remote("hi there!\n") for i in range(3)])

with open("my_data.txt") as f:
    print(f.read())
'''

def main(tmpfile, lock):
    @ray.remote
    def write_to_file(text):
        with FileLock(lock):
            with open(tmpfile, "a") as f:
                f.write(text)
    ray.init()
    ray.get([write_to_file.remote(f"hi there {i}!\n") for i in range(3)])
    with open(tmpfile) as f:
        print(f.read())

_, tmpfile = tempfile.mkstemp()
_, lock = tempfile.mkstemp()

try:
    main(tmpfile, lock)
finally:
    os.remove(tmpfile)
    os.remove(lock)
