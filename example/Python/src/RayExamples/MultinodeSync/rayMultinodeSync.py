import asyncio
import ray
import time

# This example is adapted from https://docs.ray.io/en/latest/advanced.html#multi-node-synchronization-using-an-actor
# The purpose of the code is to demo Ray's ability to do multinode synchronization
# However, this example shows non-deterministic results across multiple runs.

ray.init()

# We set num_cpus to zero because this actor will mostly just block on I/O.
@ray.remote(num_cpus=0)
class SignalActor:
    def __init__(self):
        self.ready_event = asyncio.Event()

    def send(self, clear=False):
        self.ready_event.set()
        if clear:
            self.ready_event.clear()

    async def wait(self, should_wait=True):
        if should_wait:
            await self.ready_event.wait()

@ray.remote
def wait_and_go(signal_and_i):
    signal, i = signal_and_i
    print(f"{i}: waiting...")
    ray.get(signal.wait.remote())
    print(f"{i}: go!")


signal = SignalActor.remote()
tasks = [wait_and_go.remote((signal, i)) for i in range(4)]

# Delay between wait and go signal
time.sleep(2)

print("ready...")
print("set..")

# Send go signals
ray.get(signal.send.remote())

# Tasks are unblocked.
ray.get(tasks)