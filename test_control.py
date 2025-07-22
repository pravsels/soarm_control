
import time 
import json 
import numpy as np
from bus import FeetechBus

with open('so101_motorbus_port.json') as f: 
    port = json.load(f)['port']

bus = FeetechBus(port, 
                 [1,2,3,4,5,6],
                 calib_file="so101_calibration.json")

def send_waypoints(current, target):
    steps = 20  # number of intermediate steps
    for i in range(1, steps + 1):
        intermediate = current + (target - current) * (i / steps)
        bus.set_qpos(intermediate)
        time.sleep(0.099)  # small delay between steps
        print(f"Step {i}/{steps}")

try:
    print("read qpos:", bus.get_qpos())        # should be near 0s
    target = np.array([0.5, 0, 0, 0, 0, 0], dtype=np.float32)
    current = bus.get_qpos()
    
    send_waypoints(current, target)

    time.sleep(2)

    send_waypoints(target, current)

finally:
    bus.disconnect()

