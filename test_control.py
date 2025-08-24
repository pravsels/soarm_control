
import time 
import json 
import numpy as np
from bus import FeetechBus

with open('so101_motorbus_port.json') as f: 
    port = json.load(f)['port']

bus = FeetechBus(port, 
                 [1,2,3,4,5,6],
                 calib_file="so101_calibration.json")

def joint_space_waypoints(current, target, steps=100):
    s = np.linspace(0., 1., steps, dtype=np.float32)
    # current is repeated STEPS times and the differnce (target - current)
    # is repeated STEPS times and each time it's scaled by 
    # an element of s, taking small steps towards the target. 
    return current + (target - current) * s[:,  None]

def send_waypoints(current, target, duration, verbose=False):
    waypoints = joint_space_waypoints(current, target, 500)
    delay_per_step = duration / len(waypoints)
    for i, intermediate_waypoint in enumerate(waypoints):
        bus.set_qpos(intermediate_waypoint)
        time.sleep(delay_per_step)  # small delay between steps
        if verbose:
            print(f"Step {i}/{len(waypoints)}")

try:
    target = np.array([0, 0, 0, 0, 0, 0], dtype=np.float32)
    current = bus.get_qpos()
    
    send_waypoints(current, target, duration=2.5)

    time.sleep(0.8)

    send_waypoints(target, current, duration=2.5)

    bus.set_torque(False)

finally:
    bus.disconnect()

