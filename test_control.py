# test_control.py 

import time 
import json 
import numpy as np
from bus import FeetechBus
from utils import send_waypoints

with open('so101_motorbus_port.json') as f: 
    port = json.load(f)['port']

bus = FeetechBus(port, 
                 [1,2,3,4,5,6],
                 calib_file="so101_calibration.json")

try:
    target = np.array([0, 0, 0, 0, 0, 0], dtype=np.float32)
    current = bus.get_qpos()
    
    send_waypoints(bus, target, duration=2.5)

    time.sleep(0.8)

    send_waypoints(bus, current, duration=2.5)

    bus.set_torque(False)

finally:
    bus.disconnect()

