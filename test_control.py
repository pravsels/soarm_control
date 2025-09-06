# test_control.py 

import time 
import json 
import os
import numpy as np
from bus import FeetechBus
from config import UIDS, GEAR_RATIOS

# --- Ask user which device to control ---
while True:
    device_name = input(
        "Testing which device? "
        "(so101_leader / so101_follower): "
    ).strip()
    if device_name in {"so100_leader", "so100_follower", "so101_leader", "so101_follower"}:
        break
    print("Invalid choice. Allowed options: so100_leader, so100_follower, so101_leader, so101_follower.")

port_file = f"{device_name}_motorbus_port.json"
calib_file = f"{device_name}_calibration.json"

# --- Check motorbus port config ---
if not os.path.exists(port_file):
    raise FileNotFoundError(
        f"Port config not found: {port_file}. Run the port detection script first."
    )
with open(port_file, "r") as f: 
    port_config = json.load(f)
if "port" not in port_config:
    raise ValueError(f"Invalid config in {port_file}: missing 'port'.")
port = port_config["port"]

# --- Check calibration file ---
if not os.path.exists(calib_file):
    raise FileNotFoundError(
        f"Calibration file not found: {calib_file}. Run calibrate.py first."
    )

# --- Connect to bus ---
bus = FeetechBus(port, UIDS, 
                 calib_file=calib_file)

def move_to(qpos_target, tol=1e-2, timeout=3.0, step_size=0.1):
    t0 = time.time()

    while True:
        qpos_current = bus.get_qpos()
        delta = qpos_target - qpos_current
        if np.all(np.abs(delta) < tol):
            print('going outta loop since tol reached!')
            break
        if time.time() - t0 > timeout:
            print("timeout, exiting move loop")
            break
        delta_clipped = np.clip(delta, -step_size, step_size)
        qpos_next = qpos_current + delta_clipped
        bus.set_qpos(qpos_next)
        time.sleep(0.02)  # 50 Hz

try:
    target = np.array([0, 0, 0, 0, 0, 0], dtype=np.float32)
    current = bus.get_qpos()
    
    move_to(target)

    time.sleep(0.8)

    move_to(current)

finally:
    bus.set_torque(False)

    bus.disconnect()

