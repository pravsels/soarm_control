
import time 
import json 
import numpy as np
from bus import FeetechBus

with open('so101_calibration.json') as f: 
    calib = json.load(f)

def discretize_state(joint_angle, num_bins=10):

    bin_size = (max_range - min_range) / num_bins 

    bin_index = (joint_angle - min_range) / bin_size
    bin_index = int(bin_index)
    bin_index = min(bin_index, num_bins-1)  # handle extreme edge case when its num_bins 

    return bin_index


bus = FeetechBus("/dev/ttyACM0", 
                 [1,2,3,4,5,6],
                 calib_file="so101_calibration.json")

current_state = bus.get_qpos()

current_state_bins = discretize_state(current_state)



goal_state = np.array([0.5, 0, 0, 0, 0, 0], dtype=np.float32)
joint2_goal_raw = bus._rad_to_raw(goal_state)[1]
joint2_goal_bin = enc_to_bin(joint2_goal_raw)
print('joint 2 goal bin : ', joint2_goal_bin)

