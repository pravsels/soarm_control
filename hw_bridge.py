# hw_bridge.py 

import time, json, argparse, zmq
import numpy as np 
from bus import FeetechBus
from utils import make_pub, make_sub

# port for publishing real arm state 
PUB_ADDR = "tcp://*:6000"
# port for subscribing to real arm 
SUB_ADDR = "tcp://localhost:6001"

with open('so101_motorbus_port.json') as f: 
    port = json.load(f)['port']

JOINT_IDS = [1,2,3,4,5,6]
bus = FeetechBus(port, JOINT_IDS, calib_file='so101_calibration.json')

def run_loop(pub, sub, get_state, apply_state, topic_name):
    try:
        while True:
            # publish own state
            qpos = get_state()
            msg = {"t": time.time(), "qpos": qpos}
            pub.send_multipart([topic_name.encode(), json.dumps(msg).encode()])

            # if follower, apply master's state
            if sub and sub.poll(timeout=0):
                _, payload = sub.recv_multipart(flags=zmq.NOBLOCK)
                msg_master = json.loads(payload.decode())
                apply_state(np.array(msg_master["qpos"], dtype=np.float32))

            time.sleep(0.01)  # ~100Hz
    except KeyboardInterrupt:
        raise 

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["master"], help="Set hardware as master (default=follower)")
    args = parser.parse_args()
    is_master = args.mode == "master"

    # setup ZeroMQ publisher 
    ctx = zmq.Context()
    pub = make_pub(ctx, PUB_ADDR, "so101.state_real")
    sub = None if is_master else make_sub(ctx, SUB_ADDR, "so101.state_sim")

    def get_hw_state():
        return bus.get_qpos().astype(float).tolist()

    def apply_sim_state(qpos_sim):
        qpos_current = bus.get_qpos()
        delta = qpos_sim - qpos_current

        # clip each joint to ±rad
        max_step = 0.1
        delta_clipped = np.clip(delta, -max_step, max_step)
        qpos_next = qpos_current + delta_clipped
        
        bus.set_qpos(qpos_next)

    try:
        run_loop(pub, sub, get_hw_state, apply_sim_state, "so101.state_real")
    finally:
        pub.close(0)
        if sub:
            sub.close(0)
        ctx.term()

        bus.set_torque(False)
        bus.disconnect()

if __name__ == '__main__':
    main()
