# hw_bridge.py 

import time, json, argparse, zmq
import numpy as np 
from bus import FeetechBus
from utils import make_pub, make_sub
from config import UIDS

# port for publishing real arm state 
PUB_ADDR = "tcp://*:6000"
# port for subscribing to real arm 
SUB_ADDR = "tcp://localhost:6001"

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

            time.sleep(0.02)  # ~100Hz
    except KeyboardInterrupt:
        raise 

def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--mode", 
                        choices=["leader"], 
                        default="follower",
                        help="Set hardware as leader (default=follower)")
    parser.add_argument("--device",
                        choices=['so101', 'so100'],
                        default="so101",
                        help="Which device config to use (default=so101)")

    args = parser.parse_args()
    
    is_leader = args.mode == "leader"
    family, role = args.device, args.mode
    device_name = f"{family}_{role}" 
    
    with open(f"{device_name}_motorbus_port.json") as f: 
        port = json.load(f)['port']

    bus = FeetechBus(port, UIDS, calib_file=f"{device_name}_calibration.json")

    BASE_PORTS = {
        "so101": 6000,
    }
    if family not in BASE_PORTS:
        raise ValueError(f"No base port defined for {family}")

    peer = f"{family}_follower" if role == "leader" else f"{family}_leader"
    pub_port = BASE_PORTS[family]
    
    ctx = zmq.Context()

    if is_leader:
        pub_addr = f"tcp://*:{pub_port}"
        pub = make_pub(ctx, pub_addr, f"{device_name}.state_real", bind=True)
    else:
        pub_addr = f"tcp://localhost:{pub_port}"
        pub = make_pub(ctx, pub_addr, f"{device_name}.state_real", bind=False)
    
    sub_addr = f"tcp://localhost:{pub_port}"
    sub = None if is_leader else make_sub(ctx, sub_addr, f"{peer}.state_real")

    def get_hw_state():
        return bus.get_qpos().astype(float).tolist()

    def apply_state(qpos_sim):
        qpos_current = bus.get_qpos()
        delta = qpos_sim - qpos_current

        # clip each joint to ±rad
        max_step = 0.1
        delta_clipped = np.clip(delta, -max_step, max_step)
        qpos_next = qpos_current + delta_clipped
        
        bus.set_qpos(qpos_next)

    try:
        run_loop(pub, sub, get_hw_state, apply_state, f"{device_name}.state_real")
    finally:
        pub.close(0)
        if sub:
            sub.close(0)
        ctx.term()

        bus.set_torque(False)
        bus.disconnect()

if __name__ == '__main__':
    main()
