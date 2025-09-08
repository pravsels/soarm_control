# hw_bridge.py 

import time, json, argparse, zmq
import numpy as np 
from bus import FeetechBus
from utils import make_pub, make_sub, to_norm, from_norm
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
                        choices=["leader", "follower"], 
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
    with open(f"{device_name}_calibration.json") as f:
        calib_json = json.load(f)
    calib_by_id = {entry["id"]: entry for entry in calib_json.values()}

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
        raw = bus.get_qpos(return_raw=True)
        return to_norm(raw, calib_by_id, UIDS).tolist()

    def apply_state(qpos_norm):
        raw_current = bus.get_qpos(return_raw=True)
        norm_current = to_norm(raw_current, calib_by_id, UIDS)

        delta = np.array(qpos_norm, dtype=np.float32) - norm_current
        max_step = 5.0  # step in normalized units
        norm_next = norm_current + np.clip(delta, -max_step, max_step)

        raw_next = from_norm(norm_next, calib_by_id, UIDS)
        qpos_next_rad = bus._raw_to_rad(raw_next)  
        bus.set_qpos(qpos_next_rad.astype(np.float32))

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
