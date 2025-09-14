# teleop.py 

import time, json, argparse, zmq
import numpy as np 
from bus import FeetechBus
from utils import make_pub, make_sub, to_norm, from_norm
from config import UIDS
import sys 

def run_loop(pub, sub, get_state, apply_state, topic_name, calib_by_id, debug=False):
    try:
        follower_goal = None
        while True:
            # publish own state
            qpos, qpos_norm = get_state()
            msg = {"t": time.time(), "qpos_norm": qpos_norm}
            pub.send_multipart([topic_name.encode(), json.dumps(msg).encode()])

            if sub and sub.poll(timeout=0):
                latest_payload = None
                
                # Manually drain all messages, keep only the latest
                while sub.poll(timeout=0):
                    _, latest_payload = sub.recv_multipart(flags=zmq.NOBLOCK)
                
                if latest_payload:
                    latest_msg = json.loads(latest_payload.decode())
                    follower_goal = np.array(latest_msg["qpos_norm"], dtype=np.float32)
                    apply_state(follower_goal)

            time.sleep(0.01)  # ~100Hz

            if debug:
                sys.stdout.write(f"\033[{len(UIDS)}F") 

                lines = []
                for i, uid in enumerate(UIDS):
                    entry = calib_by_id.get(uid, {})
                    rmin, rmax = entry.get("range_min", 0), entry.get("range_max", 0)
                    norm_val = qpos_norm[i]
                    row = f"{i+1:<5} {entry['name']:<15} {qpos[i]:>6} {rmin:>6} {rmax:>6} {norm_val:>7.1f}"

                    if follower_goal is not None:  # only for follower
                        row += f" goal={follower_goal[i]:>7.1f}"

                    lines.append(row)

                sys.stdout.write("\n".join(lines) + "\n")
                sys.stdout.flush()
        
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
    parser.add_argument("--debug", action="store_true",
                        help="Show realtime joint table + goals (if follower)")

    args = parser.parse_args()
    
    is_leader = args.mode == "leader"
    family, role = args.device, args.mode
    device_name = f"{family}_{role}" 
    
    with open(f"{device_name}_motorbus_port.json") as f: 
        port = json.load(f)['port']
    with open(f"{device_name}_calibration.json") as f:
        calib_json = json.load(f)
    calib_by_id = {}
    for name, entry in calib_json.items():
        entry = dict(entry)   # copy so we can add to it
        entry["name"] = name  # stash the joint name
        calib_by_id[entry["id"]] = entry

    bus = FeetechBus(port, UIDS, calib_file=f"{device_name}_calibration.json")

    BASE_PORTS = {
        "so101": {"leader": 6000, "follower": 6001},
    }

    if family not in BASE_PORTS:
        raise ValueError(f"No base port defined for {family}")
    
    if is_leader:
        pub_port = BASE_PORTS[family]["leader"]
        sub_port = BASE_PORTS[family]["follower"] 
    else:
        pub_port = BASE_PORTS[family]["follower"]
        sub_port = BASE_PORTS[family]["leader"]

    peer = f"{family}_follower" if role == "leader" else f"{family}_leader"
    
    ctx = zmq.Context()

    # Publisher setup
    pub_addr = f"tcp://*:{pub_port}"
    pub = make_pub(ctx, pub_addr, f"{device_name}.state_real", bind=True)

    # Subscriber setup  
    sub_addr = f"tcp://localhost:{sub_port}"
    sub = None if is_leader else make_sub(ctx, sub_addr, f"{peer}.state_real")

    def get_hw_state():
        raw = bus.get_qpos()
        return raw, to_norm(raw, calib_by_id, UIDS).tolist()

    def apply_state(qpos_norm):
        raw_current, norm_current = get_hw_state()

        delta = qpos_norm - norm_current
        max_step = 10.0  # step in normalized units
        norm_next = norm_current + np.clip(delta, -max_step, max_step)

        raw_next = from_norm(norm_next, calib_by_id, UIDS)
        bus.set_qpos(raw_next)

    try:
        if args.debug:
            print(f"{'idx':<5} {'name':<15} {'raw':>6} {'min':>6} {'max':>6} {'norm':>8}")
            for _ in range(len(UIDS)):
                print()  # reserve N lines
                
        run_loop(pub, sub, 
                 get_hw_state, 
                 apply_state, 
                 f"{device_name}.state_real", 
                 calib_by_id,
                 debug=args.debug)
    finally:
        pub.close(0)
        if sub:
            sub.close(0)
        ctx.term()

        bus.set_torque(False)
        bus.disconnect()

        if args.debug:
            sys.stdout.write("\033[?25h\n")
            sys.stdout.flush()

if __name__ == '__main__':
    main()
