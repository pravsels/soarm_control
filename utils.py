
import zmq
import numpy as np

def make_pub(ctx, addr, topic_name, bind=True):
    pub = ctx.socket(zmq.PUB)
    if bind:
        pub.bind(addr)
        print(f"Publishing (bind) on {addr}, topic = {topic_name}")
    else:
        pub.connect(addr)
        print(f"Publishing (connect) to {addr}, topic = {topic_name}")
    return pub

def make_sub(ctx, addr, topic_name):
    sub = ctx.socket(zmq.SUB)
    sub.connect(addr)
    sub.setsockopt(zmq.SUBSCRIBE, topic_name.encode())
    sub.setsockopt(zmq.CONFLATE, 1)
    print(f"Subscribed to {addr}, topic = {topic_name}")
    return sub

NORM_RANGE_MAX = 200.0

def to_norm(raw_vals, calib_by_id, ids):
    """Map raw ticks → normalized values using [range_min, range_max]."""
    norm = []
    for raw, sid in zip(raw_vals, ids):
        c = calib_by_id[sid]
        rmin, rmax = c["range_min"], c["range_max"]
        t = (raw - rmin) / float(rmax - rmin + 1e-9)
        t = float(np.clip(t, 0.0, 1.0)) 

        norm.append(t * NORM_RANGE_MAX)

    return np.array(norm, dtype=np.float32)

def from_norm(norm_vals, calib_by_id, ids):
    """Map normalized values → raw ticks using [range_min, range_max]."""
    raw = []
    for n, sid in zip(norm_vals, ids):
        c = calib_by_id[sid]
        rmin, rmax = c["range_min"], c["range_max"]

        t = np.clip(n / NORM_RANGE_MAX, 0.0, 1.0)

        raw.append(int(round(rmin + t * (rmax - rmin))))
    return np.array(raw, dtype=np.int32)

