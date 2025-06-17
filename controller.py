# controller.py

import time
import numpy as np
from bus import busy_wait

def move_to_qpos(bus, target: np.ndarray,
                 rate_hz: int = 30,
                 max_step: float = 0.025,
                 timeout: float = 20.0):
    """
    Ramp current → target pose at `rate_hz`, stepping ≤ max_step per joint.
    """
    cur = bus.get_qpos()
    t0 = time.perf_counter()
    period = 1.0 / rate_hz

    while True:
        delta = np.clip(target - cur, -max_step, max_step)
        if np.linalg.norm(delta) < 1e-4:
            break
        cur += delta
        bus.set_qpos(cur)

        elapsed = time.perf_counter() - t0
        busy_wait(max(0.0, period - (elapsed % period)))
        if elapsed > timeout:
            print("⚠️  timed out")
            break

    return cur
