# robot.py
import time
import numpy as np
from bus import FeetechBus, busy_wait

class Robot:
    def __init__(self, bus: FeetechBus, rate_hz=30, max_step=0.025, timeout=20.0):
        self.bus       = bus
        self.rate_hz   = rate_hz
        self.max_step  = max_step
        self.timeout   = timeout
        self.period    = 1.0 / rate_hz

    def move_to(self, target: np.ndarray) -> np.ndarray:
        cur = self.bus.get_qpos()
        t0  = time.perf_counter()

        while True:
            delta = np.clip(target - cur, -self.max_step, self.max_step)
            if np.linalg.norm(delta) < 1e-4:
                break

            cur += delta
            self.bus.set_qpos(cur)

            elapsed = time.perf_counter() - t0
            busy_wait(max(0.0, self.period - (elapsed % self.period)))
            if elapsed > self.timeout:
                print("⚠️ timeout")
                break

        return cur
