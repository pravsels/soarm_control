# bus.py

import time
import numpy as np
from scservo_sdk import PortHandler, PacketHandler, GroupSyncRead, GroupSyncWrite
from scservo_sdk import COMM_SUCCESS

_CTL = {
    "Present_Position": (56, 2),
    "Goal_Position":    (42, 2),
}

def busy_wait(dt_s: float):
    end = time.perf_counter() + dt_s
    while time.perf_counter() < end:
        pass

class FeetechBus:
    def __init__(self, port: str, ids: list[int],
                 baudrate: int = 1_000_000, protocol: int = 0):
        self.ids = ids
        self.ph = PortHandler(port)
        if not self.ph.openPort():
            raise OSError(f"Cannot open {port}")
        self.ph.setBaudRate(baudrate)
        self.pkt = PacketHandler(protocol)

        # setup reader
        addr_r, len_r = _CTL["Present_Position"]
        self.reader = GroupSyncRead(self.ph, self.pkt, addr_r, len_r)
        for i in ids:
            self.reader.addParam(i)

        # setup writer
        addr_w, len_w = _CTL["Goal_Position"]
        self.writer = GroupSyncWrite(self.ph, self.pkt, addr_w, len_w)

    def disconnect(self):
        self.ph.closePort()

    def get_qpos(self) -> np.ndarray:
        """Read Present_Position (radians)."""
        if self.reader.txRxPacket() != COMM_SUCCESS:
            raise RuntimeError("Read failed")
        raw = [self.reader.getData(i, *_CTL["Present_Position"]) for i in self.ids]
        # 12-bit → degrees → radians
        deg = (np.array(raw, np.float32) - 2048) * (360.0 / 4096.0)
        return np.deg2rad(deg)

    def set_qpos(self, qpos: np.ndarray):
        """Write Goal_Position (radians)."""
        deg = (np.rad2deg(qpos).astype(int) + 2048).tolist()
        self.writer.clearParam()
        for i, d in zip(self.ids, deg):
            low, high = d & 0xFF, (d >> 8) & 0xFF
            self.writer.addParam(i, [low, high])
        if self.writer.txPacket() != COMM_SUCCESS:
            raise RuntimeError("Write failed")
