# bus.py

import time
import numpy as np
from scservo_sdk import PortHandler, PacketHandler, GroupSyncRead, GroupSyncWrite
from scservo_sdk import COMM_SUCCESS, COMM_TX_FAIL

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
        self.port_handler = PortHandler(port)
        if not self.port_handler.openPort():
            raise OSError(f"Cannot open {port}")
        self.port_handler.setBaudRate(baudrate)
        self.packet_handler = PacketHandler(protocol)

        # setup reader
        addr_r, len_r = _CTL["Present_Position"]
        self.reader = GroupSyncRead(self.port_handler, self.packet_handler, addr_r, len_r)
        for i in ids:
            self.reader.addParam(i)

        # setup writer
        addr_w, len_w = _CTL["Goal_Position"]
        self.writer = GroupSyncWrite(self.port_handler, self.packet_handler, addr_w, len_w)

    def disconnect(self):
        self.port_handler.closePort()

    def get_qpos(self, return_raw=False) -> np.ndarray:
        """Read Present_Position (radians)."""
        if self.reader.txRxPacket() != COMM_SUCCESS:
            raise RuntimeError("Read failed")
        raw = [self.reader.getData(i, *_CTL["Present_Position"]) for i in self.ids]
        if return_raw:
            return raw 
        # 12-bit → degrees (-180 to 180) → radians
        deg = (np.array(raw, np.float32) - 2048) * (360.0 / 4096.0)
        return np.deg2rad(deg)

    def set_qpos(self, qpos: np.ndarray):
        """Write Goal_Position (radians)."""
        deg = ((np.rad2deg(qpos) * (4096.0 / 360.0)).astype(int) + 2048).tolist()
        self.writer.clearParam()
        for i, d in zip(self.ids, deg):
            # extract low order and high order bits 
            low, high = d & 0xFF, (d >> 8) & 0xFF
            self.writer.addParam(i, [low, high])
        if self.writer.txPacket() != COMM_SUCCESS:
            raise RuntimeError("Write failed")

    def set_torque(self, enabled: bool):
        """Enable/disable torque on all servos in this bus."""
        val = 1 if enabled else 0
        for sid in self.ids:
            # DYNAMIXEL/Feetech Torque‑Enable register = 40 (0x28), 1 byte
            dxl_comm = self.packet_handler.write1ByteTxRx(self.port_handler, sid, 40, val)
            