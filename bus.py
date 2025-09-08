# bus.py

import time
import json 
import numpy as np
from scservo_sdk import PortHandler, PacketHandler, GroupSyncRead, GroupSyncWrite
from scservo_sdk import COMM_SUCCESS, COMM_TX_FAIL
from typing import Optional, List

_CTL = {
    "Present_Position": (56, 2),
    "Goal_Position":    (42, 2),
}

_ENC2RAD = 2.0 * np.pi / 4096.      # radians per encoder count 

def _load_calibration(path: str, ids: list[int]): 
    offset = np.zeros(len(ids), dtype=np.int32)
    range_min = np.zeros(len(ids), dtype=np.int32)
    range_max = np.full(len(ids), 4095, dtype=np.int32)

    try: 
        with open(path) as f: 
            by_name = json.load(f)
        
        by_id = {e['id']: e for e in by_name.values()}

        for idx, sid in enumerate(ids):
            joint_info = by_id[sid]

            if joint_info:
                offset[idx] = int(by_id[sid]['homing_offset'])
                range_min[idx] = int(by_id[sid]['range_min'])
                range_max[idx] = int(by_id[sid]['range_max'])

    except FileNotFoundError: 
        print('Please check if calibration file exists!')

    return offset, range_min, range_max

def busy_wait(dt_s: float):
    end = time.perf_counter() + dt_s
    while time.perf_counter() < end:
        pass

class FeetechBus:
    def __init__(self, 
                 port: str, 
                 ids: list[int],
                 calib_file: str,
                 baudrate: int = 1_000_000, 
                 protocol: int = 0):
        self.ids = ids
        self.port_handler = PortHandler(port)
        if not self.port_handler.openPort():
            raise OSError(f"Cannot open {port}")
        self.port_handler.setBaudRate(baudrate)
        self.packet_handler = PacketHandler(protocol)

        self.assert_same_firmware()

        if calib_file:
            self._off_raw, self._min_raw, self._max_raw = _load_calibration(calib_file, ids)
        else: 
            self._off_raw = np.zeros(len(ids), np.int32)
            self._min_raw = np.zeros(len(ids), np.int32)
            self._max_raw = np.full(len(ids), 4095, np.int32)

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
        comm = self.reader.txRxPacket()
        if comm != COMM_SUCCESS:
            print("comm : ", comm, self.packet_handler.getTxRxResult(comm))
            raise RuntimeError("Read failed")
        raw = [self.reader.getData(i, *_CTL["Present_Position"]) for i in self.ids]
        raw = np.array(raw, dtype=np.int32)
        # normalize to be in range 0-4095
        raw = raw & 0x0FFF
        return raw if return_raw else self._raw_to_rad(raw)

    def set_qpos(self, qpos: np.ndarray):
        """Write Goal_Position (radians)."""
        raw = self._rad_to_raw(qpos)
        self.writer.clearParam()
        for i, enc in zip(self.ids, raw):
            # extract low order and high order bits 
            low, high = enc & 0xFF, (enc >> 8) & 0xFF
            self.writer.addParam(i, [low, high])
        if self.writer.txPacket() != COMM_SUCCESS:
            raise RuntimeError("Write failed")

    def set_torque(self, enabled: bool):
        """Enable/disable torque on all servos in this bus."""
        val = 1 if enabled else 0
        for sid in self.ids:
            # DYNAMIXEL/Feetech Torque‑Enable register = 40 (0x28), 1 byte
            res = self.packet_handler.write1ByteTxOnly(self.port_handler, sid, 40, val)
            if res not in (COMM_SUCCESS, COMM_TX_FAIL):
                raise RuntimeError(f"Torque write failed for ID {sid}")
            
    def _raw_to_rad(self, raw: np.ndarray) -> np.ndarray:
        return (raw - 2048 - self._off_raw) * _ENC2RAD

    def _rad_to_raw(self, rad: np.ndarray) -> np.ndarray:
        enc_val = (rad / _ENC2RAD) + 2048 + self._off_raw
        return np.clip(enc_val.round().astype(int), self._min_raw, self._max_raw)

    def get_firmware_versions(self):
        """Return {id: 'X'} for each motor (firmware version byte)."""
        versions = {}
        for sid in self.ids:
            # major
            major, comm, err = self.packet_handler.read1ByteTxRx(self.port_handler, sid, 0)
            if comm != COMM_SUCCESS or err != 0:
                print(f"Could not read firmware major for ID {sid}")
                continue

            # minor
            minor, comm, err = self.packet_handler.read1ByteTxRx(self.port_handler, sid, 1)
            if comm != COMM_SUCCESS or err != 0:
                print(f"Could not read firmware minor for ID {sid}")
                continue

            versions[sid] = f"{major}.{minor}"
        return versions

    def assert_same_firmware(self):
        versions = self.get_firmware_versions()
        if not versions:
            raise RuntimeError("Could not read firmware versions from any motors")
        if len(set(versions.values())) != 1:
            raise RuntimeError(
                "Motors have mismatched firmware versions:\n"
                + "\n".join(f"  ID {sid}: {ver}" for sid, ver in versions.items())
            )
        # print(f"All motors firmware version {next(iter(versions.values()))}")

