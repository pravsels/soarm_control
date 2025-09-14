# bus.py

import time
import json 
import numpy as np
from scservo_sdk import PortHandler, PacketHandler, GroupSyncRead, GroupSyncWrite
from scservo_sdk import COMM_SUCCESS, COMM_TX_FAIL
from typing import Optional, List
from config import MOTOR_RESOLUTION

_CTL = {
    "ID": (5, 1),
    "Present_Position": (56, 2),
    "Goal_Position":    (42, 2),
    "Homing_Offset": (31, 2),
    "Min_Position_Limit": (9, 2),
    "Max_Position_Limit": (11, 2),
}

_SIGNBIT = {
    "Homing_Offset": 11,
}

_ENC2RAD = 2.0 * np.pi / MOTOR_RESOLUTION      # radians per encoder count 

MID_POSITION = int((MOTOR_RESOLUTION -1)/ 2)

def _load_calibration(path: str, ids: list[int]): 
    offset = np.zeros(len(ids), dtype=np.int32)
    range_min = np.zeros(len(ids), dtype=np.int32)
    range_max = np.full(len(ids), MOTOR_RESOLUTION - 1, dtype=np.int32)

    try: 
        with open(path) as f: 
            by_name = json.load(f)
        
        by_id = {e['id']: e for e in by_name.values()}

        for idx, sid in enumerate(ids):
            joint_info = by_id.get(sid)

            if joint_info:
                offset[idx] = int(joint_info.get('homing_offset', 0))
                range_min[idx] = int(joint_info.get('range_min', 0))
                range_max[idx] = int(joint_info.get('range_max', MOTOR_RESOLUTION - 1))

    except FileNotFoundError: 
        print('Please check if calibration file exists!')

    return offset, range_min, range_max

def busy_wait(dt_s: float):
    end = time.perf_counter() + dt_s
    while time.perf_counter() < end:
        pass

def _encode_signmag(v: int, sign_bit: int, nbytes: int) -> int:
    """Encode int v into sign-magnitude with given sign_bit."""
    mag_mask = (1 << sign_bit) - 1
    if v < 0:
        mag = min(-int(v), mag_mask)
        u = (1 << sign_bit) | mag
    else:
        mag = min(int(v), mag_mask)
        u = mag
    # clamp to register width
    width_mask = (1 << (8 * nbytes)) - 1
    return u & width_mask
    
def _decode_signmag(u: int, sign_bit: int, nbytes: int) -> int:
    """Decode unsigned u from sign-magnitude into python int."""
    width_mask = (1 << (8 * nbytes)) - 1
    u &= width_mask
    mag_mask = (1 << sign_bit) - 1
    
    sign_is_set = u & (1 << sign_bit)  # Check sign bit
    magnitude = u & mag_mask           # Extract magnitude bits
    
    return -magnitude if sign_is_set else magnitude

def _to_le_bytes(value: int, nbytes: int) -> list[int]:
    return list(int(value).to_bytes(nbytes, "little", signed=False))

class FeetechBus:
    def __init__(self, 
                 port: str, 
                 ids: list[int],
                 calib_file: str | None = None,
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
            self._max_raw = np.full(len(ids), MOTOR_RESOLUTION-1, np.int32)

    def disconnect(self):
        self.port_handler.closePort()

    def sync_read(self, reg_name: str, ids: Optional[list[int]] = None) -> np.ndarray:
        if reg_name not in _CTL:
            raise KeyError(f"Unknown register {reg_name}")
        addr, length = _CTL[reg_name]
        ids = self.ids if ids is None else ids 

        reader = GroupSyncRead(self.port_handler, self.packet_handler, addr, length)
        for sid in ids: 
            reader.addParam(sid)
        
        comm = reader.txRxPacket()
        if comm != COMM_SUCCESS:
            print("comm : ", comm, self.packet_handler.getTxRxResult(comm))
            raise RuntimeError(f"Read failed for '{reg_name}'")
        
        vals = []
        sign_bit = _SIGNBIT.get(reg_name)
        for sid in ids: 
            u = reader.getData(sid, addr, length)
            v = _decode_signmag(u, sign_bit, length) if sign_bit is not None else u 
            vals.append(v)
        
        return np.array(vals, dtype=np.int32)

    def sync_write(self, reg_name: str, values: list[int], ids: Optional[list[int]] = None):
        if reg_name not in _CTL:
            raise KeyError(f"Unknown register '{reg_name}'")
        addr, length = _CTL[reg_name]
        ids = self.ids if ids is None else ids

        if len(values) != len(ids):
            raise ValueError("values length must match ids length")
        
        writer = GroupSyncWrite(self.port_handler, self.packet_handler, addr, length)
        sign_bit = _SIGNBIT.get(reg_name)
        width_mask = (1 << (8 * length)) - 1

        for sid, v in zip(ids, values):
            u = _encode_signmag(int(v), sign_bit, length) if sign_bit is not None else (int(v) & width_mask)
            writer.addParam(sid, _to_le_bytes(u, length))

        if writer.txPacket() != COMM_SUCCESS:
            raise RuntimeError(f"Write failed for {reg_name}")

    def set_homing_offsets(self, raws: np.ndarray) -> np.ndarray:
        """Write Homing_Offset."""
        offsets = (raws.astype(np.int32) - MID_POSITION).tolist()
        self.sync_write("Homing_Offset", offsets)
        return np.array(offsets, dtype=np.int32)

    def get_qpos(self) -> np.ndarray:
        """Read Present_Position."""
        raw = self.sync_read("Present_Position")
        # normalize to be in range 0 - 4095
        raw = raw & 0x0FFF
        return raw

    def set_qpos(self, raw: np.ndarray):
        """Write Goal_Position."""
        self.sync_write("Goal_Position", raw)

    def set_torque(self, enabled: bool):
        """Enable/disable torque on all servos in this bus."""
        val = 1 if enabled else 0
        for sid in self.ids:
            # DYNAMIXEL/Feetech Torque‑Enable register = 40 (0x28), 1 byte
            res = self.packet_handler.write1ByteTxOnly(self.port_handler, sid, 40, val)
            if res not in (COMM_SUCCESS, COMM_TX_FAIL):
                raise RuntimeError(f"Torque write failed for ID {sid}")
            
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

