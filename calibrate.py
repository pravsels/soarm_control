#!/usr/bin/env python3
# calibrate.py 

import argparse
import json
from bus import FeetechBus

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--ids", nargs="+", type=int,
                    default=[1, 2, 3, 4, 5, 6],
                    help="Bus IDs, ordered base→gripper")
    args = ap.parse_args()

    bus = FeetechBus(args.port, args.ids)

    try: 
        bus.set_torque(False)

        input("\nMove the arm to its *middle* pose, "
              "then press ENTER … ")

        # read encoder values 
        encoder_vals = bus.get_qpos(return_raw=True)

        for sid, val in zip(args.ids, encoder_vals):
            print(f"  ID {sid}: {val}")

        calib = {}                         # final dict → JSON

        for name, sid, mid_raw in zip(JOINT_NAMES, args.ids, encoder_vals):
            print(f"\nJoint {name}  (ID {sid})")

            input("  Rotate to *one* hard stop (either end) and press ENTER … ")
            stop1 = bus.get_qpos(return_raw=True)[args.ids.index(sid)]

            input("  Rotate to the *other* hard stop and press ENTER … ")
            stop2 = bus.get_qpos(return_raw=True)[args.ids.index(sid)]

            # Decide which is min / max
            raw_min, raw_max = sorted((stop1, stop2))

            calib[name] = {
                "id": sid,
                "homing_offset": int(mid_raw - 2048),
                "range_min": int(raw_min),
                "range_max": int(raw_max),
            }

            print(f"    ↳ offset {calib[name]['homing_offset']}, "
                f"range [{raw_min}, {raw_max}]")

        # ----------- save ---------------------------------------------------
        with open("so101_calibration.json", "w") as f:
            json.dump(calib, f, indent=2)
        print("\n✔ Saved so101_calibration.json")

    finally: 
        try: 
            bus.set_torque(False)
        finally: 
            bus.disconnect()


if __name__ == "__main__":
    main()
