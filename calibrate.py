#!/usr/bin/env python3
# calibrate.py — simple mechanical zero calibration

import argparse
import json
from bus import FeetechBus
from config import UIDS, DEFAULT_PORT

def main():
    parser = argparse.ArgumentParser(prog="soarm-calibrate")
    parser.add_argument(
        "--uid", choices=list(UIDS.keys()), default="so100",
        help="Robot UID (determines default motor IDs)"
    )
    parser.add_argument(
        "--port", default=DEFAULT_PORT,
        help="Serial port for the bus (e.g. /dev/ttyACM0)"
    )
    parser.add_argument(
        "--ids", nargs="+", type=int,
        help="Override default motor IDs"
    )
    parser.add_argument(
        "--output", help="Output filename (defaults to <uid>_calibration.json)"
    )
    args = parser.parse_args()

    ids = args.ids if args.ids else UIDS[args.uid]
    bus = FeetechBus(args.port, ids)
    try:
        # Prompt user and read raw positions as zero offsets
        input("Move all joints to your middle pose, then press ENTER → ")
        offsets = bus.get_qpos()
        bus.zero_offset = offsets
    finally:
        bus.disconnect()

    # Persist to <uid>_calibration.json or custom file
    fname = args.output or f"{args.uid}_calibration.json"
    # Build a simple dict: motor ID → offset (rad)
    calib_dict = {str(mid): float(off) for mid, off in zip(ids, offsets.tolist())}
    with open(fname, "w") as f:
        json.dump(calib_dict, f, indent=2)
    print(f"✔ Saved zero-offset calibration to '{fname}'")


if __name__ == "__main__":
    main()
