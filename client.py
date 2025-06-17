#!/usr/bin/env python3
# client.py — main entry point for SO arm control
import argparse
import json
import numpy as np
import os
from bus import FeetechBus
from robot import Robot
from config import UIDS, DEFAULT_PORT

def load_saved_port(uid):
    """Load saved port from JSON file, return None if not found."""
    filename = f"{uid}_motorbus_port.json"
    try:
        with open(filename, 'r') as f:
            config = json.load(f)
            return config.get("port")
    except (FileNotFoundError, json.JSONDecodeError, KeyError):
        return None

def main():
    parser = argparse.ArgumentParser(prog="soarm-control")
    parser.add_argument("--uid", choices=list(UIDS.keys()), default="so100",
                       help="Robot UID (uses default IDs and calibration file)")
    parser.add_argument("--port", default=None,
                       help="Serial port for the motors bus (e.g. /dev/ttyACM0)")
    parser.add_argument("--ids", nargs="+", type=int,
                       help="Override default motor IDs")
    parser.add_argument("--calib-file", default=None,
                       help="Path to calibration JSON file. Defaults to '<uid>_calibration.json'.")
    parser.add_argument("--action", nargs=6, required=True, type=float,
                       help="Six target joint angles in radians")
    parser.add_argument("--rate", type=int, default=30,
                       help="Control loop frequency in Hz")
    parser.add_argument("--step", type=float, default=0.025,
                       help="Max joint-step in radians per cycle")
    parser.add_argument("--timeout", type=float, default=20.0,
                       help="Safety timeout in seconds")
    
    args = parser.parse_args()

    # Determine motor IDs
    ids = args.ids if args.ids is not None else UIDS[args.uid]
    
    # Determine port: command line arg > saved port > default
    port = args.port
    if port is None:
        port = load_saved_port(args.uid)
        if port:
            print(f"Using saved port for {args.uid}: {port}")
    if port is None:
        port = DEFAULT_PORT
        print(f"Using default port: {port}")

    # Initialize bus and load zero-offset calibration
    bus = FeetechBus(port, ids)
    calib_path = args.calib_file or f"{args.uid}_calibration.json"
    
    try:
        # Create and run the motion controller
        robot = Robot(bus,
                     rate_hz=args.rate,
                     max_step=args.step,
                     timeout=args.timeout)
        final = robot.move_to(np.array(args.action, dtype=np.float32))
        print(f"\nFinal qpos (rad): {final.tolist()}")
    finally:
        bus.disconnect()

if __name__ == "__main__":
    main()

