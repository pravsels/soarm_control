# calibrate.py 

import os 
import json, argparse
from bus import FeetechBus
from config import JOINT_NAMES, UIDS

def main():

    parser = argparse.ArgumentParser()

    parser.add_argument("--mode", 
                        choices=["leader"], 
                        default="follower",
                        help="Set hardware as leader (default=follower)")
    parser.add_argument("--device",
                        choices=['so101', 'so100'],
                        default="so101",
                        help="Which device config to use (default=so101)")
    
    family, role = args.device, args.mode 
    device_name = f"{family}_{role}"
    
    # Load port info from the corresponding motorbus config
    port_config_file = f"{device_name}_motorbus_port.json"
    if not os.path.exists(port_config_file):
        raise FileNotFoundError(
            f"Missing {port_config_file}. Please run the port detection script first."
        )
    with open(port_config_file, "r") as f:
        port_config = json.load(f)
    port = port_config.get("port")
    if not port:
        raise ValueError(f"Invalid config in {port_config_file}: 'port' missing.")

    calib_file = f"{device_name}_calibration.json"

    bus = FeetechBus(port, UIDS)

    try: 
        bus.set_torque(False)

        input("\nMove the arm to its *middle* pose, "
              "then press ENTER … ")

        # read encoder values 
        encoder_vals = bus.get_qpos(return_raw=True)

        for sid, val in zip(UIDS, encoder_vals):
            print(f"  ID {sid}: {val}")

        calib = {}                         

        for name, sid, mid_raw in zip(JOINT_NAMES, UIDS, encoder_vals):
            print(f"\nJoint {name}  (ID {sid})")

            input("  Rotate to *one* hard stop (either end) and press ENTER … ")
            stop1 = bus.get_qpos(return_raw=True)[UIDS.index(sid)]

            input("  Rotate to the *other* hard stop and press ENTER … ")
            stop2 = bus.get_qpos(return_raw=True)[UIDS.index(sid)]

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

        # save 
        with open(calib_file, "w") as f:
            json.dump(calib, f, indent=2)
        print(f"Saved {calib_file}")

    finally: 
        try: 
            bus.set_torque(False)
        finally: 
            bus.disconnect()


if __name__ == "__main__":
    main()
