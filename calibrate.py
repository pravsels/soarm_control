# calibrate.py 

import os 
import json, argparse, time 
from bus import FeetechBus
from config import JOINT_NAMES, UIDS

def main():

    parser = argparse.ArgumentParser()

    parser.add_argument("--mode", 
                        choices=["leader", "follower"], 
                        default="follower",
                        help="Set hardware as leader (default=follower)")
    parser.add_argument("--device",
                        choices=['so101', 'so100'],
                        default="so101",
                        help="Which device config to use (default=so101)")
    
    args = parser.parse_args()

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

    print(f"Starting calibration for device: {device_name}")

    try: 
        bus.set_torque(False)

        bus.sync_write("Homing_Offset", [0] * len(UIDS), ids=UIDS)
        time.sleep(0.05)  # small pause for EEPROM write

        input("\nMove the arm to its *middle* pose, "
              "then press ENTER … ")

        # read encoder values 
        encoder_vals = bus.get_qpos()

        homing_offsets = bus.set_homing_offsets(encoder_vals)

        for sid, val in zip(UIDS, encoder_vals):
            print(f"  ID {sid}: {val}")

        calib = {}                         

        for name, sid, homing_offset in zip(JOINT_NAMES, UIDS, homing_offsets):
            print(f"\nJoint {name}  (ID {sid})")

            input("  Rotate to *one* hard stop (either end) and press ENTER … ")
            stop1 = bus.get_qpos()[UIDS.index(sid)]

            input("  Rotate to the *other* hard stop and press ENTER … ")
            stop2 = bus.get_qpos()[UIDS.index(sid)]

            # Decide which is min / max
            raw_min, raw_max = sorted((stop1, stop2))

            bus.sync_write("Min_Position_Limit", [raw_min], ids=[sid])
            bus.sync_write("Max_Position_Limit", [raw_max], ids=[sid])

            calib[name] = {
                "id": sid,
                "drive_mode": 0,
                "homing_offset": int(homing_offset),
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
