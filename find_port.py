#!/usr/bin/env python3
# Copyright 2024 The HuggingFace Inc. team.
# Licensed under the Apache License, Version 2.0.
import time
import json
from serial.tools import list_ports

def list_serial_ports():
    """Return a set of all serial port device names."""
    return {port.device for port in list_ports.comports()}

def find_motorsbus_port():
    before = list_serial_ports()
    input("1) Unplug your MotorsBus USB cable, then press Enter → ")
    time.sleep(0.5)
    removed = list(before - list_serial_ports())
    
    match removed:
        case [port]:
            print(f"Detected MotorsBus on port: {port}")
            input("Re-plug the USB cable now, then press Enter → ")
            
            # Ask user for a name to save the port configuration
            name = input("Enter a name for this MotorsBus device: ").strip()
            if not name:
                name = "default"
            
            # Save the port configuration
            config = {"port": port}
            filename = f"{name}_motorbus_port.json"
            
            try:
                with open(filename, 'w') as f:
                    json.dump(config, f, indent=2)
                print(f"Port configuration saved to: {filename}")
                return port
            except Exception as e:
                print(f"Warning: Could not save configuration file: {e}")
                return port
                
        case []:
            raise OSError("No port change detected. Make sure you unplugged the device.")
        case _:
            raise OSError(f"Multiple ports changed: {removed}. Try again one at a time.")

if __name__ == "__main__":
    try:
        port = find_motorsbus_port()
        print(f"MotorsBus successfully detected on: {port}")
    except OSError as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("\nOperation cancelled by user.")

        