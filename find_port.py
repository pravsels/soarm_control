#!/usr/bin/env python3
# Copyright 2024 The HuggingFace Inc. team.
# Licensed under the Apache License, Version 2.0.

import time
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
            print(f"Detected MotorsBus on port: {port}\nRe-plug the USB cable now.")
        case []:
            raise OSError("No port change detected. Make sure you unplugged the device.")
        case _:
            raise OSError(f"Multiple ports changed: {removed}. Try again one at a time.")

if __name__ == "__main__":
    find_motorsbus_port()
