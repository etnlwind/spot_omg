#!/usr/bin/env python3
"""List serial ports which may be connected to a servo adapter."""

from serial.tools import list_ports


def main() -> None:
    ports = sorted(list_ports.comports(), key=lambda item: item.device)
    if not ports:
        print("No serial ports found.")
        return
    for port in ports:
        description = port.description or "unknown device"
        print(f"{port.device:20} {description}")


if __name__ == "__main__":
    main()
