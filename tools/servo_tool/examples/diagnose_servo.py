#!/usr/bin/env python3
"""Read a compact health snapshot from one servo."""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from servo import STS3215, ServoBus  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port")
    parser.add_argument("--id", type=int, default=1)
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    args = parser.parse_args()

    with ServoBus(args.port, args.baudrate) as bus:
        servo = STS3215(bus, args.id)
        state = servo.read_state()

    print(f"ID:          {args.id}")
    print(f"Position:    {state.position}")
    print(f"Speed:       {state.speed}")
    print(f"Load:        {state.load}")
    print(f"Voltage:     {state.voltage:.1f} V")
    print(f"Temperature: {state.temperature} C")
    print(f"Current:     {state.current}")
    print(f"Moving:      {'yes' if state.moving else 'no'}")


if __name__ == "__main__":
    main()
