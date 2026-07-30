#!/usr/bin/env python3
"""Enable or disable servo torque."""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from servo import STS3215, ServoBus  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port")
    parser.add_argument("state", choices=("on", "off"))
    parser.add_argument("--id", type=int, default=1)
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    args = parser.parse_args()

    with ServoBus(args.port, args.baudrate) as bus:
        STS3215(bus, args.id).enable_torque(args.state == "on")
        print(f"Servo {args.id} torque {args.state}.")


if __name__ == "__main__":
    main()
