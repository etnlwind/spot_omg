#!/usr/bin/env python3
"""Move a servo to a raw position (0-4095)."""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from servo import STS3215, ServoBus  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port")
    parser.add_argument("position", type=int)
    parser.add_argument("--id", type=int, default=1)
    parser.add_argument("--speed", type=int, default=1000)
    parser.add_argument("--acceleration", type=int, default=50)
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    args = parser.parse_args()

    with ServoBus(args.port, args.baudrate) as bus:
        servo = STS3215(bus, args.id)
        servo.enable_torque()
        servo.move(
            args.position, speed=args.speed, acceleration=args.acceleration
        )
        print(f"Servo {args.id} commanded to position {args.position}.")


if __name__ == "__main__":
    main()
