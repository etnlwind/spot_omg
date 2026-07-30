#!/usr/bin/env python3
"""Scan a serial bus for responding servos."""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from servo import ServoBus  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--min-id", type=int, default=1)
    parser.add_argument("--max-id", type=int, default=253)
    args = parser.parse_args()
    if not 0 <= args.min_id <= args.max_id <= 253:
        parser.error("ID range must satisfy 0 <= min-id <= max-id <= 253")

    with ServoBus(args.port, args.baudrate) as bus:
        found = bus.scan(range(args.min_id, args.max_id + 1))
    print("Found:", ", ".join(map(str, found)) if found else "none")


if __name__ == "__main__":
    main()
