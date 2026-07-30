#!/usr/bin/env python3
"""Change one servo ID. Keep only one servo connected while doing this."""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from servo import STS3215, ServoBus  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port")
    parser.add_argument("old_id", type=int)
    parser.add_argument("new_id", type=int)
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument(
        "--yes",
        action="store_true",
        help="confirm that only the target servo is connected",
    )
    args = parser.parse_args()
    if not args.yes:
        parser.error("disconnect other servos, then pass --yes")

    with ServoBus(args.port, args.baudrate) as bus:
        servo = STS3215(bus, args.old_id)
        if not servo.ping():
            raise SystemExit(f"servo {args.old_id} did not respond")
        servo.change_id(args.new_id)
        print(f"Changed servo ID: {args.old_id} -> {args.new_id}")


if __name__ == "__main__":
    main()
