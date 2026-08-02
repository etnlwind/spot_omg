#!/usr/bin/env python3
"""Change one servo ID. Keep only one servo connected while doing this."""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from servo.cli import main as cli_main  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port")
    parser.add_argument("old_id", type=int)
    parser.add_argument("new_id", type=int)
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    args = parser.parse_args()

    raise SystemExit(
        cli_main(
            [
                "--port",
                args.port,
                "--baudrate",
                str(args.baudrate),
                "change-id",
                str(args.old_id),
                str(args.new_id),
            ]
        )
    )


if __name__ == "__main__":
    main()
