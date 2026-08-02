#!/usr/bin/env python3
"""Compatibility wrapper for the installed ``spotctl`` command."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from servo.cli import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
