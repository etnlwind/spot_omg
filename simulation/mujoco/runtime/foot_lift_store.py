"""Simulator equivalent of robot-owned persistent foot-lift settings."""
import json
import os
from pathlib import Path
import tempfile


def validate(values):
    if not isinstance(values, list) or len(values) != 4 or any(type(v) is not int or not 0 <= v <= 2147483647 for v in values):
        raise ValueError('four nonnegative integer mm required')
    return values


class FootLiftStore:
    def __init__(self, path=None):
        self.path = Path(path) if path else None
        self.values = [0]*4
        if self.path and self.path.exists():
            self.values = validate(json.loads(self.path.read_text())['foot_lift_mm'])

    def save(self, values):
        values = validate(values)
        if values == self.values:
            return
        if self.path:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            fd, temporary = tempfile.mkstemp(dir=self.path.parent, prefix='.foot-lift-')
            try:
                with os.fdopen(fd, 'w') as stream:
                    json.dump({'version': 1, 'foot_lift_mm': values}, stream)
                    stream.flush(); os.fsync(stream.fileno())
                os.replace(temporary, self.path)
            finally:
                if os.path.exists(temporary): os.unlink(temporary)
        self.values = list(values)
