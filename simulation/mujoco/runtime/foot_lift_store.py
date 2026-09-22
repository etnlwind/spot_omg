"""Simulator equivalent of robot-owned persistent foot-lift settings."""
import json
import os
from pathlib import Path
import tempfile


def validate(values):
    if not isinstance(values, list) or len(values) != 4 or any(type(v) is not int or not 0 <= v <= 2147483647 for v in values):
        raise ValueError('four nonnegative integer mm required')
    return values


def validate_width(values):
    if not isinstance(values,list) or len(values)!=4 or any(type(v) is not int or not -2147483647<=v<=2147483647 for v in values):
        raise ValueError('four signed integer width mm required')
    return values


class FootLiftStore:
    def __init__(self, path=None):
        self.path = Path(path) if path else None
        self.values = [0]*4
        self.widths = [0]*4
        if self.path and self.path.exists():
            saved=json.loads(self.path.read_text())
            self.values=validate(saved['foot_lift_mm'])
            self.widths=validate_width(saved.get('foot_width_mm',[0]*4))

    def save(self, values, widths=None):
        values = validate(values)
        widths=validate_width(self.widths if widths is None else widths)
        if values == self.values and widths == self.widths:
            return
        if self.path:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            fd, temporary = tempfile.mkstemp(dir=self.path.parent, prefix='.foot-lift-')
            try:
                with os.fdopen(fd, 'w') as stream:
                    json.dump({'version': 2, 'foot_lift_mm': values, 'foot_width_mm': widths}, stream)
                    stream.flush(); os.fsync(stream.fileno())
                os.replace(temporary, self.path)
            finally:
                if os.path.exists(temporary): os.unlink(temporary)
        self.values = list(values)
        self.widths = list(widths)
