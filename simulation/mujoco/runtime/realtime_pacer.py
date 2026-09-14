"""Fixed physics timestep scheduling with bounded catch-up after slow rendering."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))
import math


class PhysicsPacer:
    def __init__(self, now, period=.02):
        if not math.isfinite(now) or not math.isfinite(period) or period<=0:
            raise ValueError('finite start and positive period required')
        self.deadline=now
        self.period=period

    def due(self, now):
        if now<self.deadline:return 0
        # A long pause must not replay seconds of stale drive input at once.
        if now-self.deadline>.5:self.deadline=now
        count=min(4,math.floor((now-self.deadline+1e-9)/self.period)+1)
        self.deadline+=count*self.period
        return count
