"""Stable launcher; implementation lives in runtime/."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from simulation.mujoco.runtime.walk import main
if __name__ == '__main__':
    main()
