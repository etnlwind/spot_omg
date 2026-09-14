"""Stable resource locations, independent of working directory and module layout."""
from pathlib import Path
SIM_ROOT = Path(__file__).resolve().parent
REPO_ROOT = SIM_ROOT.parents[1]
RESULTS_ROOT = REPO_ROOT / 'artifacts/simulation/mujoco'
