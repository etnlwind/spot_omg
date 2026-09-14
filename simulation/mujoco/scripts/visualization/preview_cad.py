"""Display the imported STEP assembly, without unverified joint physics."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import argparse
import time
from pathlib import Path
import mujoco
import mujoco.viewer


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    scene = SIM_ROOT / "cad_300mm" / "scene.xml"
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    print(f"New STEP CAD: {model.nmesh} meshes, {model.ngeom} geoms; static preview, no joint physics", flush=True)
    if not args.check:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.lookat[:] = model.stat.center
            viewer.cam.distance = model.stat.extent * 1.9
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -20
            while viewer.is_running():
                viewer.sync()
                time.sleep(1 / 30)


if __name__ == "__main__":
    main()
