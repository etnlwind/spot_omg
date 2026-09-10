import mujoco
import numpy as np
from stow_clearance import LegClearance, UNRESTRICTED_FOLDED, DESIGN_CLEARANCE_M
from stow_policy import LANDING, FOLDED
from stow_preview import make_plant


def test_detailed_mesh_finds_old_overlap_and_new_powered_endpoint_margin():
    plant=make_plant(True,True);check=LegClearance(plant.model)
    assert len(check.pairs)==54
    def measure(angles):
        plant.data.qpos[plant.q]=np.radians(angles)
        mujoco.mj_forward(plant.model,plant.data)
        return check.measure(plant.data)[0]
    assert measure(UNRESTRICTED_FOLDED)==0
    assert measure(FOLDED)>=DESIGN_CLEARANCE_M
    # Check the whole nominal powered path, not only the endpoint.
    for fraction in np.linspace(0,1,101):
        assert measure(np.array(LANDING)+(np.array(FOLDED)-LANDING)*fraction)>=DESIGN_CLEARANCE_M
