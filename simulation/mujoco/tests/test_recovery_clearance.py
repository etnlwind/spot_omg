"""Verify mesh-floor constraints independently of the search residual."""
import mujoco
import numpy as np
import pytest

from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import load_parameters, parse_args
from simulation.mujoco.scripts.analysis.solve_recovery_clearance import CushionClearance


@pytest.fixture(scope='module')
def solver():
    return CushionClearance(Simulation(load_parameters(parse_args([]))))


@pytest.mark.parametrize('leg', range(4))
def test_minimum_uses_rotated_lowest_cushion_vertex(solver, leg):
    original=solver.walking_reference.copy()
    result=solver.solve(original,leg,5.,60.)
    pose=original.copy()
    pose[solver.q[leg*3+1]]=np.radians(60.)
    pose[solver.q[leg*3+2]]=np.radians(result['j3_deg'])
    data=mujoco.MjData(solver.model)
    data.qpos[:]=pose
    mujoco.mj_kinematics(solver.model,data)
    geom=solver.feet[leg]
    vertices=solver.vertices[leg]@data.geom_xmat[geom].reshape(3,3).T+data.geom_xpos[geom]
    assert vertices[:,2].min()==pytest.approx(.005,abs=1e-9)
    pose[solver.q[leg*3+2]]-=np.radians(.1)
    assert solver.point(pose,leg)[2]<.005
    np.testing.assert_array_equal(original,solver.walking_reference)


def test_body_sink_changes_required_flexion(solver):
    level=solver.solve(solver.walking_reference,3,5.,60.)
    lowered=solver.walking_reference.copy()
    lowered[2]-=.005
    sink=solver.solve(lowered,3,5.,60.)
    margin=solver.solve(solver.walking_reference,3,10.,60.)
    assert sink['j3_deg']>level['j3_deg']
    assert sink['j3_deg']==pytest.approx(margin['j3_deg'],abs=1e-6)


def test_unreachable_clearance_is_explicit(solver):
    assert solver.solve(solver.walking_reference,3,500.,60.) is None


def test_unfolding_while_lowering_example_clears_floor(solver):
    # A geometric example, not a physically validated gait or actuator timing.
    for u in np.linspace(0,1,81):
        pose=solver.walking_reference.copy()
        for leg in range(4):
            pose[solver.q[leg*3+1]]=np.radians(85.-40*u)
            pose[solver.q[leg*3+2]]=np.radians(104.-4*u)
            assert solver.point(pose,leg)[2]>.005
