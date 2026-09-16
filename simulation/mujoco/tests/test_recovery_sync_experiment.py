"""Footprint, entry and isolation contracts for the offline recovery candidate."""
import copy
import json
from pathlib import Path

import numpy as np
import pytest
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import RobotController, load_parameters, parse_args
from simulation.mujoco.runtime.s_native_gait import SNativeGait, PROFILES
from simulation.mujoco.scripts.analysis.analyze_knee_liftoff import tuck_experiment
from simulation.mujoco.scripts.analysis.analyze_j1_steady_hold import j1_experiment

ROOT=Path(__file__).resolve().parents[3]
CONFIG=json.loads((ROOT/'config/experiments/post_push_j2_j3_synchronized.json').read_text())
MONOTONIC=json.loads((ROOT/'config/experiments/post_push_j2_j3_monotonic.json').read_text())


@pytest.fixture(scope='module')
def plant():
    return Simulation(load_parameters(parse_args([])))


@pytest.mark.parametrize('config',[CONFIG,MONOTONIC])
def test_entry_shared_diagonals_stance_and_stop(plant,config):
    baseline=copy.deepcopy(PROFILES)
    with tuck_experiment(**config['trajectory']),j1_experiment('fixed_j1'):
        gait=SNativeGait(plant.model,plant.stand_target,copy.deepcopy(PROFILES['s_native_v6_2_6']))
        gait.prepare_support(1.,0.)
        previous=None
        for phase in np.linspace(.5,2.5,1001):
            target=gait.targets(phase%1,1.,1.,0.)
            feet=gait.last_target_points-gait.origin
            np.testing.assert_allclose(feet[0,[0,2]],feet[3,[0,2]],atol=3e-5)
            np.testing.assert_allclose(feet[1,[0,2]],feet[2,[0,2]],atol=3e-5)
            if phase==1.:
                np.testing.assert_allclose(target[::3]-gait.standing[::3],[0,-18,0,0],atol=1e-5)
            if phase>=1.5:
                np.testing.assert_allclose(target[::3]-gait.standing[::3],[-9]*4,atol=1e-5)
                leg_phase=(phase+np.array([.5,0,0,.5]))%1
                stance=leg_phase<.5
                u=leg_phase[stance]*2
                smooth=u**3*(10+u*(-15+6*u))
                # Existing common fore/aft body transfer is retained as well.
                common_shift=-.006*(1-(1-2*smooth)**2)
                np.testing.assert_allclose(feet[stance,0],.020-.145*smooth+common_shift,atol=3e-5)
                np.testing.assert_allclose(feet[stance,2],0.,atol=3e-5)
                if previous is not None:
                    assert np.max(abs(target-previous))<1.5
            previous=target.copy()
        gait.begin_stop()
        for _ in range(100):gait.targets(.5,1.,0.,0.)
        assert gait.stop_ready
        np.testing.assert_allclose(gait.previous,gait.standing,atol=1e-9)
    assert PROFILES==baseline


def test_experiment_restores_methods_and_profile_after_exception(plant):
    methods=(SNativeGait.points,SNativeGait.fr_entry_targets,RobotController.select_profile)
    profiles=copy.deepcopy(PROFILES)
    with pytest.raises(RuntimeError,match='intentional'):
        with tuck_experiment(**CONFIG['trajectory']),j1_experiment('fixed_j1'):
            raise RuntimeError('intentional')
    assert methods==(SNativeGait.points,SNativeGait.fr_entry_targets,RobotController.select_profile)
    assert profiles==PROFILES
    robot=RobotController(plant)
    assert robot.profile=='s_native_v6_2_7'
    assert robot.profiles['s_native_v6_2_6']['params']==profiles['s_native_v6_2_6']['params']


def test_vertical_projection_removes_early_recovery_reversal(plant):
    from simulation.mujoco.scripts.analysis.audit_recovery_geometry import sample_geometry,summarize
    old=summarize(sample_geometry(plant,CONFIG,251))
    rows=sample_geometry(plant,MONOTONIC,251)
    new=summarize(rows)
    # This detects the observed forward/backward reversal, not just a zero clock rate.
    assert all(r['reverse_travel_mm']>1 for r in old.values())
    assert all(r['reverse_travel_mm']==0 for r in new.values())
    assert max(r['j3_command_range_deg'][1] for r in new.values())<120
    early=[r for r in rows if r['cycle_fraction']<=.08]
    assert early[-1]['target_deg'][7]>early[0]['target_deg'][7]
    assert early[-1]['target_deg'][8]>early[0]['target_deg'][8]
    assert early[-1]['knee_proxy_deg'][2]<early[0]['knee_proxy_deg'][2]
