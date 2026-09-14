import json
from pathlib import Path
from types import SimpleNamespace
import mujoco
import numpy as np
import pytest
from cad_physics import build
from shared_swing_ground import SharedSwingGround


@pytest.fixture(scope='module')
def model():
    root=Path(__file__).parent
    p=json.loads((root/'cad_300mm/physics_parameters_measured_total_2754g.json').read_text())
    p['foot_cushion']=json.loads((root/'foot_cushion_10mm.json').read_text())
    xml,_=build(p,write_scene=False)
    return mujoco.MjModel.from_xml_string(xml)


def test_equal_scalar_target_and_stance_preservation(model):
    control=SharedSwingGround(model)
    q=np.tile([0.,45.,90.],4)
    imu=SimpleNamespace(filtered=[20.,-10.],failures=0)
    for _ in range(12):
        out=control.apply(q,q,imu,.82,.7,3.2,1.,.5)
    diag=control.diagnostic
    assert diag['equal_scalar_clearance_m'][0] == pytest.approx(.032)
    assert diag['equal_scalar_clearance_m'][0] == diag['equal_scalar_clearance_m'][3]
    assert diag['full_shared_target_world_z_m'][0] == diag['full_shared_target_world_z_m'][3]
    np.testing.assert_array_equal(out.reshape(4,3)[[1,2]],q.reshape(4,3)[[1,2]])
    assert diag['stance_preserved']
    assert max(abs(out-q)) <= 6.+1e-9
    assert max(diag['body_xy_residual_m']) < .0003


def test_slew_and_swing_boundary_return(model):
    control=SharedSwingGround(model)
    q=np.tile([0.,45.,90.],4)
    imu=SimpleNamespace(filtered=[30.,-10.],failures=0)
    previous=np.zeros(12)
    for phase in np.arange(.65,1.051,.02/3.2):
        out=control.apply(q,q,imu,phase,.7,3.2,1.,.5)
        correction=out-q
        assert max(abs(correction)) <= 6.+1e-9
        assert max(abs(correction-previous)) <= .400001
        previous=correction
        if phase>=1:
            np.testing.assert_array_equal(out,q)


def test_missing_invalid_imu_or_reference_releases_with_slew(model):
    control=SharedSwingGround(model)
    q=np.tile([0.,45.,90.],4)
    good=SimpleNamespace(filtered=[0.,0.],failures=0)
    for enc,imu in [(None,good),(q,None),(q,SimpleNamespace(filtered=[np.nan,0],failures=0)),
                    (q,SimpleNamespace(filtered=[0,0],failures=1))]:
        control.correction[:]=2.
        output=control.apply(q,enc,imu,.82,.7,3.2,1.)
        np.testing.assert_allclose(output-q,np.full(12,1.6))
        assert not control.diagnostic['enabled']
        assert control.diagnostic['max_correction_step_deg'] <= .400001
        assert not control.diagnostic['stance_preserved']
        for _ in range(4):output=control.apply(q,enc,imu,.82,.7,3.2,1.)
        np.testing.assert_allclose(output,q,atol=1e-10)
    # Scheduled support heights do not describe one floor when one reported
    # support leg is bent by a large amount; reject rather than guess.
    bad=q.copy();bad[4:6]=[70,130]
    np.testing.assert_array_equal(control.apply(q,bad,good,.82,.7,3.2,1.),q)
    assert control.diagnostic['reason']=='scheduled-floor-reference-invalid'


def test_reject_nonfinite_and_out_of_limit_targets(model):
    control=SharedSwingGround(model);q=np.tile([0.,45.,90.],4)
    imu=SimpleNamespace(filtered=[0.,0.],failures=0)
    invalid=q.copy();invalid[0]=31
    with pytest.raises(ValueError):control.apply(invalid,q,imu,.82,.7,3.2,1.)
    with pytest.raises(ValueError):control.apply(q,q,imu,.82,.7,np.nan,1.)


def test_phase_or_nominal_jump_enters_fallback(model):
    control=SharedSwingGround(model);q=np.tile([0.,45.,90.],4)
    imu=SimpleNamespace(filtered=[0.,0.],failures=0)
    control.apply(q,q,imu,.82,.7,3.2,1.)
    control.correction[:]=2.
    out=control.apply(q,q,imu,.1,.7,3.2,1.)
    assert control.diagnostic['reason']=='phase-discontinuity'
    np.testing.assert_allclose(out-q,np.full(12,1.6))
    jumped=q.copy();jumped[1]+=20
    out=control.apply(jumped,q,imu,.11,.7,3.2,1.)
    assert control.diagnostic['reason']=='nominal-discontinuity'
    np.testing.assert_allclose(out-q,np.full(12,1.2))


def test_invalid_sensor_plus_nominal_jump_near_limit_keeps_fallback_slew(model):
    control=SharedSwingGround(model)
    base=np.tile([0.,45.,90.],4);base[1]=95.
    control.last_nominal=base.copy()
    control.correction[1]=5.
    control.last_output=base+control.correction
    requested=base.copy();requested[1]=99.;requested[4]+=20.
    out=control.apply(requested,None,None,.82,.7,3.2,1.)
    assert out[1] == pytest.approx(99.6)
    assert out[4] == base[4]
    assert control.diagnostic['max_correction_step_deg'] <= .400001
    assert control.diagnostic['max_output_step_deg'] <= .400001
    assert np.all(out<=control.high) and np.all(out>=control.low)
    for _ in range(12):out=control.apply(requested,None,None,.82,.7,3.2,1.)
    np.testing.assert_array_equal(out,base)
