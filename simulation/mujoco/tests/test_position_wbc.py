
# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))
import numpy as np
from simulation.mujoco.runtime.position_wbc import EncoderChannel, priority_solve


def test_encoder_feedback_has_delay_and_quantization():
    channel=EncoderChannel()
    assert channel.read(np.ones(12)*1.01) is None
    assert channel.read(np.ones(12)*2.01) is None
    sample=channel.read(np.ones(12)*3.01)
    np.testing.assert_allclose(sample,np.round(1.01*4096/360)*360/4096)


def test_contact_priority_cannot_be_overridden_by_orientation():
    contact=np.array([[1.,1.,0.]])
    orientation=np.array([[1.,0.,0.]])
    q,_=priority_solve(contact,orientation,np.array([.1]))
    np.testing.assert_allclose(contact@q,0,atol=1e-12)
    np.testing.assert_allclose(orientation@q,.1,atol=1e-12)


def test_impossible_orientation_does_not_break_contact():
    contact=np.array([[1.,0.,0.]])
    q,_=priority_solve(contact,contact,np.array([.1]))
    np.testing.assert_allclose(contact@q,0,atol=1e-12)
