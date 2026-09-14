
# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))
import numpy as np
import pytest
from simulation.mujoco.runtime.support_sweep import fore_aft_path


def path(phase):
    return np.array(fore_aft_path(phase,.52,1.35,.020,-.076))


def test_contact_endpoints_and_constant_stance_speed():
    np.testing.assert_allclose(path(0),[.020,-.096/(1.35*.52),0.])
    np.testing.assert_allclose(path(.52),[-.076,-.096/(1.35*.52),0.])
    for q in np.linspace(0,.52,50):
        x,v,a=path(q)
        assert x==pytest.approx(.020-.096*q/.52)
        assert v==pytest.approx(-.096/(1.35*.52))
        assert a==pytest.approx(0.)


@pytest.mark.parametrize('boundary',[0.,.52])
def test_c2_contact_boundaries(boundary):
    np.testing.assert_allclose(path(boundary-1e-8),path(boundary+1e-8),atol=1e-6)


def test_analytic_velocity_and_acceleration_and_visible_swing_excursion():
    h=1e-6
    for q in np.linspace(.001,.999,100):
        before=path(q-h);after=path(q+h);x,v,a=path(q)
        assert (after[0]-before[0])/(2*h*1.35)==pytest.approx(v,abs=1e-8)
        assert (after[1]-before[1])/(2*h*1.35)==pytest.approx(a,abs=1e-7)
    sample=np.array([path(q) for q in np.linspace(0,1,1001)])
    # Nonzero backward contact velocity requires overshoot outside the two
    # contact endpoints. The forward limit must not be described as 20mm.
    assert .020<sample[:,0].max()<.030
    assert -.086<sample[:,0].min()<-.076


@pytest.mark.parametrize('args',[(0,.52,0,.02,-.076),(0,1,1,.02,-.076),(0,.5,1,0,.1),(float('nan'),.5,1,.02,-.076)])
def test_invalid_parameters(args):
    with pytest.raises(ValueError):fore_aft_path(*args)
