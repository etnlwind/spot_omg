
# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))
import numpy as np
import pytest
from simulation.mujoco.runtime.centroidal_lateral_reference import CentroidalLateralReference


GOALS=np.array([[.2,.093,.05],[.2,-.093,.05],[-.2,.093,.05],[-.2,-.093,.05]])
COM=np.array([0.,0.,.25])


def test_periodic_reference_satisfies_its_stated_equation():
    reference=CentroidalLateralReference(GOALS,COM,1.35,.52,.096)
    reconstructed=reference.position-.2/9.81*reference.acceleration
    np.testing.assert_allclose(reconstructed,reference.cop,atol=1e-12)
    assert reference.at(0)==reference.at(1)
    assert reference.diagnostic['planned_within_10mm']==bool(abs(reference.position).max()<=.01)
    # Diagonal reversal should reverse the lateral reference.
    np.testing.assert_allclose(reference.position[:512],-reference.position[512:],atol=1e-12)


@pytest.mark.parametrize('period,stride,samples',[(float('nan'),.096,1024),(1.35,float('inf'),1024),(1.35,.096,0),(1.35,.096,32.5)])
def test_invalid_reference_input_is_rejected(period,stride,samples):
    with pytest.raises(ValueError):CentroidalLateralReference(GOALS,COM,period,.52,stride,samples)


def test_degenerate_support_line_is_rejected():
    points=GOALS.copy();points[:,0]=0
    with pytest.raises(ValueError):CentroidalLateralReference(points,COM,1.35,.52,.096)
