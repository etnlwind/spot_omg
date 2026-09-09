import math
import numpy as np
import pytest
from compare_stance import shift_height
from servo import SharedGaitPolicy
from cad_physics import KEYS


def fk(a):
    a=np.radians(np.array(a).reshape(4,3))
    return np.sin(a[:,1])+np.sin(a[:,1]-a[:,2]),np.cos(a[:,1])+np.cos(a[:,1]-a[:,2])


def test_coupled_neutral_and_preserved_foot_path():
    p=SharedGaitPolicy()
    for hip in (25,35,40,45,55):
        neutral=np.tile([0,45,90],4)
        actual=shift_height(neutral,hip).reshape(4,3)
        np.testing.assert_allclose(actual[:,1],hip,atol=1e-8)
        np.testing.assert_allclose(actual[:,2],2*hip,atol=1e-8)
        for phase in np.linspace(0,1,101):
            targets,_=p.drive_stride_targets(float(phase),1,1,0,1.6)
            a=np.array([targets[k] for k in KEYS]);b=shift_height(a,hip)
            x,z=fk(a);xx,zz=fk(b)
            np.testing.assert_allclose(x,xx,atol=1e-9)
            np.testing.assert_allclose(zz-z,2*(math.cos(math.radians(hip))-math.cos(math.pi/4)),atol=1e-9)
            if hip==45:np.testing.assert_allclose(a,b,atol=1e-8)


def test_overextended_trajectory_is_rejected():
    with pytest.raises(ValueError):shift_height(np.tile([0,0,0],4),0)
