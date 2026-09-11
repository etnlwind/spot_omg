import numpy as np
from types import SimpleNamespace
from support_j1 import SupportJ1


def test_disabled_is_exact_passthrough_and_resets():
    c=SupportJ1();c.delta[:]=3;c.scale=.6
    target=np.tile([1.,45.,90.],4)
    assert np.array_equal(c.apply(target,target,None,0,.6,{},False),target)
    assert np.all(c.delta==0) and c.scale==1


def test_correction_is_bounded_nonaccumulating_and_preserves_xz():
    c=SupportJ1();target=np.tile([.75,45.,90.],4)
    attitude=SimpleNamespace(filtered=[30.,0.],rate=[0.,0.])
    for _ in range(100):result=c.apply(target,target,attitude,.2,.6,{'gain':1,'damping':0,'limit_deg':3},True)
    assert max(abs(result[::3]-target[::3]))<=3.000001
    def xz(q):
        a,u,k=np.radians(q.reshape(4,3)).T
        return np.array([.141*np.sin(u)+.150*np.sin(u-k),(.141*np.cos(u)+.150*np.cos(u-k))*np.cos(a)])
    np.testing.assert_allclose(xz(result),xz(target),atol=1e-10)
    assert result[0]>target[0] and result[3]<target[3]


def test_large_attitude_reduces_stride_and_recovers_slowly():
    c=SupportJ1();target=np.tile([0.,45.,90.],4)
    for _ in range(80):c.apply(target,target,SimpleNamespace(filtered=[100.,0.],rate=[0.,0.]),.2,.6,{},True)
    assert .55<=c.scale<.8
    previous=c.scale
    c.apply(target,target,SimpleNamespace(filtered=[0.,0.],rate=[0.,0.]),.2,.6,{},True)
    assert 0<c.scale-previous<=.003001
