import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from cad_physics import Simulation
from pivot_hold import PivotHold

def test_proprioceptive_translation_estimate_and_bounded_correction():
    p=json.loads(Path(__file__).with_name('measured_response_plant.json').read_text())
    h=PivotHold(Simulation(p).model);q=np.tile([0.,45.,90.],4)
    attitude=SimpleNamespace(filtered=[0,0],failures=0)
    reading={'yaw_tenths':0};config={'gain':.5,'limit_m':.03}
    np.testing.assert_allclose(h.apply(q,q,attitude,reading,.03,.6,config,True),q,atol=1e-6)
    h.kin.set_angles(q);points=np.array([h.kin.foot(i) for i in range(4)])
    points[:,0]-=.01
    displaced,residual=h.kin.solve(points,q,iterations=12)
    assert residual<.0001
    for _ in range(12):
        h.apply(q,displaced,attitude,reading,.03,.6,config,True)
    assert .008<h.diagnostic['estimated_center_error_m'][0]<.012
    assert 0<h.delta[0]<.006
    np.testing.assert_allclose(h.apply(q,None,attitude,None,.03,.6,config,True),q)
    assert np.all(h.delta==0)
