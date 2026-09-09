"""The shipped C gait must match the selected simulation profile."""
import json
import numpy as np
import pytest
from servo import SharedGaitPolicy
from servo.cli import parse_args, console_line_for
from servo.console import estimate_timeout
from optimize_cad_gait import RESULTS,NAMES,targets


def test_shared_c_matches_selected_profile_over_full_cycle_and_startup():
    params=json.loads((RESULTS/'selected.json').read_text())['params']
    p=np.array([params[k] for k in NAMES]);policy=SharedGaitPolicy()
    for scale in (0.,.2,.5,1.):
        for phase in np.linspace(0,1,301):
            actual,support=policy.trot5_targets(float(phase),scale)
            expected=targets(p,phase*p[0],scale)
            values=np.array([actual[(leg,j)] for leg in ('FL','FR','RL','RR') for j in (1,2,3)])
            np.testing.assert_allclose(values,expected,atol=.0002,rtol=0)
            if scale==0:assert len(support)==4
            else:assert support in ({'FL','RR'},{'FR','RL'},{'FL','FR','RL','RR'})


@pytest.mark.parametrize('phase,scale',[(float('nan'),1),(float('inf'),1),(0,-.1),(0,1.1),(0,float('nan'))])
def test_shared_c_rejects_invalid_frames(phase,scale):
    with pytest.raises(ValueError):SharedGaitPolicy().trot5_targets(phase,scale)


def test_trot5_cli_uses_motion_timeout_and_validates_cycles_and_period():
    assert console_line_for(parse_args(['trot5','3','844']))=='trot5 3 844'
    assert estimate_timeout('trot5')>=22.5
    for args in (['trot5','0'],['trot5','11'],['trot5','3','600'],['trot5','3','2401']):
        with pytest.raises(ValueError):console_line_for(parse_args(args))
