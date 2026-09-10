import numpy as np
import pytest
from gait_profiles import foot_targets, load_profiles
from optimize_cad_gait import targets


def test_diagonal_path_matches_existing_search_math():
    params=load_profiles()['trot']['params']
    for t in np.linspace(0,2,101):
        np.testing.assert_allclose(foot_targets(params,t,.8),targets(params,t,.8),atol=1e-4)


@pytest.mark.parametrize('name',list(load_profiles()))
def test_profiles_periodic_and_neutral_is_stationary(name):
    p=load_profiles()[name];params=p['params']
    for t in np.linspace(0,params[0],30):
        a=foot_targets(params,t,family=p['family'])
        np.testing.assert_allclose(a,foot_targets(params,t+params[0],family=p['family']),atol=1e-9)
        neutral=foot_targets(params,t,family=p['family'],linear=0)
        np.testing.assert_allclose(neutral,foot_targets(params,0,family=p['family'],linear=0),atol=1e-10)


def test_invalid_profile_input_is_rejected():
    params=load_profiles()['cruise']['params']
    with pytest.raises(ValueError):foot_targets(params,0,linear=float('nan'))
    with pytest.raises(ValueError):foot_targets(params,0,linear=1.1)


@pytest.mark.parametrize('name', list(load_profiles()))
@pytest.mark.parametrize('linear,yaw', [(0, -.5), (0, .5), (.7, .5), (-.6, -.5)])
def test_turn_paths_remain_reachable_periodic_and_continuous(name, linear, yaw):
    profile = load_profiles()[name]
    params = profile['params']
    for phase in (0., profile['params'][1], 1.):
        t = phase * params[0]
        a = foot_targets(params, t, family=profile['family'], linear=linear, yaw=yaw)
        b = foot_targets(params, t + params[0], family=profile['family'], linear=linear, yaw=yaw)
        np.testing.assert_allclose(a, b, atol=1e-9)
        before = foot_targets(params, t - 1e-6, family=profile['family'], linear=linear, yaw=yaw)
        after = foot_targets(params, t + 1e-6, family=profile['family'], linear=linear, yaw=yaw)
        assert max(abs(after - before)) < .01
