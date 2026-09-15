
# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))
import numpy as np
import pytest
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import RobotController, load_parameters, parse_args
from simulation.mujoco.runtime.s_native_gait import SNativeGait, NAME, PROFILES

@pytest.fixture(scope='module')
def plant():
    return Simulation(load_parameters(parse_args([])))

@pytest.mark.parametrize('name,front,rear',[('s_native_v1',.020,.020),('s_native_v2',.020,.020),('s_native_v3',.020,.060),('s_native_v6',.020,.060),('s_native_v6_1',.020,.065)])
def test_diagonals_share_paths_and_opposing_extrema(plant,name,front,rear):
    g=SNativeGait(plant.model,plant.stand_target,PROFILES[name])
    for phase in np.linspace(0,1,101):
        delta=g.points(phase,1,1,0)-g.origin
        np.testing.assert_allclose(delta[0,[0,2]],delta[3,[0,2]],atol=1e-12)
        np.testing.assert_allclose(delta[1,[0,2]],delta[2,[0,2]],atol=1e-12)
        opposite=g.points(phase+.5,1,1,0)-g.origin
        np.testing.assert_allclose(delta[0,[0,2]],opposite[2,[0,2]],atol=1e-12)
    for phase,sign in ((0,1),(.5,-1)):
        d=g.points(phase,1,1,0)-g.origin
        assert d[2,0] == pytest.approx(front if sign>0 else -rear)
        assert d[3,0] == pytest.approx(-rear if sign>0 else front)

def test_s_native_default_direct_start_and_zero_motion(plant):
    r=RobotController(plant)
    assert r.profile==NAME
    q=plant.data.qpos.copy()
    r.command('drive 600 0 1',0)
    assert r.transition is None
    np.testing.assert_array_equal(q,plant.data.qpos)
    for phase in (0,.25,.5,.75):
        np.testing.assert_allclose(r.s_native_gait.targets(phase,0,1,0),r.stand_target,atol=.01)
        np.testing.assert_allclose(r.s_native_gait.targets(phase,1,0,0),r.stand_target,atol=.01)

@pytest.mark.parametrize('linear,yaw',[(1,0),(-1,0),(0,.5),(0,-.5),(1,.5)])
def test_native_contact_ik_reaches_path(plant,linear,yaw):
    g=SNativeGait(plant.model,plant.stand_target)
    # Reach the settled walking footprint before checking complete-cycle IK.
    for phase in np.linspace(.5,2.5,81):g.targets(phase%1,1,linear,yaw)
    for phase in np.linspace(0,1,41):
        target=g.targets(phase,1,linear,yaw);g.kin.set_angles(target)
        actual=np.array([g.kin.foot(i) for i in range(4)])
        reference=g.command_points(phase,1,linear,yaw)
        np.testing.assert_allclose(actual[:,[0,2]],reference[:,[0,2]],atol=.0002)
        # Body transfer is common to all feet; it cannot change pair timing.
        shift=actual[:,1]-reference[:,1]
        assert np.ptp(shift)<.0002

def test_sole_xy_tracks_one_material_point_without_vertex_jumps(plant):
    from simulation.mujoco.runtime.standing_pose import SoleKinematics
    k=SoleKinematics(plant.model,plant.stand_target)
    for joint in (0,1,2):
        positions=[]
        for offset in np.linspace(-3,3,601):
            q=plant.stand_target.copy();q[joint]+=offset
            k.set_angles(q);positions.append(k.foot(0)[:2])
        # 0.01 degree cannot physically move a point on these links by 0.5mm.
        assert np.max(np.linalg.norm(np.diff(positions,axis=0),axis=1)) < .00005

@pytest.mark.parametrize('name,rear', [('s_native_v6',.060), ('s_native_v6_1',.065)])
def test_body_transfer_keeps_opposing_extrema_and_s(plant,name,rear):
    g=SNativeGait(plant.model,plant.stand_target,PROFILES[name])
    for linear in (.3,.6,1.):
        path=np.array([g.command_points(t,1,linear,0)-g.origin for t in np.linspace(0,1,201)])
        np.testing.assert_allclose(path[:,:,0].max(axis=0),.020*linear,atol=1e-9)
        np.testing.assert_allclose(path[:,:,0].min(axis=0),-rear*linear,atol=1e-9)
        np.testing.assert_allclose(g.command_points(.25,0,linear,0),g.origin)


def test_v2_exchanges_without_a_ground_hold_and_keeps_v1_available(plant):
    from simulation.mujoco.runtime.s_native_gait import V2_PROFILE, V1_PROFILE
    current=SNativeGait(plant.model,plant.stand_target,V2_PROFILE)
    old=SNativeGait(plant.model,plant.stand_target,V1_PROFILE)
    # At 20ms after either exchange the next diagonal must already be lifting.
    for phase, pair in ((.025, (0,3)), (.525, (1,2))):
        delta=current.points(phase,1,1,0)-current.origin
        assert np.all(delta[list(pair),2]>.005)
        np.testing.assert_allclose((old.points(phase,1,1,0)-old.origin)[:,2],0.)
    assert V2_PROFILE['params'][0]*(.5-2*V2_PROFILE['transfer_fraction']) == pytest.approx(.4)
    robot=RobotController(plant)
    assert robot.profiles['s_native_v1']==V1_PROFILE
    assert robot.profiles['s_native_v2']==V2_PROFILE
    robot.command('syncstate',0)
    assert 's_native_v2' in robot.drain().decode()
    robot.profile='s_native_v1'
    robot.command('drive 600 0 1',0)
    assert robot.s_native_gait.profile==V1_PROFILE


@pytest.mark.parametrize('allow_fall,fault,keeps_walking',[(False,'tilt',False),(True,'tilt',True),(True,'imu',False)])
def test_fall_experiment_only_ignores_tilt_during_native_walk(allow_fall,fault,keeps_walking):
    plant=Simulation(load_parameters(parse_args(['--allow-fall'] if allow_fall else [])))
    robot=RobotController(plant)
    for i in range(30):robot.tick(i*.02)
    robot.command('drive 600 0 1',.6)
    robot.attitude_filter.update=lambda reading: fault
    robot.tick(.62)
    assert (robot.motion is not None)==keeps_walking
    assert robot.safety==('ok' if keeps_walking else fault)
    if keeps_walking:
        robot.command('@S 2',.64)
        # An interrupted first step completes the selected model's two
        # placements and the following one-second S hold.
        stop_seconds=robot.profiles[robot.profile]['stop_period_s']
        for i in range(round((stop_seconds+1.2)/.02)):robot.tick(.64+i*.02)
        assert robot.motion is None
        assert robot.transition is None


def test_v3_extends_push_behind_s_without_changing_front_placement_or_exchange(plant):
    old=SNativeGait(plant.model,plant.stand_target,PROFILES['s_native_v2'])
    current=SNativeGait(plant.model,plant.stand_target,PROFILES['s_native_v3'])
    assert current.profile['params'][0]==old.profile['params'][0]
    for linear in (.3,.6,1.):
        for phase in np.linspace(0,1,101):
            before=old.points(phase,1,linear,0)-old.origin
            after=current.points(phase,1,linear,0)-current.origin
            forward=before[:,0]>=0
            np.testing.assert_allclose(after[forward,0],before[forward,0],atol=1e-12)
            assert np.all(after[~forward,0]<=before[~forward,0]+1e-12)
            np.testing.assert_allclose(after[:,2],before[:,2],atol=1e-12)
        for phase,pair in ((.025,(0,3)),(.525,(1,2))):
            assert np.all((current.points(phase,1,linear,0)-current.origin)[list(pair),2]>.005)


def test_v4_walk_footprint_is_under_j1_without_changing_s_or_stride(plant):
    current=SNativeGait(plant.model,plant.stand_target,PROFILES["s_native_v4"])
    old=SNativeGait(plant.model,plant.stand_target,PROFILES['s_native_v3'])
    hips=np.array([current.kin.data.xanchor[plant.model.joint(l+'_j1').id,1] for l in ('fl','fr','rl','rr')])
    for phase in np.linspace(0,1,41):
        before=old.points(phase,1,1,0)
        after=current.points(phase,1,1,0)
        np.testing.assert_allclose(after[:,[0,2]],before[:,[0,2]],atol=1e-12)
        np.testing.assert_allclose(after[:,1],hips,atol=1e-12)
        np.testing.assert_allclose(current.points(phase,0,1,0),old.origin,atol=1e-12)


def test_v4_only_moves_entry_footprint_during_swing_and_reaches_both_pairs(plant):
    g=SNativeGait(plant.model,plant.stand_target,PROFILES["s_native_v4"])
    prev_fraction=np.zeros(4)
    previous=None
    for t in np.arange(0,2,.02):
        phase=(.25+t/.8)%1
        amplitude=plant.policy.smootherstep(min(t,1.))
        g.targets(phase,amplitude,1,0)
        q=(phase+np.array([.5,0,0,.5]))%1
        changed=g.placement_fraction-prev_fraction
        # A landing sample may complete the immediately preceding swing.
        stance=(q<.5)&((previous<.5) if previous is not None else True)
        np.testing.assert_allclose(changed[stance],0.,atol=1e-12)
        assert np.all(changed>=-1e-12)
        assert g.placement_fraction[0]==g.placement_fraction[3]
        assert g.placement_fraction[1]==g.placement_fraction[2]
        if t<.5:np.testing.assert_array_equal(g.placement_fraction,0.)
        prev_fraction=g.placement_fraction.copy();previous=q
    np.testing.assert_array_equal(g.placement_fraction,1.)
    # Nominal horizontal footprint width equals the distance between J1 axes.
    np.testing.assert_allclose(g.last_target_points[0,1]-g.last_target_points[1,1],.0782385895,atol=.0001)


@pytest.mark.parametrize('name', ['s_native_v5', 's_native_v6', 's_native_v6_1'])
def test_first_fr_rl_swing_and_double_fr_angular_adduction(plant, name):
    g=SNativeGait(plant.model,plant.stand_target,PROFILES[name])
    assert g.profile['start_phase']==.5
    for p in np.linspace(.5,1.,51):
        q=g.targets(p%1,1.,1.,0.)
        np.testing.assert_allclose(q[[0,6,9]],plant.stand_target[[0,6,9]],atol=1e-9)
        # A different J1 must not change the shared FR/RL X/Z displacement.
        delta=g.last_target_points-g.origin
        np.testing.assert_allclose(delta[1,[0,2]],delta[2,[0,2]],atol=.00004)
        if .5<p<1.:
            assert delta[1,2]>.001
            np.testing.assert_allclose(delta[[0,3],2],0.,atol=.00004)
    assert q[3]-plant.stand_target[3]==pytest.approx(2*g.normal_adduction[1],abs=1e-8)
    np.testing.assert_allclose(g.placement_fraction,[0,2,0,0],atol=1e-9)
    previous=g.placement_fraction.copy()
    for p in np.linspace(1.01,1.5,50):
        g.targets(p%1,1.,1.,0.)
        assert g.placement_fraction[1]<=previous[1]+1e-9
        assert np.all(g.placement_fraction[[0,2,3]]>=previous[[0,2,3]]-1e-9)
        previous=g.placement_fraction.copy()
    np.testing.assert_allclose(g.placement_fraction,1.,atol=1e-9)


@pytest.mark.parametrize('progress',[.25,.6,1.2])
@pytest.mark.parametrize('name', ['s_native_v5', 's_native_v6', 's_native_v6_1'])
def test_stop_reverses_adduction_continuously_to_s(plant,progress,name):
    g=SNativeGait(plant.model,plant.stand_target,PROFILES[name])
    for p in np.arange(0,progress,.025):g.targets((.5+p)%1,1.,1.,0.)
    start=g.previous.copy();g.begin_stop()
    previous=abs(start[::3]-plant.stand_target[::3])
    for tick in range(round(g.profile['params'][0]/.02)+1):
        command=max(0.,1-(tick+1)/25)
        q=g.targets((.5+progress+(tick+1)*.025)%1,1.,command,0.)
        inward=abs(q[::3]-plant.stand_target[::3])
        assert np.all(inward<=previous+1e-8)
        assert np.max(abs(q[::3]-g.stop_j1))<30
        if tick==0:assert np.max(abs(q[::3]-start[::3]))<.01
        previous=inward
    assert g.stop_ready
    np.testing.assert_allclose(q,plant.stand_target,atol=.01)
    np.testing.assert_allclose(g.placement_fraction,0.,atol=1e-9)


@pytest.mark.parametrize('phase',[.18,.68])
def test_v61_stop_places_two_diagonals_at_planned_clearance(plant,phase):
    g=SNativeGait(plant.model,plant.stand_target,PROFILES['s_native_v6_1'])
    for p in np.arange(.5,2+phase,.02):g.targets(p%1,1.,.6,0.)
    g.targets(phase,1.,.6,0.)
    g.begin_stop()
    first=g.stop_first;second=np.array([i for i in range(4) if i not in first])
    start=g.previous.reshape(4,3).copy();old=start.copy()
    for tick in range(60):
        # Zero drive input must not collapse the two placement lifts.
        q=g.targets((phase+.02*tick)%1,1.,0.,0.).reshape(4,3)
        changed=abs(q[:,0]-old[:,0])>1e-6
        assert np.all(g.last_target_points[changed,2]>=g.origin[changed,2]+.0119)
        if tick<30:
            np.testing.assert_allclose(q[second],start[second],atol=1e-10)
        if tick>=21:
            np.testing.assert_allclose(q[first],plant.stand_target.reshape(4,3)[first],atol=.01)
        if tick>=12:
            np.testing.assert_allclose(q[first,0],plant.stand_target.reshape(4,3)[first,0],atol=1e-9)
        if tick>=42:
            np.testing.assert_allclose(q[:,0],plant.stand_target[::3],atol=1e-9)
        old=q.copy()
    assert g.stop_ready
    np.testing.assert_allclose(q.ravel(),plant.stand_target,atol=.01)
    # A completed placement never opens farther or resumes its oscillator.
    for p in (.1,.6,.9):
        np.testing.assert_allclose(g.targets(p,1.,0.,0.),plant.stand_target,atol=.01)
