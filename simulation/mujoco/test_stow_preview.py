import numpy as np
from stow_preview import make_plant, STAGES


def test_experimental_limits_do_not_modify_normal_robot():
    extended=make_plant(True)
    nominal=make_plant(False)
    for leg in ('fl','fr','rl','rr'):
        np.testing.assert_allclose(np.degrees(nominal.model.jnt_range[nominal.model.joint(leg+'_j2').id]),[-45,100])
        np.testing.assert_allclose(np.degrees(extended.model.jnt_range[extended.model.joint(leg+'_j2').id]),[-95,105])


def test_requested_front_rotation_and_back_legs():
    before=np.array(STAGES[2][1]).reshape(4,3)
    after=np.array(STAGES[3][1]).reshape(4,3)
    np.testing.assert_array_equal(after[:2,1]-before[:2,1],[180,180])
    np.testing.assert_array_equal(after[2:],before[2:])


def test_no_direct_pose_changes_when_requesting_stow():
    plant=make_plant(True)
    before=plant.data.qpos.copy()
    plant.step(targets_deg=STAGES[-1][1],balance=False)
    # A command advances dynamics; it must not teleport to the requested pose.
    assert plant.data.time > 0
    assert not np.allclose(np.degrees(plant.data.qpos[plant.q]),STAGES[-1][1])
    assert np.all(np.isfinite(plant.data.qpos))
    assert not np.array_equal(before,plant.data.qpos)


def test_overhead_path_passes_above_hip_and_ends_at_same_orientation():
    import mujoco
    from stow_preview import stages_for
    plant=make_plant(True,overhead=True)
    stages=stages_for('overhead')
    before=np.array(stages[2][1]);after=np.array(stages[3][1])
    np.testing.assert_array_equal((after-before)[[1,4]],[-180,-180])
    positions=[]
    for values in ((before+after)/2,after,np.array(STAGES[3][1])):
        plant.data.qpos[plant.q]=np.radians(values)
        mujoco.mj_forward(plant.model,plant.data)
        positions.append(plant.data.geom_xpos[plant.model.geom('fl_foot').id].copy())
        if len(positions)==1:
            hip=plant.data.xpos[plant.model.body('fl_j2_link').id]
            assert positions[0][2]-hip[2] > .2
    np.testing.assert_allclose(positions[1],positions[2],atol=1e-6)
    assert plant.p['embedded_servo_quantization'] is False
    assert make_plant(True).p.get('embedded_servo_quantization',True) is True


def test_simultaneous_fold_moves_front_and_rear_on_same_slow_timeline():
    from stow_preview import stages_for
    stages=stages_for('simultaneous')
    assert len(stages)==3
    assert stages[1][2] == 12
    start=np.array(stages[0][1]);end=np.array(stages[1][1])
    delta=(end-start).reshape(4,3)
    assert np.all(delta[:,1:] < 0)
    plant=make_plant(True,overhead=True)
    for fraction in (.01,.25,.5,.75,.99):
        t=plant.policy.smootherstep(fraction)
        command=start+(end-start)*t
        moving=np.flatnonzero(end-start)
        np.testing.assert_allclose((command-start)[moving]/(end-start)[moving],t)
    np.testing.assert_array_equal(stages[1][1],stages[2][1])


def test_selected_stow_cycle_unfolds_along_reverse_command_path():
    from stow_preview import stages_for
    cycle=stages_for('stow-cycle')
    assert cycle[:3] == stages_for('simultaneous')
    assert cycle[1][2] == cycle[3][2] == 12
    landing=np.array(cycle[0][1],float)
    folded=np.array(cycle[1][1],float)
    np.testing.assert_array_equal(cycle[3][1],landing)
    np.testing.assert_array_equal(cycle[4][1],landing)
    p=make_plant(True,overhead=True).policy
    for fraction in (0,.1,.25,.5,.9,1):
        unfold=folded+(landing-folded)*p.smootherstep(fraction)
        fold=landing+(folded-landing)*p.smootherstep(1-fraction)
        # Shared C smootherstep uses float32; allow sub-millidegree rounding.
        np.testing.assert_allclose(unfold,fold,atol=1e-3,rtol=0)
