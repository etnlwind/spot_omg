"""Flat-ground two-point support placement from a planned COM acceleration.

With centroidal angular-momentum rate tau, Fz=m*(g+a_z),
P_xy=C_xy-h*a_xy/(g+a_z)+[-tau_y,tau_x]/Fz.
The underlying force/moment equations are described by MIT Underactuated
Robotics, https://underactuated.csail.mit.edu/humanoids.html .

This pure calculation has no robot, sensor or simulator-state access. All
vectors must share one gravity-aligned frame and one predicted contact epoch.
Acceleration must be a declared plan or delivered-sensor estimate, not an
unlabelled simulator oracle. The returned feasibility applies only to point
contacts on one plane, not to leg reach, motors, finite pads or tracking.
"""
import numpy as np


def acceleration_support_targets(com_xy_m, height_m, acceleration_xy_m_s2,
                                 feet_xy_m, *, mass_kg=2.754,
                                 acceleration_z_m_s2=0., mode='minimumXY',
                                 gravity_m_s2=9.81, friction_coefficient=None,
                                 centroidal_moment_xy_nm=(0.,0.)):
    """Translate both predicted foot points equally to contain desired CoP.

`minimumXY` finds the nearest point on the original support segment and the
minimum common 2-D translation to place that point at desired CoP. `x-only`
keeps both Y coordinates fixed. An out-of-segment x-only construction is
reported infeasible with its candidate/negative reaction retained for audit;
callers must check `feasible` before applying it. No displacement is silently
clipped and no input acceleration is reduced to make a test pass.

The requested centroidal moment is [tau_x,tau_y,0], in Nm, in the same
gravity-aligned frame. It is angular-momentum rate, not a joint torque command.
Start from tangential forces proportional to normal force, then add the
smallest equal-and-opposite tangential force needed to remove residual yaw
moment. Coulomb friction is checked per foot for that explicit allocation;
norm(a_xy)/(g+a_z) is only a net-force lower bound when tau is nonzero.
No friction optimization is claimed. If no coefficient is supplied, actual
friction feasibility is unknown and is explicitly returned as None.
"""
    if mode not in ('x-only', 'minimumXY'):
        raise ValueError('mode must be x-only or minimumXY')
    result=dict(feasible=False, geometry_feasible=False, friction_feasible=None,
        mode=mode, flags=[], desired_cop_xy_m=None, translation_xy_m=None,
        targets_xy_m=None, support_weights=None, normal_forces_n=None,
        contact_forces_xyz_n=None, required_friction_coefficient=None,
        limitations='Flat coplanar point contacts; declared Hdot_xy with Hdot_z=0; explicit force allocation, no reach/torque/speed/pad/impact constraints.')
    try:
        center=np.asarray(com_xy_m,dtype=float)
        acceleration=np.asarray(acceleration_xy_m_s2,dtype=float)
        feet=np.asarray(feet_xy_m,dtype=float)
        torque=np.asarray(centroidal_moment_xy_nm,dtype=float)
        height,mass,az,gravity=map(float,(height_m,mass_kg,acceleration_z_m_s2,gravity_m_s2))
    except (TypeError,ValueError):
        result['flags'].append('invalid-input-type')
        return result
    if center.shape!=(2,) or acceleration.shape!=(2,) or feet.shape!=(2,2) or torque.shape!=(2,):
        result['flags'].append('invalid-input-shape')
        return result
    if not np.isfinite(np.r_[center,acceleration,feet.ravel(),torque,height,mass,az,gravity]).all():
        result['flags'].append('nonfinite-input')
        return result
    if height<=0 or mass<=0 or gravity<=0:
        result['flags'].append('nonpositive-height-mass-or-gravity')
        return result
    effective_gravity=gravity+az
    if effective_gravity<=1e-9:
        result['flags'].append('nonpositive-normal-acceleration')
        return result
    mu=None
    if friction_coefficient is not None:
        try:mu=float(friction_coefficient)
        except (TypeError,ValueError):mu=np.nan
        if not np.isfinite(mu) or mu<0:
            result['flags'].append('invalid-friction-coefficient')
            return result
    normal_force=mass*effective_gravity
    desired=center-height*acceleration/effective_gravity+np.array([-torque[1],torque[0]])/normal_force
    net_friction=float(np.linalg.norm(acceleration)/effective_gravity)
    target_moment=np.r_[torque,0.]
    result.update(desired_cop_xy_m=desired.tolist(),net_force_required_friction_coefficient=net_friction,
        requested_moment_about_com_nm=target_moment.tolist(),
        total_normal_force_n=normal_force,effective_gravity_m_s2=effective_gravity)
    direction=feet[1]-feet[0]
    length2=float(direction@direction)
    if length2<=1e-16:
        result['flags'].append('degenerate-support-segment')
        return result
    unconstrained=float((desired-feet[0])@direction/length2)
    if mode=='minimumXY':
        fraction=float(np.clip(unconstrained,0.,1.))
        translation=desired-(feet[0]+fraction*direction)
    elif abs(direction[1])<=1e-10:
        # A horizontal segment is not degenerate. If its Y already agrees,
        # choose the smallest X move, including zero when CoP is inside.
        if abs(desired[1]-feet[0,1])>1e-9:
            result['flags'].append('x-only-cannot-change-support-y')
            return result
        fraction=float(np.clip(unconstrained,0.,1.))
        translation=np.array([desired[0]-feet[0,0]-fraction*direction[0],0.])
    else:
        fraction=float((desired[1]-feet[0,1])/direction[1])
        translation=np.array([desired[0]-feet[0,0]-fraction*direction[0],0.])
    weights=np.array([1.-fraction,fraction])
    segment_valid=bool(np.all(weights>=-1e-10))
    if not segment_valid:result['flags'].append('cop-outside-translated-support-segment')
    # Rounding only at numerical zero, never clipping a negative reaction
    # belonging to an infeasible x-only support solution.
    weights[np.abs(weights)<1e-12]=0.
    targets=feet+translation
    total_force=mass*np.r_[acceleration,effective_gravity]
    contact_forces=weights[:,None]*total_force
    cop_reconstructed=weights@targets
    arms=np.column_stack((targets-center,np.full(2,-height)))
    initial_moment=np.cross(arms,contact_forces).sum(axis=0)
    # A common force direction with tau_xy != 0 generally creates unwanted
    # yaw. Opposite horizontal forces do not change net force or roll/pitch
    # moment because both feet lie on the same ground plane.
    # Preserve the established zero-tau allocation exactly.
    tangential_delta=np.zeros(2)
    if np.any(torque!=0.):
        tangential_delta=-initial_moment[2]*np.array([-direction[1],direction[0]])/length2
        contact_forces[0,:2]-=tangential_delta
        contact_forces[1,:2]+=tangential_delta
    moment=np.cross(arms,contact_forces).sum(axis=0)
    force_error=contact_forces.sum(axis=0)+[0.,0.,-mass*gravity]-mass*np.r_[acceleration,az]
    per_foot_friction=[]
    force_allocation_valid=segment_valid
    for force in contact_forces:
        tangent=float(np.linalg.norm(force[:2]))
        if force[2]>1e-10:
            per_foot_friction.append(tangent/float(force[2]))
        elif tangent<=1e-10 and force[2]>=-1e-10:
            per_foot_friction.append(0.)
        else:
            # None is JSON-safe and denotes no finite Coulomb coefficient.
            per_foot_friction.append(None)
            force_allocation_valid=False
    friction=None if not force_allocation_valid else float(max(per_foot_friction))
    friction_feasible=(False if not force_allocation_valid else
                       None if mu is None else bool(friction<=mu+1e-12))
    if segment_valid and not force_allocation_valid:
        result['flags'].append('unloaded-contact-needs-tangential-force')
    if friction_feasible is False:result['flags'].append('insufficient-friction')
    if segment_valid and min(weights)<=1e-10:result['flags'].append('one-contact-unloaded')
    result.update(feasible=bool(segment_valid and force_allocation_valid and friction_feasible is not False),
        geometry_feasible=segment_valid,friction_feasible=friction_feasible,
        force_allocation_feasible=force_allocation_valid,
        required_friction_coefficient=friction,per_foot_required_friction_coefficient=per_foot_friction,
        yaw_cancel_tangential_delta_n=tangential_delta.tolist(),
        proportional_allocation_moment_nm=initial_moment.tolist(),
        lambda_on_segment=fraction,unclipped_projection_fraction=unconstrained,
        translation_xy_m=translation.tolist(),translation_norm_m=float(np.linalg.norm(translation)),
        targets_xy_m=targets.tolist(),support_weights=weights.tolist(),
        normal_forces_n=contact_forces[:,2].tolist(),contact_forces_xyz_n=contact_forces.tolist(),
        cop_reconstruction_error_m=(cop_reconstructed-desired).tolist(),
        force_balance_error_n=force_error.tolist(),moment_about_com_nm=moment.tolist(),
        moment_balance_error_nm=(moment-target_moment).tolist())
    return result
