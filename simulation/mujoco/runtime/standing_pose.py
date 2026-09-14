"""S: rigid cushion ground-contact point aligned with the CAD J2 axis in X."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))
import numpy as np
import mujoco
from simulation.mujoco.runtime.support_shift import SupportShift, LEGS

class ContactKinematics(SupportShift):
    def foot(self, i, jacobian=False):
        m,d=self.model,self.data;f=self.feet[i]
        world=self.vertices[i] @ d.geom_xmat[f].reshape(3,3).T+d.geom_xpos[f]
        # Rounded rigid cushion: use the lowest surface, not the geom centre.
        point=world[world[:,2] <= world[:,2].min()+1e-7].mean(axis=0)
        if not jacobian:return point
        jp=np.zeros((3,m.nv));jr=np.zeros_like(jp)
        mujoco.mj_jac(m,d,jp,jr,point,int(m.geom_bodyid[f]))
        return point,jp[:,self.v[i*3:i*3+3]]

def solve_standing_pose(model):
    kin=ContactKinematics(model)
    angles=np.tile([0.,45.,90.],4);kin.set_angles(angles)
    goals=np.array([kin.foot(i) for i in range(4)])
    goals[:,2]=goals[:,2].mean()
    for _ in range(8):
        kin.set_angles(angles)
        goals[:,0]=[kin.data.xanchor[model.joint(l+'_j2').id,0] for l in LEGS]
        angles,residual=kin.solve(goals,angles,iterations=100)
    kin.set_angles(angles)
    errors=np.array([kin.foot(i)[0]-kin.data.xanchor[model.joint(l+'_j2').id,0] for i,l in enumerate(LEGS)])
    if residual>.0002 or np.max(abs(errors))>.0002:
        raise ValueError(f'S contact alignment unreachable: residual={residual}, X={errors}')
    return angles, errors

class SoleKinematics(ContactKinematics):
    """Track a fixed sole material point in XY; use the surface minimum for Z.

    Choosing a new lowest mesh vertex for XY at every frame makes a tiny
    foot rotation jump between contact vertices. The material point is the
    real contact point at S and remains fixed on that foot throughout gait.
    """
    def __init__(self,model,standing):
        super().__init__(model)
        self.set_angles(standing)
        self.local_points=[]
        for i,f in enumerate(self.feet):
            point=ContactKinematics.foot(self,i)
            self.local_points.append(self.data.geom_xmat[f].reshape(3,3).T@(point-self.data.geom_xpos[f]))

    def contact(self,i):
        return ContactKinematics.foot(self,i)

    def solve_xz(self,targets,seed,locked_j1,iterations=60):
        """Fix mechanical J1, solve shared sole X/Z with J2/J3; Y is free."""
        locked_j1=np.asarray(locked_j1,dtype=float)
        if locked_j1.shape!=(4,) or not np.isfinite(locked_j1).all() or np.max(abs(locked_j1))>30:
            raise ValueError('Expected four finite J1 angles within joint limits')
        angles=np.asarray(seed).copy();angles[::3]=locked_j1
        for _ in range(iterations):
            self.set_angles(angles);increments=np.zeros(12);errors=[]
            for i in range(4):
                point,jac=self.foot(i,True)
                error=(targets[i]-point)[[0,2]];errors.append(np.linalg.norm(error))
                active=jac[[0,2],1:]
                dq=np.linalg.solve(active.T@active+np.eye(2)*1e-9,active.T@error)
                increments[i*3+1:i*3+3]=np.clip(np.degrees(dq),-4,4)
            if max(errors)<.00001:break
            angles+=increments
            angles=np.clip(angles,np.tile([-30,-45,0],4),np.tile([30,100,150],4))
        self.set_angles(angles)
        residual=max(np.linalg.norm((targets[i]-self.foot(i))[[0,2]]) for i in range(4))
        return angles,float(residual)

    def foot(self,i,jacobian=False):
        m,d=self.model,self.data;f=self.feet[i]
        material=d.geom_xpos[f]+d.geom_xmat[f].reshape(3,3)@self.local_points[i]
        contact=ContactKinematics.foot(self,i)
        point=material.copy();point[2]=contact[2]
        if not jacobian:return point
        jp=np.zeros((3,m.nv));jr=np.zeros_like(jp)
        mujoco.mj_jac(m,d,jp,jr,material,int(m.geom_bodyid[f]))
        cp=np.zeros_like(jp);cr=np.zeros_like(jp)
        mujoco.mj_jac(m,d,cp,cr,contact,int(m.geom_bodyid[f]));jp[2]=cp[2]
        return point,jp[:,self.v[i*3:i*3+3]]

class StandingGaitFrame:
    """Translate nominal foot displacements onto S, then solve CAD contact IK."""
    def __init__(self, model, standing):
        self.kin=SoleKinematics(model,standing)
        self.standing=np.asarray(standing).copy()
        self.kin.set_angles(standing)
        self.feet=np.array([self.kin.foot(i) for i in range(4)])
        self.previous=self.standing.copy()

    def targets(self, angles, neutral):
        k=self.kin;k.set_angles(neutral)
        reference=np.array([k.foot(i) for i in range(4)])
        k.set_angles(angles)
        delta=np.array([k.foot(i) for i in range(4)])-reference
        result,error=k.solve(self.feet+delta,self.previous,iterations=35)
        if error>.001 or not np.isfinite(result).all():
            raise ValueError(f'S gait contact target unreachable: {error:.6f} m')
        self.previous=result.copy()
        return result
