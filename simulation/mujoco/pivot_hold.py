"""Experimental proprioceptive pivot hold: delayed encoders/IMU, scheduled contacts.
No simulator position, velocity, contact force, or world-pose inputs are accepted.
"""
import numpy as np
from support_shift import SupportShift
from footstep_tracker import rotation

class PivotHold:
    def __init__(self,model):
        self.kin=SupportShift(model);self.anchors=None;self.translation=np.zeros(3)
        self.previous=np.ones(4,dtype=bool);self.delta=np.zeros(2);self.diagnostic={}

    def apply(self,nominal,encoders,attitude,reading,phase,duty,config,enabled):
        if not enabled or encoders is None or reading is None or attitude.failures:
            self.anchors=None;self.delta[:]=0;return np.asarray(nominal)
        gain=float(config.get('gain',.5));limit=float(config.get('limit_m',.03))
        if not 0<=gain<=1 or not 0<limit<=.06:raise ValueError('Invalid pivot hold gains')
        k=self.kin;k.set_angles(encoders);feet=np.array([k.foot(i) for i in range(4)])
        center=k.data.xipos[k.model.body('cad_base').id].copy()
        R=rotation(*np.radians(np.asarray(attitude.filtered)/10),np.radians(reading.get('yaw_tenths',0)/10))
        relative=feet@R.T;chassis=R@center
        scheduled=np.array([(phase+o)%1<duty for o in (0,.5,.5,0)])
        if self.anchors is None:
            self.anchors=relative.copy();self.translation[:]=0;self.goal=chassis.copy();self.previous=scheduled.copy()
        support=np.flatnonzero(scheduled & self.previous)
        valid=len(support)>=2
        if valid:
            candidates=self.anchors[support]-relative[support]
            estimate=np.median(candidates,axis=0)
            valid=np.max(np.linalg.norm(candidates[:,:2]-estimate[:2],axis=1))<.02
            if valid:self.translation+=np.clip(estimate-self.translation,-.003,.003)
        for i in np.flatnonzero(scheduled & ~self.previous):self.anchors[i]=self.translation+relative[i]
        self.previous=scheduled
        error=self.translation+chassis-self.goal
        desired=(R.T@error)[:2]*gain;norm=np.linalg.norm(desired)
        if norm>limit:desired*=limit/norm
        self.delta+=np.clip(desired-self.delta,-.0005,.0005)
        k.set_angles(nominal);points=np.array([k.foot(i) for i in range(4)]);points[:,:2]+=self.delta
        q,residual=k.solve(points,nominal,iterations=8,stance=scheduled)
        self.diagnostic=dict(estimated_center_error_m=error.tolist(),correction_body_m=self.delta.tolist(),consistent_support=bool(valid),residual_m=residual,
            source='delayed IMU/encoders and scheduled-contact hypothesis; no contact sensor')
        if residual>.001:raise ValueError('Pivot hold target unreachable')
        return q
