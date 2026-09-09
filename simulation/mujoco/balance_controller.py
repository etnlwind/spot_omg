"""Bounded feedback through the same C leg-correction policy as STM32.

Only delayed BNO055 observations enter this controller; no MuJoCo truth state.
Separate nominal targets prevent integration/accumulation of corrections.
"""
import math
import numpy as np
from servo.attitude import ImuSample
from cad_gait import KEYS


class BalanceController:
    def __init__(self, policy, kp=.25, kd=.015):
        self.policy=policy
        self.kp=kp;self.kd=kd;self.ki=.15
        self.enabled=True
        self.standing=False
        self.integral=np.zeros(2)
        self.correction=np.zeros(12)
        self.applied=False
        self.saturated=False

    def apply(self, nominal, attitude, available, permitted):
        wanted=np.zeros(12)
        length_saturated=False
        length_limit=.12 if self.standing else .06
        joint_limit=10 if self.standing else 6
        self.applied=bool(self.enabled and available and permitted)
        if self.applied:
            factor=math.pi/1800
            error=np.array(attitude.filtered)*factor
            # Slow bounded integral removes persistent slope/load error. Stop
            # winding into saturation, but allow the integral to unwind.
            for axis in range(2):
                if not self.saturated or error[axis]*self.integral[axis]<0:
                    self.integral[axis]=np.clip(self.integral[axis]+self.ki*error[axis]*.02,-length_limit,length_limit)
            controlled=error+self.integral/max(self.kp,.001)
            sample=ImuSample(*controlled,*(v*factor for v in attitude.rate))
            effort=self.kp*controlled+self.kd*np.array(attitude.rate)*factor
            length_saturated=abs(effort[0])+abs(effort[1])>=length_limit
            out=self.policy.balance_targets(dict(zip(KEYS,nominal)),sample=sample,
                support_legs=('FL','FR','RL','RR'),kp=self.kp,kd=self.kd,
                leg_length_limit=length_limit,mode='contact-aware',j1_gain=0,j1_limit=0,
                foot_placement_gain=0,foot_placement_limit=0)
            wanted=np.array([out[k] for k in KEYS])-nominal
        else:
            self.integral[:]=0
        self.saturated=bool(length_saturated or np.any(abs(wanted)>joint_limit))
        wanted=np.clip(wanted,-joint_limit,joint_limit)
        # 15 degrees/s avoids steps on enable, stale input and pose changes.
        self.correction+=np.clip(wanted-self.correction,-.3,.3)
        target=np.array(nominal)+self.correction
        return np.clip(target.reshape(4,3),[-30,-45,0],[30,100,150]).reshape(12)

    def diagnostic(self):
        return dict(enabled=self.enabled,applied=self.applied,kp=self.kp,kd=self.kd,ki=self.ki,
                    integral_rad=self.integral.tolist(),max_correction_deg=float(max(abs(self.correction))),saturated=self.saturated)
