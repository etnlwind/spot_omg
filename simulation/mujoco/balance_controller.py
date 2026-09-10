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
        self.profile="legacy";self.phase=0.;self.moving=False;self.linear=self.yaw=0.
        self.standing=False
        self.integral=np.zeros(2)
        self.correction=np.zeros(12)
        self.applied=False
        self.saturated=False

    def apply(self, nominal, attitude, available, permitted):
        import ctypes
        from drive_controller import NAMES
        fn=self.policy._library.spot_balance_control_policy
        fp=ctypes.POINTER(ctypes.c_float)
        fn.argtypes=(fp,fp,fp,ctypes.c_int,ctypes.c_int,ctypes.c_float,ctypes.c_float,ctypes.c_float,ctypes.c_int,ctypes.c_float,ctypes.c_int,ctypes.c_float,ctypes.c_float)
        fn.restype=ctypes.c_int
        state=(ctypes.c_float*16)(*self.integral,*self.correction,self.saturated,self.applied)
        values=(ctypes.c_float*12)(*nominal)
        factor=math.pi/1800
        imu=(ctypes.c_float*4)(*(v*factor for v in (*attitude.filtered,*attitude.rate)))
        if not fn(state,values,imu,self.enabled and available and permitted,self.standing,self.kp,self.kd,self.ki,
                  NAMES.index(self.profile),self.phase,self.moving,self.linear,self.yaw):
            raise ValueError('Shared balance target is unreachable')
        self.integral=np.array(state[:2]);self.correction=np.array(state[2:14])
        self.saturated=bool(state[14]);self.applied=bool(state[15])
        return np.array(values)

    def diagnostic(self):
        return dict(policy="level-roll" if self.profile in ("level","level15","joint","jointfast","jointsport") else "imu-phase-aware-v2" if self.profile=="imu" else "legacy-pi",expected_support_only=True,
                    enabled=self.enabled,applied=self.applied,kp=self.kp,kd=self.kd,ki=self.ki,
                    phase_aware_active=self.profile=="imu" and self.moving and self.applied,
                    max_j1_correction_deg=float(max(abs(self.correction[::3]))),
                    integral_rad=self.integral.tolist(),max_correction_deg=float(max(abs(self.correction))),saturated=self.saturated)
