"""Persistent STS motion registers, separate from nominal motor capabilities.

STS position speed is encoder steps/s; acceleration units are 100 steps/s².
Source: Feetech STS control-table tutorial (2022-06-18), addresses 41 and 46.
This is a command-profile model, not an identified internal servo controller.
"""
import numpy as np

class ServoProfile:
    def __init__(self, speed=3400, acceleration=254, acceleration_cap=254):
        cap=np.broadcast_to(np.asarray(acceleration_cap,dtype=float),(12,))
        if not np.isfinite(cap).all() or np.any(cap!=np.floor(cap)) or np.any((cap<1)|(cap>254)):
            raise ValueError('Invalid installed acceleration cap')
        self.acceleration_cap=cap.astype(int).copy()
        self.set(speed,acceleration)

    def set(self,speed,acceleration):
        speed=np.broadcast_to(np.asarray(speed,dtype=float),(12,))
        acceleration=np.broadcast_to(np.asarray(acceleration,dtype=float),(12,))
        if (not np.isfinite(speed).all() or not np.isfinite(acceleration).all()
            or np.any(speed!=np.floor(speed)) or np.any(acceleration!=np.floor(acceleration))
            or np.any((speed<0)|(speed>3400)) or np.any((acceleration<0)|(acceleration>254))):
            raise ValueError('Invalid STS speed/acceleration register')
        self.speed=speed.astype(int).copy()
        self.acceleration=np.minimum(acceleration.astype(int),self.acceleration_cap)

    def velocity_limit(self,nominal):
        requested=np.where(self.speed==0,np.inf,self.speed*2*np.pi/4096)
        return np.minimum(nominal,requested)

    def acceleration_limit(self):
        return np.where(self.acceleration==0,np.inf,self.acceleration*100*2*np.pi/4096)

    def snapshot(self):
        return dict(goal_speed=self.speed.tolist(),acceleration=self.acceleration.tolist())
