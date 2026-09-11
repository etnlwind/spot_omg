"""Opt-in J1 support feedback experiment using delayed IMU only."""
import math
import numpy as np


def smooth(x):
    x=float(np.clip(x,0,1));return x*x*x*(10+x*(-15+6*x))


class SupportJ1:
    def __init__(self):
        self.delta=np.zeros(4)
        self.scale=1.
        self.saturated=False

    def reset(self):
        self.delta[:]=0;self.scale=1.;self.saturated=False

    def apply(self, nominal, balanced, attitude, phase, duty, config, enabled):
        if not enabled:
            self.reset();return balanced
        roll,pitch=np.asarray(attitude.filtered)/10.
        rate=float(attitude.rate[0])/10.
        gain=float(config.get('gain',1.5));damping=float(config.get('damping',.015))
        limit=float(config.get('limit_deg',5.))
        correction=gain*roll+damping*float(np.clip(rate,-30,30))
        self.saturated=abs(correction)>limit
        desired_scale=float(np.clip(1-(max(abs(roll),abs(pitch))-5)/8,.55,1))
        if self.saturated:desired_scale=min(desired_scale,.8)
        self.scale+=float(np.clip(desired_scale-self.scale,-.015,.003))
        result=np.array(balanced).reshape(4,3).copy();nominal=np.array(nominal).reshape(4,3)
        for leg,offset in enumerate((0,.5,.5,0)):
            q=(phase+offset)%1
            support=smooth(q/.08)*(1-smooth((q-duty+.08)/.08)) if q<duty else 0.
            swing_weight=float(config.get('swing_weight',.85))
            weight=swing_weight+(1-swing_weight)*support
            target=float(np.clip(correction,-limit,limit))*(1 if leg%2==0 else -1)*weight
            self.delta[leg]+=float(np.clip(target-self.delta[leg],-.2,.2))
            before=math.radians(result[leg,0])
            after=float(np.clip(nominal[leg,0]+self.delta[leg],-25,25))
            # Preserve forward reach and vertical projection while J1 rolls the leg.
            upper,knee=np.radians(result[leg,1:]);l1,l2=.141,.150
            x=l1*math.sin(upper)+l2*math.sin(upper-knee)
            z=(l1*math.cos(upper)+l2*math.cos(upper-knee))*math.cos(before)/math.cos(math.radians(after))
            cosine=(x*x+z*z-l1*l1-l2*l2)/(2*l1*l2)
            if not -1<=cosine<=1:
                self.saturated=True;continue
            k=math.acos(cosine);u=math.atan2(x,z)+math.atan2(l2*math.sin(k),l1+l2*math.cos(k))
            angles=np.degrees([u,k])
            if not (-45<=angles[0]<=100 and 0<=angles[1]<=150):
                self.saturated=True;continue
            result[leg]=[after,*angles]
        return result.ravel()
