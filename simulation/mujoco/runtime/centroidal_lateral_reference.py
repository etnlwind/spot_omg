"""Periodic constant-height LIPM lateral reference, offline planned only.

For zero centroidal angular-momentum rate, CoP_y = c_y - h*c_y_ddot/g.
Solve the periodic equation with a Fourier filter for the planned diagonal
support-line intersections. This is not torque MPC or a contact measurement.
The finite pad support polygon, varying height and angular momentum are omitted.
"""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))
import numpy as np
from simulation.mujoco.runtime.gait_profiles import smooth


class CentroidalLateralReference:
    def __init__(self, goals, com, period_s, duty, stride_m, samples=1024):
        if not np.isfinite([duty,period_s,stride_m,samples]).all() or not .5 < duty < 1 or period_s <= 0:
            raise ValueError('Expected finite-overlap periodic trot')
        if not isinstance(samples,int) or not 32<=samples<=16384 or not 0<stride_m<=.2:
            raise ValueError('Invalid sample count or stride')
        goals=np.asarray(goals,dtype=float);com=np.asarray(com,dtype=float)
        if goals.shape!=(4,3) or com.shape!=(3,) or not np.isfinite(np.r_[goals.ravel(),com]).all():
            raise ValueError('Expected finite CAD foot goals and COM')
        if any(abs(goals[b,0]-goals[a,0])<.01 for a,b in ((0,3),(1,2))):
            raise ValueError('Diagonal support line has no fore-aft span')
        height=float(com[2]-goals[:,2].mean())
        if not .05 < height < .5:raise ValueError('Invalid COM height')
        self.phase=np.arange(samples)/samples
        cop=[]
        for phase in self.phase:
            points=goals.copy()
            for i,offset in enumerate((0.,.5,.5,0.)):
                q=(phase+offset)%1
                if q<duty:x=.5-q/duty
                else:
                    u=(q-duty)/(1-duty);k=(1-duty)/duty
                    x=-.5+(1+k)*smooth(u)-k*u
                points[i,0]+=stride_m*x
            lines=[]
            for pair in ((0,3),(1,2)):
                a,b=points[list(pair),:2]
                lines.append(a[1]+(com[0]-a[0])*(b[1]-a[1])/(b[0]-a[0])-com[1])
            window=duty-.5
            if phase<.5:
                blend=smooth(phase/window);y=(1-blend)*lines[1]+blend*lines[0]
            else:
                blend=smooth((phase-.5)/window);y=(1-blend)*lines[0]+blend*lines[1]
            cop.append(y)
        self.cop=np.asarray(cop)
        omega2=9.81/height
        frequency=2*np.pi*np.fft.fftfreq(samples,d=period_s/samples)
        spectrum=np.fft.fft(self.cop)*omega2/(omega2+frequency**2)
        self.position=np.fft.ifft(spectrum).real
        self.acceleration=np.fft.ifft(-frequency**2*spectrum).real
        residual=self.position-self.acceleration/omega2-self.cop
        self.diagnostic=dict(method='periodic-constant-height-LIPM-reference',height_m=height,
            planned_max_body_y_m=float(abs(self.position).max()),
            planned_within_10mm=bool(abs(self.position).max()<=.01),
            planned_max_acceleration_m_s2=float(abs(self.acceleration).max()),
            cop_equation_residual_m=float(abs(residual).max()))

    def at(self, phase):
        if not np.isfinite(phase):raise ValueError('Nonfinite phase')
        x=(float(phase)%1)*len(self.position)
        i=int(x);f=x-i
        return float((1-f)*self.position[i]+f*self.position[(i+1)%len(self.position)])
