"""Offline J2-near-horizontal recovery experiment; no registered model changes."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4]
sys.path[:0]=[str(ROOT),str(ROOT/'tools/servo_tool')]
import argparse
import numpy as np
import imageio_ffmpeg
from simulation.mujoco.scripts.analysis.capture_s_native_balance import capture

def smooth(u):
    u=np.clip(u,0,1)
    return u**3*(10+u*(-15+6*u))

def high_j2(robot,gait,phase,nominal):
    if gait.stop_progress is not None:return nominal
    q=nominal.copy()
    swing=((phase+np.array([.5,0,0,.5]))%1-.5)*2
    rise=smooth(swing/.25)
    fall=1-smooth((swing-.25)/.65)
    weight=np.where((swing>0)&(swing<1),rise*fall,0)
    # Entry starts with FR/RL swinging from S without a preceding push.
    # FL/RR finish their first support half-cycle at entry_phase=.5;
    # FR/RL finish their first post-landing support at entry_phase=1.
    # This is commanded support history, not a claim of measured contact.
    weight*=gait.entry_phase>=np.array([.5,1.,1.,.5])
    q[1::3]+=weight*np.maximum(0,85-q[1::3])
    # At J2=85 the old 16mm sole lift is geometrically unreachable: even
    # the lowest RL position at J1=-9 is about38mm above S. Fold toward104
    # rather than claiming to retain that incompatible Cartesian target.
    q[2::3]+=weight*np.maximum(0,104-q[2::3])
    return q


def high_j2_after_entry(robot,gait,phase,nominal):
    # Preserve both entry half-cycles. At entry_phase=1 the commanded J1
    # adduction is symmetric and FR/RL begin the next recovery from zero
    # bump weight; FL/RR follow half a cycle later. Actual alignment is
    # checked in the recorded feedback, not assumed from this phase gate.
    if gait.entry_phase < 1.:
        return nominal
    return high_j2(robot,gait,phase,nominal)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline',action='store_true')
    parser.add_argument('--after-push',action='store_true',help='Reproduce the previous one-entry-step experiment')
    cli=parser.parse_args()
    name='baseline' if cli.baseline else 'j2-85-after-push' if cli.after_push else 'j2-85-after-symmetric-entry'
    args=argparse.Namespace(command=344,walk_seconds=8,settle_seconds=4,
        profile='s_native_v6_2_7',allow_fall=False,fourth_view='side',
        ffmpeg=imageio_ffmpeg.get_ffmpeg_exe(),output=ROOT/'artifacts/imu-trace-v77/j2-horizontal'/name)
    capture(args,parser,experiment={'name':name,'label':name+' candidate | J2 85 / J3 104 | input 344','pack_open_circuit_voltage':11.1},
            target_adjustment=None if cli.baseline else high_j2 if cli.after_push else high_j2_after_entry)

if __name__=='__main__':main()
