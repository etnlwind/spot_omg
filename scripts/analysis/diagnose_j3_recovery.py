"""Read-only host-C swing diagnosis. No robot transport or firmware edits."""
import argparse
import csv
import ctypes
import json
from pathlib import Path
import tempfile

import numpy as np

import compare_v4_v5_cadence as shared

EXTRA_C = r'''
int without_lift(int profile,float phase,float linear,float feet[12]) {
    float p[7]; locomotion_params(profile,linear,p); p[3]=0;
    GaitPolicyLegTarget out[4];
    if(!center_pivot_targets_weighted(p,phase,1,linear,0,
            locomotion_forward_template_weight(profile,linear),out))return 0;
    for(int i=0;i<4;i++) {
        float q[3]={out[i].j1_deg,out[i].j2_deg,out[i].j3_deg};
        arc_foot(i,q,&feet[3*i],0);
    }
    return 1;
}
void foot_fk(int leg,float q[3],float out[3]) { arc_foot(leg,q,out,0); }
'''


def first_event(mask, fraction, duration):
    indices = np.flatnonzero(mask)
    if not len(indices):
        return None
    i = int(indices[0])
    return {"swing_fraction": float(fraction[i]), "time_ms": float(fraction[i]*duration*1000)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=shared.ROOT / "artifacts/attitudepd-v5/j3-recovery-diagnosis")
    args = parser.parse_args()
    original_hashes = shared.source_hashes()
    shared.C_SOURCE += EXTRA_C
    cases, rows = [], []
    u = np.linspace(0, 1, 1001)
    with tempfile.TemporaryDirectory() as temporary:
        kernel = shared.Kernel(Path(temporary))
        fp = ctypes.POINTER(ctypes.c_float)
        kernel.lib.without_lift.argtypes = (ctypes.c_int, ctypes.c_float, ctypes.c_float, fp)
        kernel.lib.without_lift.restype = ctypes.c_int
        kernel.lib.foot_fk.argtypes = (ctypes.c_int, fp, fp)
        for name in shared.PROFILES:
            profile = kernel.lib.profile_id(name.encode())
            for linear in (.588, 1.):
                p = (ctypes.c_float*7)()
                kernel.lib.parameters(profile, linear, p)
                period = float(kernel.lib.profile_period(profile, linear))
                duration = (1-p[1])*period
                for front in (0, 30):
                    case = {"profile": name, "linear": linear, "foot_lift_mm": [front,front,0,0],
                            "period_s": period, "swing_duration_s": duration, "legs": {}}
                    for leg, label in enumerate(shared.LEGS):
                        output, baseline = [], []
                        for fraction in u:
                            phase = float(p[1] + (1-p[1])*fraction - shared.OFFSETS[leg])
                            angles, feet, ticks = kernel.sample(profile, phase, linear, front)
                            zero = (ctypes.c_float*12)()
                            if not kernel.lib.without_lift(profile, phase, linear, zero):
                                raise ValueError("Invalid no-lift reference")
                            output.append((angles.reshape(4,3)[leg], feet[leg], ticks.reshape(4,3)[leg]))
                            baseline.append(np.array(zero).reshape(4,3)[leg])
                        angles = np.array([v[0] for v in output])
                        feet = np.array([v[1] for v in output])
                        baseline = np.array(baseline)
                        baseline_lift = (feet[:,2]-baseline[:,2])*1000
                        liftoff_z = (feet[:,2]-feet[0,2])*1000
                        dx = (feet[:,0]-feet[0,0])*1000
                        vx = np.gradient(feet[:,0], duration/1000)
                        # Recovery direction is the sign of the touchdown displacement.
                        sign = np.sign(dx[-1])
                        velocity = np.gradient(angles, duration/1000, axis=0)
                        recovery = first_event(sign*vx > 1.e-4, u, duration)
                        points = []
                        for index in (0, 50, 100, 150, 200, 250, 300, 400, 500, 700, 850, 900, 950, 1000):
                            points.append({"swing_fraction": float(u[index]),
                                "time_ms": float(u[index]*duration*1000),
                                "j2_deg": float(angles[index,1]), "j3_deg": float(angles[index,2]),
                                "x_from_liftoff_mm": float(dx[index]),
                                "z_from_liftoff_mm": float(liftoff_z[index]),
                                "z_lift_vs_same_phase_no_lift_mm": float(baseline_lift[index])})
                        mid = angles[500].copy()
                        mid[2] = angles[0,2]
                        held_foot = (ctypes.c_float*3)()
                        kernel.lib.foot_fk(leg, (ctypes.c_float*3)(*mid), held_foot)
                        case["legs"][label] = {
                            "j2_deg": {"start": float(angles[0,1]), "end": float(angles[-1,1]),
                                "min": float(angles[:,1].min()), "max": float(angles[:,1].max()),
                                "peak_swing_speed_deg_s": float(abs(velocity[2:-2,1]).max())},
                            "j3_deg": {"start": float(angles[0,2]), "end": float(angles[-1,2]),
                                "min": float(angles[:,2].min()), "max": float(angles[:,2].max()),
                                "peak_swing_speed_deg_s": float(abs(velocity[2:-2,2]).max())},
                            "peak_lift_vs_same_phase_no_lift_mm": float(baseline_lift.max()),
                            "minimum_z_from_liftoff_mm": float(liftoff_z.min()),
                            "maximum_z_from_liftoff_mm": float(liftoff_z.max()),
                            "first_recovery_direction_velocity": recovery,
                            "first_passing_liftoff_x_in_recovery_direction": first_event(sign*dx > .1, u, duration),
                            "lift_thresholds_vs_no_lift": {str(mm): first_event(baseline_lift >= mm, u, duration) for mm in (1,3,5,10)},
                            "mid_swing_z_loss_if_j3_held_at_liftoff_angle_mm": float((feet[500,2]-held_foot[2])*1000),
                            "selected_phases": points,
                        }
                        for i in range(len(u)):
                            rows.append({"profile": name,"linear":linear,"front_lift_mm":front,"leg":label,
                                "swing_fraction":float(u[i]),"time_ms":float(u[i]*duration*1000),
                                "j1_deg":float(angles[i,0]),"j2_deg":float(angles[i,1]),"j3_deg":float(angles[i,2]),
                                "x_from_liftoff_mm":float(dx[i]),"z_from_liftoff_mm":float(liftoff_z[i]),
                                "lift_vs_no_lift_mm":float(baseline_lift[i])})
                    cases.append(case)
    if shared.source_hashes() != original_hashes:
        raise RuntimeError("Source changed during sampling")
    args.output.mkdir(parents=True, exist_ok=True)
    report = {"scope":"Commanded canonical joint angles and CAD FK only; no actual joint readback or floor clearance measurement",
        "source_sha256": original_hashes,
        "limits":[
            "No IMU residual, servo acceleration/speed dynamics, compliance, load or ground contact.",
            "No-lift reference uses the same planar X trajectory with base swing height and extra front lift removed; CAD X may also differ slightly after IK.",
            "Canonical J3 angle is neither the link interior angle nor raw servo feedback.",
            "Fine-grid speed estimates may contain floating-point/IK noise; use the companion cadence report for rounded 50 Hz commands.",
            "Holding J3 at liftoff is a kinematic counterfactual at the mid-swing J1/J2 target, not an actual motor test.",
        ], "cases":cases}
    (args.output/"summary.json").write_text(json.dumps(report,indent=2)+"\n")
    with (args.output/"swing-curves.csv").open("w",newline="") as stream:
        writer=csv.DictWriter(stream,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
    for case in cases:
        for leg in ("FL","RL"):
            item=case["legs"][leg]
            print(json.dumps({"profile":case["profile"],"linear":case["linear"],"lift":case["foot_lift_mm"][0],"leg":leg,
                "swing_ms":case["swing_duration_s"]*1000,"j3":item["j3_deg"],
                "cad_lift_mm":item["peak_lift_vs_same_phase_no_lift_mm"],
                "recovery_onset":item["first_recovery_direction_velocity"],
                "lift_thresholds":item["lift_thresholds_vs_no_lift"]}))


if __name__ == "__main__":
    main()
