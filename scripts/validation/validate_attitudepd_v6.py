"""Compare production C gait targets through the existing estimated STS profile.

This is an offline regression screen, not a hardware test or ground-contact
simulation. Observed J3 errors are angular sensitivity cases, never time delays.
"""
import argparse
import csv
import ctypes
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools/servo_tool"))
from servo.host_build import build_shared, library_suffix
from simulation.mujoco.runtime.servo_profile import ServoProfile

LEGS = ("FL", "FR", "RL", "RR")
OFFSETS = np.array((0., .5, .5, 0.))
JOINTS = tuple(f"{leg}_J{j}" for leg in LEGS for j in (1, 2, 3))
FRAME_DT = .02
SUBSTEP_DT = .0005
C_SOURCE = r'''
#include "foot_lift.h"
#include "locomotion_servo.h"
#include "motor_capability.h"
int profile_id(const char *name) { return locomotion_profile_id(name); }
float period(int profile,float linear,float yaw) {
    return locomotion_period(profile,linear,yaw);
}
float duty(int profile,float linear) {
    float p[7]; locomotion_params(profile,linear,p);return p[1];
}
float next_phase(float phase,float period) {
    return fmodf(phase+.02f/period,1.f);
}
void foot_points(const float angles[12],float feet[12]) {
    for(int leg=0;leg<4;leg++)arc_foot(leg,&angles[3*leg],&feet[3*leg],0);
}
int sample(int profile,float phase,float linear,float yaw,unsigned front_mm,
           float raw[12],float rounded[12],float feet[12]) {
    GaitPolicyLegTarget out[4];uint16_t ticks[12];
    uint32_t lift[4]={front_mm,front_mm,0,0};
    if(!locomotion_targets(profile,phase,1,linear,yaw,out) ||
       !foot_lift_profile(lift,profile,phase,fminf(1,(fabsf(linear)+fabsf(yaw))/.15f),linear,yaw,out) ||
       !locomotion_servo_targets(out,ticks))return 0;
    for(int i=0;i<4;i++) {
        raw[3*i]=out[i].j1_deg;raw[3*i+1]=out[i].j2_deg;raw[3*i+2]=out[i].j3_deg;
    }
    for(int i=0;i<12;i++) {
        const RobotJointConfig *j=&g_robot_joints[i];
        rounded[j->leg_index*3+j->joint_index-1]=((int)ticks[i]-j->center)*j->direction*360.f/4096.f;
    }
    foot_points(raw,feet);return 1;
}
void actuator_parameters(float speeds[12],float accelerations[12]) {
    for(int i=0;i<12;i++) {
        speeds[i]=g_robot_joints[i].joint_index==2 ?
            MOTOR_STS3250_NOMINAL_MAX_VELOCITY_DEG_S : MOTOR_STS3215_NOMINAL_MAX_VELOCITY_DEG_S;
        accelerations[i]=robot_servo_profile_acceleration(g_robot_joints[i].servo_id,254);
    }
}
'''


def source_hashes():
    sources = list((ROOT / "firmware/stm32-learning/Inc").glob("*.h"))
    sources += [ROOT / "firmware/stm32-learning/Src/robot_config.c",
                ROOT / "config/locomotion_profiles.json",
                ROOT / "simulation/mujoco/runtime/servo_profile.py", Path(__file__).resolve()]
    return {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sources}


class Kernel:
    def __init__(self, directory):
        unit = directory / "validation.c"
        unit.write_text(C_SOURCE)
        library = build_shared([unit, ROOT / "firmware/stm32-learning/Src/robot_config.c"],
            ROOT / "firmware/stm32-learning/Inc", directory / ("validation" + library_suffix()),
            extra=("-ffp-contract=off",))
        self.lib = ctypes.CDLL(str(library))
        fp = ctypes.POINTER(ctypes.c_float)
        self.lib.profile_id.argtypes = (ctypes.c_char_p,)
        self.lib.profile_id.restype = ctypes.c_int
        self.lib.period.argtypes = (ctypes.c_int, ctypes.c_float, ctypes.c_float)
        self.lib.period.restype = ctypes.c_float
        self.lib.duty.argtypes = (ctypes.c_int, ctypes.c_float)
        self.lib.duty.restype = ctypes.c_float
        self.lib.next_phase.argtypes = (ctypes.c_float, ctypes.c_float)
        self.lib.next_phase.restype = ctypes.c_float
        self.lib.sample.argtypes = (ctypes.c_int, ctypes.c_float, ctypes.c_float,
                                    ctypes.c_float, ctypes.c_uint, fp, fp, fp)
        self.lib.sample.restype = ctypes.c_int
        self.lib.foot_points.argtypes = (fp, fp)
        self.lib.actuator_parameters.argtypes = (fp, fp)
        speed, acceleration = (ctypes.c_float*12)(), (ctypes.c_float*12)()
        self.lib.actuator_parameters(speed, acceleration)
        self.nominal_speed = np.radians(np.array(speed, dtype=float))
        self.acceleration_caps = np.array(acceleration, dtype=int)

    def sample(self, profile, phase, linear, yaw, lift):
        raw, rounded, feet = ((ctypes.c_float*12)() for _ in range(3))
        if not self.lib.sample(profile, phase, linear, yaw, lift, raw, rounded, feet):
            raise ValueError(f"Invalid shared target: profile={profile}, phase={phase}, "
                             f"linear={linear}, yaw={yaw}, front_lift={lift}")
        return np.array(raw, dtype=float), np.array(rounded, dtype=float), np.array(feet, dtype=float).reshape(4,3)

    def feet(self, angles):
        result = (ctypes.c_float*12)()
        self.lib.foot_points((ctypes.c_float*12)(*np.asarray(angles).reshape(12)), result)
        return np.array(result, dtype=float).reshape(4,3)


def first(mask, phases):
    indices = np.flatnonzero(mask)
    return float(phases[indices[0]]) if len(indices) else None


def measured_error_reference():
    path = ROOT / "artifacts/attitudepd-v5/j3-recovery-diagnosis/actual-gait-summary.json"
    if not path.exists():
        return {"source": "User-operated V91 diagnostic summary unavailable", "j3_error_bounds_deg": [14.5,17.1]}
    report = json.loads(path.read_text())
    values = [j["peak_error_deg"] for j in report["joints"] if j["joint"] == "J3"]
    return {"source": str(path.relative_to(ROOT)), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "firmware": report.get("firmware"), "j3_peak_errors_deg": values,
            "j3_error_bounds_deg": [min(values), max(values)],
            "use": "Angular FK sensitivity only. No match_age or past-goal-match field is used as latency."}


def run_case(kernel, name, linear, lift, seconds, observed_bounds, phase_samples=1000):
    profile = kernel.lib.profile_id(name.encode())
    if profile < 0:
        raise ValueError(f"Profile not present yet: {name}")
    period = float(kernel.lib.period(profile, linear, 0))
    duty = float(kernel.lib.duty(profile, linear))
    phases = np.arange(phase_samples)/phase_samples
    fine = [kernel.sample(profile, float(p), linear, 0, lift) for p in phases]
    raw = np.array([s[0] for s in fine])
    feet = np.array([s[2] for s in fine])
    foot_velocity = (np.roll(feet,-1,axis=0)-np.roll(feet,1,axis=0))/(2*period/phase_samples)
    low, high = np.tile((-30.,-45.,0.),4), np.tile((30.,100.,150.),4)
    nominal_limits = bool(np.all(raw >= low-.0001) and np.all(raw <= high+.0001))
    leg_metrics, baseline = {}, np.empty((4,2))
    for leg, label in enumerate(LEGS):
        start = kernel.sample(profile, float(duty-OFFSETS[leg]), linear, 0, lift)[2][leg]
        end = kernel.sample(profile, float(1-OFFSETS[leg]), linear, 0, lift)[2][leg]
        stance_start = kernel.sample(profile, float(-OFFSETS[leg]), linear, 0, lift)[2][leg]
        baseline[leg] = (start[2],end[2])
        q = (phases+OFFSETS[leg]) % 1
        stance = (q > 2/phase_samples) & (q < duty-2/phase_samples)
        indices = np.flatnonzero(q >= duty)
        indices = indices[np.argsort(q[indices])]
        u = (q[indices]-duty)/(1-duty)
        swing_feet = feet[indices,leg]
        reference_z = start[2]+u*(end[2]-start[2])
        clearance = (swing_feet[:,2]-reference_z)*1000
        forward_sign = np.sign(end[0]-start[0])
        dx = forward_sign*(swing_feet[:,0]-start[0])*1000
        vx = np.gradient(swing_feet[:,0], period/phase_samples)*forward_sign
        onset_index = np.flatnonzero(vx > .002)
        onset = int(onset_index[0]) if len(onset_index) else None
        early = (u >= .1) & (u <= .2)
        middle = (u >= .2) & (u <= .8)
        held = u <= .18
        sensitivity = {}
        for error in observed_bounds:
            worst = []
            for index, z_ref in zip(indices,reference_z):
                variants = []
                for sign in (-1,1):
                    altered = raw[index].copy(); altered[3*leg+2] += sign*error
                    variants.append((kernel.feet(altered)[leg,2]-z_ref)*1000)
                worst.append(min(variants))
            worst = np.array(worst)
            sensitivity[f"j3_plus_or_minus_{error:g}_deg"] = {
                "mid_swing_min_proxy_mm": float(worst[middle].min()),
                "mid_swing_mean_proxy_mm": float(worst[middle].mean()),
                "mid_swing_fraction_below_reference": float(np.mean(worst[middle] < 0)),
            }
        stance_dx = (start[0]-stance_start[0])*1000
        leg_metrics[label] = {
            "stance_signed_x_travel_mm": float(stance_dx),
            "stance_mean_x_speed_mm_s": float(stance_dx/(duty*period)),
            "stance_peak_abs_x_speed_mm_s": float(abs(foot_velocity[stance,leg,0]).max()*1000),
            "stance_direction_opposes_command": bool(stance_dx*linear < 0),
            "early_10_to_20pct_mean_command_proxy_mm": float(clearance[early].mean()),
            "early_10_to_20pct_min_command_proxy_mm": float(clearance[early].min()),
            "mid_swing_min_command_proxy_mm": float(clearance[middle].min()),
            "mid_swing_mean_command_proxy_mm": float(clearance[middle].mean()),
            "first_5mm_command_proxy_at_swing_fraction": first(clearance >= 5,u),
            "first_forward_recovery_at_swing_fraction": None if onset is None else float(u[onset]),
            "first_forward_displacement_over_1mm_at_swing_fraction": first(dx >= 1,u),
            "first_18pct_x_excursion_mm": float(np.ptp(swing_feet[held,0])*1000),
            "command_proxy_at_18pct_mm": float(np.interp(.18,u,clearance)),
            "command_proxy_at_20pct_mm": float(np.interp(.2,u,clearance)),
            "command_proxy_at_forward_recovery_mm": None if onset is None else float(clearance[onset]),
            "backward_excursion_after_liftoff_mm": float(max(0,-dx.min())),
            "observed_error_sensitivity": sensitivity,
        }

    registers = ServoProfile(3400,254,kernel.acceleration_caps)
    # Four full cycles warm up this reference model, independently of startup.
    warmup = 4*period
    duration = max(seconds,warmup+4*period)
    frames = round(duration/FRAME_DT)
    phase = 0.
    initial = kernel.sample(profile,phase,linear,0,lift)[1]
    position, velocity = np.radians(initial), np.zeros(12)
    commands, unrounded, references, phase_history, times = [], [], [], [], []
    model_feet, model_before = [], []
    for frame in range(frames):
        target, rounded, _ = kernel.sample(profile,phase,linear,0,lift)
        before = np.degrees(position.copy())
        for _ in range(round(FRAME_DT/SUBSTEP_DT)):
            position,velocity=registers.advance_reference(position,velocity,np.radians(rounded),SUBSTEP_DT,kernel.nominal_speed)
        if frame*FRAME_DT >= warmup:
            commands.append(rounded);unrounded.append(target);references.append(np.degrees(position.copy()))
            model_before.append(before);phase_history.append(phase);times.append(frame*FRAME_DT)
            model_feet.append(kernel.feet(np.degrees(position)))
        phase = kernel.lib.next_phase(phase,period)
    commands, unrounded, references = map(np.asarray,(commands,unrounded,references))
    model_feet, phase_history = np.asarray(model_feet),np.asarray(phase_history)
    velocity_commands=np.diff(commands,axis=0)/FRAME_DT
    acceleration_commands=np.diff(velocity_commands,axis=0)/FRAME_DT
    jerk_commands=np.diff(acceleration_commands,axis=0)/FRAME_DT
    raw_velocity=np.diff(unrounded,axis=0)/FRAME_DT
    raw_acceleration=np.diff(raw_velocity,axis=0)/FRAME_DT
    raw_jerk=np.diff(raw_acceleration,axis=0)/FRAME_DT
    error=commands-references
    joint_metrics={label:{
        "max_command_velocity_deg_s":float(abs(velocity_commands[:,j]).max()),
        "max_command_acceleration_deg_s2":float(abs(acceleration_commands[:,j]).max()),
        "max_command_jerk_deg_s3":float(abs(jerk_commands[:,j]).max()),
        "max_unrounded_velocity_deg_s":float(abs(raw_velocity[:,j]).max()),
        "max_unrounded_acceleration_deg_s2":float(abs(raw_acceleration[:,j]).max()),
        "max_unrounded_jerk_deg_s3":float(abs(raw_jerk[:,j]).max()),
        "max_estimated_reference_error_deg":float(abs(error[:,j]).max()),
        "rms_estimated_reference_error_deg":float(np.sqrt(np.mean(error[:,j]**2))),
    } for j,label in enumerate(JOINTS)}
    for leg,label in enumerate(LEGS):
        local=(phase_history+OFFSETS[leg])%1
        u=(local-duty)/(1-duty)
        middle=(u>=.2)&(u<=.8)
        early=(u>=.1)&(u<=.2)
        swing=(u>=0)&(u<1)
        stance=u<0
        reference_z=baseline[leg,0]+u*(baseline[leg,1]-baseline[leg,0])
        proxy=(model_feet[:,leg,2]-reference_z)*1000
        leg_metrics[label].update({
            "early_10_to_20pct_mean_estimated_proxy_mm":float(proxy[early].mean()),
            "mid_swing_min_estimated_proxy_mm":float(proxy[middle].min()),
            "mid_swing_mean_estimated_proxy_mm":float(proxy[middle].mean()),
            "mid_swing_fraction_estimated_below_reference":float(np.mean(proxy[middle]<0)),
            "mid_swing_max_j3_reference_error_deg":float(abs(error[middle,3*leg+2]).max()),
            "swing_max_j3_reference_error_deg":float(abs(error[swing,3*leg+2]).max()),
            "stance_max_j3_reference_error_deg":float(abs(error[stance,3*leg+2]).max()),
            "swing_rms_j3_reference_error_deg":float(np.sqrt(np.mean(error[swing,3*leg+2]**2))),
        })
    rows=[]
    for i,t in enumerate(times):
        row={"time_s":t,"phase":float(phase_history[i])}
        for j,label in enumerate(JOINTS):
            row[label+"_command_deg"]=float(commands[i,j]);row[label+"_estimated_deg"]=float(references[i,j])
        for leg,label in enumerate(LEGS):row[label+"_estimated_body_z_m"]=float(model_feet[i,leg,2])
        rows.append(row)
    return {"profile":name,"linear":linear,"foot_lift_mm":[lift,lift,0,0],
            "period_s":period,"duty":duty,"stance_duration_s":period*duty,
            "swing_duration_s":period*(1-duty),"sampled_nominal_limits_valid":nominal_limits,
            "all_shared_targets_and_servo_encodings_valid":True,"warmup_s":warmup,
            "model_duration_s":duration,"legs":leg_metrics,"joints":joint_metrics},rows


def compare(cases):
    comparisons=[]
    for baseline in cases:
        if baseline["profile"]!="attitudepd_v5":continue
        matches=[c for c in cases if c["profile"]=="attitudepd_v6" and
                 c["linear"]==baseline["linear"] and c["foot_lift_mm"]==baseline["foot_lift_mm"]]
        if not matches:continue
        after=matches[0]; legs={}
        for leg in LEGS:
            a,b=baseline["legs"][leg],after["legs"][leg]
            ratio=b["stance_mean_x_speed_mm_s"]/a["stance_mean_x_speed_mm_s"]
            peak_ratio=b["stance_peak_abs_x_speed_mm_s"]/a["stance_peak_abs_x_speed_mm_s"]
            legs[leg]={
                "stance_mean_speed_ratio":ratio,"stance_mean_speed_preserved_within_1pct":abs(ratio-1)<=.01,
                "stance_peak_speed_ratio":peak_ratio,"stance_peak_speed_preserved_within_1pct":abs(peak_ratio-1)<=.01,
                "early_command_proxy_gain_mm":b["early_10_to_20pct_mean_command_proxy_mm"]-a["early_10_to_20pct_mean_command_proxy_mm"],
                "early_estimated_proxy_gain_mm":b["early_10_to_20pct_mean_estimated_proxy_mm"]-a["early_10_to_20pct_mean_estimated_proxy_mm"],
                "mid_estimated_proxy_gain_mm":b["mid_swing_mean_estimated_proxy_mm"]-a["mid_swing_mean_estimated_proxy_mm"],
                "mid_estimated_j3_error_change_deg":b["mid_swing_max_j3_reference_error_deg"]-a["mid_swing_max_j3_reference_error_deg"],
            }
        comparisons.append({"linear":baseline["linear"],"foot_lift_mm":baseline["foot_lift_mm"],"legs":legs,
            "initial_same_mean_speed_screen_pass":all(v["stance_mean_speed_preserved_within_1pct"] and
                v["early_command_proxy_gain_mm"]>=2 and v["mid_estimated_proxy_gain_mm"]>=-.5
                for v in legs.values()) if baseline["linear"]>0 else None})
    return comparisons


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,default=ROOT/"artifacts/attitudepd-v6/profile-validation")
    parser.add_argument("--profiles",nargs="+",default=["attitudepd_v5","attitudepd_v6"])
    parser.add_argument("--linears",nargs="+",type=float,default=[1.,.588,-1.,-.588])
    parser.add_argument("--front-lifts",nargs="+",type=int,default=[0,30])
    parser.add_argument("--seconds",type=float,default=12.)
    args=parser.parse_args()
    if args.seconds<1 or not np.isfinite(args.seconds) or any(not 0<abs(v)<=1 for v in args.linears):
        parser.error("Use finite nonzero input magnitudes <=1 and seconds>=1")
    if any(v<0 or v>2147483647 for v in args.front_lifts):parser.error("Invalid lift value")
    args.output.mkdir(parents=True,exist_ok=True)
    hashes=source_hashes(); observed=measured_error_reference(); cases=[]
    with tempfile.TemporaryDirectory() as temporary:
        kernel=Kernel(Path(temporary))
        for name in args.profiles:
            for linear in args.linears:
                for lift in args.front_lifts:
                    result,rows=run_case(kernel,name,linear,lift,args.seconds,observed["j3_error_bounds_deg"])
                    cases.append(result)
                    filename=f"{name}_{linear:+.3f}_front{lift}.csv"
                    with (args.output/filename).open("w",newline="") as stream:
                        writer=csv.DictWriter(stream,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
                    print(json.dumps({"profile":name,"linear":linear,"lift":lift,"legs":result["legs"]}),flush=True)
        model={"speed_register":3400,"acceleration_request":254,
            "observed_acceleration_caps":kernel.acceleration_caps.tolist(),
            "nominal_speed_deg_s":np.degrees(kernel.nominal_speed).tolist(),
            "profile_acceleration_deg_s2":np.degrees(ServoProfile(3400,254,kernel.acceleration_caps).acceleration_limit()).tolist(),
            "command_interval_s":FRAME_DT,"model_substep_s":SUBSTEP_DT,"added_delay_s":0}
    if hashes!=source_hashes():raise RuntimeError("Source changed during validation; rerun against frozen sources")
    summary={"source_commit":subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT,text=True).strip(),
        "source_sha256":hashes,"observed_error_reference":observed,"actuator_reference_model":model,
        "scope":"Shared C targets -> existing estimated STS command profile -> CAD FK; no hardware or loaded physics",
        "proxy_definition":"Body-frame foot Z above the straight interpolation between commanded liftoff/touchdown Z; not measured ground clearance",
        "initial_same_mean_speed_screen":"Every leg keeps signed mean stance speed within1%, gains >=2mm mean commanded clearance proxy over swing10-20%, and loses no more than0.5mm mean estimated mid-swing clearance proxy. Prototype3 intentionally changes mean speed; compare peak and mean separately rather than treating a preserved peak as preserved mean. This is not physical safety or gait acceptance.",
        "limitations":["Only command-reference acceleration/braking is modeled; no torque/load/gravity/contact/battery model.",
            "Four cycles warm up the reference model. Startup and stop are tested separately by the owner.",
            "Modeled reference is sampled at the end of each20ms held-command interval; it is not a measured encoder value.",
            "The observed14.5-17.1degree J3 errors provide angular FK sensitivity only; no measured or assumed communication latency is inferred.",
            "Fixed body-frame clearance proxy cannot prove floor contact or foot lift on the physical robot.",
            "Both floating target and quantized50Hz derivative peaks are reported; a single tick changes a20ms velocity by4.3945deg/s."],
        "cases":cases,"comparisons":compare(cases)}
    (args.output/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print(json.dumps({"summary":str(args.output/"summary.json"),"comparisons":summary["comparisons"]},indent=2))


if __name__=="__main__":main()
