# attitudepd diagonal synchronization audit

2026-09-14. Current local code, default 2.754kg model and D37.3 x 27mm rigid estimated cushions. Forward input 600/1000, yaw 0. Each trial 20s; start 2s, stop 17s, measurement window 5–17s, 50Hz. Compared S alignment/rebasing ON/OFF and attitude PD ON/OFF. Heading left at default ON. No physical hardware used or motion policy modified by this audit.

FAIL: physical diagonal swing synchronization in current S+PD case.

|Foot|Peak ground clearance mm|Median airborne interval ms (>1mm)|
|---|---:|---:|
|FL|11.93|460|
|FR|12.03|460|
|RL|1.42|40|
|RR|1.30|40|

FL/RR and FR/RL airborne state differs in 26.7% and 28.0% of sampled frames. The nominal paired height-difference peak-to-peak is only 0.152/0.161mm, rising to 0.714/0.710mm after PD. Thus the observed physical asymmetry is much larger than nominal paired target differences. PD OFF retains 26.8%/27.7% mismatch: PD alone is not the cause. Exact causal attribution among physical tracking/load/contact and S transformation needs further controlled tests.

S OFF tests also have asymmetry; S OFF + PD ON trips planner protection. Do not interpret these unequal valid motion windows as a clean performance ranking. Nearest-event matching in summary.json can reuse liftoffs and is unreliable when an entire swing is missed; airtime and height are the principal evidence. Current matched liftoffs are 20–40ms apart at 20ms sampling resolution, but the rear swing is almost absent, making phase-delay-only analysis misleading.

Current S+PD trial: safety ok, stop completed, maximum tilt 2.08deg. These checks alone do NOT establish gait quality. CSV files include nominal and corrected contact-point height, physical clearance and tilt. current-airtime.json contains the current case's airborne metrics. No corrective tuning applied.
