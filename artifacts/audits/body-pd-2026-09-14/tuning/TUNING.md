# Attitude-PD bounded gain comparison

This experiment did not change production settings or automatically promote a candidate. Its frozen base-config.json was kp=0.2, kd=0.03s on both axes; later parent integration choices are separate from this experiment. All runs retain the 5mm Cartesian limit, original gait, physics, safety and motor model.

Each 30s run stands for 10s, then requests full forward for 20s. The tables use exactly t∈[10,30), 1,000 samples. Faulted runs are evaluated over this same window (including their post-stop state) and marked failed; the raw validator summary instead stops at the fault, so its shorter-window values must not be substituted.

## Original six candidates: legacy IK

| Condition | Roll P-P ° | Pitch P-P ° | Roll RMS ° | Pitch RMS ° | Gyro X RMS °/s | Gyro Y RMS °/s | Tracking max ° | Safety |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| off | 8.294 | 5.872 | 3.280 | 1.525 | 26.902 | 13.393 | 8.784 | complete |
| on | 11.152 | 5.947 | 2.189 | 1.000 | 20.945 | 10.077 | 10.640 | complete |
| kp0.1_kd0.01_attempt1 | 10.166 | 5.353 | 2.117 | 0.961 | 20.481 | 9.846 | 8.226 | complete |
| kp0.1_kd0.05_attempt1 | 25.920 | 13.266 | 3.691 | 1.722 | 26.692 | 16.038 | 31.006 | FAILED tilt at 24.06s |
| kp0.3_kd0.01_attempt1 | 12.371 | 5.684 | 2.521 | 1.228 | 22.764 | 11.761 | 10.064 | complete |
| kp0.3_kd0.05_attempt1 | 22.135 | 11.101 | 3.143 | 1.465 | 26.033 | 14.660 | 20.080 | FAILED tilt at 27.02s |
| kp0.5_kd0.015_attempt1 | 11.513 | 5.173 | 2.167 | 1.022 | 20.873 | 10.607 | 9.685 | complete |
| kp0.5_kd0.05_attempt1 | 26.483 | 11.497 | 3.542 | 1.560 | 27.068 | 15.964 | 31.852 | FAILED tilt at 27.44s |

All six candidate runs had identical before/after source SHA snapshots. The three kd=0.05s runs tripped tilt safety at 24.06s, 27.02s and 27.44s; none completed the required forward duration. The legacy low-gain candidate improved RMS, gyro and tracking, but roll peak-to-peak increased versus OFF.

## Precise IK follow-up

The parent tightened the final Cartesian IK to 1µm and added a 5µm numerical margin inside the same 5mm cap. Old solver trials are preserved separately. These three runs also had unchanged before/after source SHA. PD OFF bypasses the adapter and preserves nominal targets.

| Condition | Roll P-P ° | Pitch P-P ° | Roll RMS ° | Pitch RMS ° | Gyro X RMS °/s | Gyro Y RMS °/s | Tracking max ° | Safety |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| off | 8.294 | 5.872 | 3.280 | 1.525 | 26.902 | 13.393 | 8.784 | complete |
| precise_kp0.1_kd0.01_30s_attempt1 | 10.190 | 5.291 | 1.974 | 0.923 | 19.933 | 10.126 | 8.162 | complete |
| precise_kp0.2_kd0.03_30s_attempt1 | 19.155 | 7.747 | 2.749 | 1.249 | 23.516 | 11.823 | 17.128 | complete |

60s forward follow-up (10–70s, 3,000 samples; not an equal-duration OFF comparison):

| Condition | Roll P-P ° | Pitch P-P ° | Roll RMS ° | Pitch RMS ° | Gyro X RMS °/s | Gyro Y RMS °/s | Tracking max ° | Safety |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| precise_kp0.1_kd0.01_70s_attempt1 | 10.711 | 5.291 | 1.909 | 0.900 | 19.979 | 10.206 | 8.162 | complete |

| Condition | Max roll / pitch ° | Forward m | Applied correction peak mm |
|---|---:|---:|---:|
| off | 6.328 / 3.172 | 2.149 | 0.000 |
| precise_kp0.1_kd0.01_30s_attempt1 | 5.698 / 2.746 | 2.302 | 0.737 |
| precise_kp0.1_kd0.01_70s_attempt1 | 5.698 / 2.746 | 7.232 | 0.737 |
| precise_kp0.2_kd0.03_30s_attempt1 | 11.100 / 5.124 | 2.177 | 3.856 |

The low-gain precise candidate (kp=0.1, kd=0.01s) completes both durations and improves five of the six angle/rate measures plus tracking versus the 20s OFF reference. Roll peak-to-peak worsens. It still fails the prior maximum-roll 3° and roll-RMS 1.5° goals, and remains unvalidated. The default kp=0.2, kd=0.03s after precise IK gives much larger excursions and tracking error.

Low-gain 20s deltas versus OFF, in table metric order: +22.9%, -9.9%, -39.8%, -39.5%, -25.9%, -24.4%, -7.1%.

Mean and oscillation must be distinguished: OFF mean roll is 2.044° with standard deviation 2.565°; low-gain precise mean roll is 0.380° with standard deviation 1.937°. A lower RMS relative to zero is therefore not proof that every swing amplitude decreased.

Machine-readable results and final JSON paths: `index.json`. Every tuning run has a `.meta.json` with source hashes and its unmodified raw JSON/CSV. Feedback uses delayed/quantized IMU and body gyro; simulator truth is evaluation-only. No hardware validation or phone installation was performed.
