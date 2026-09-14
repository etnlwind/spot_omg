# 24mm wave3 bounded shooting result

60 candidate evaluations completed; 0 passed all quality gates. No further optimization was run.

The five body-wave coefficients were searched within ±2mm of stable rank3. Lift stayed at 24mm, period at 1.2s, transfer width/lead/gain at 0.12s / 0s / 0.3, and the legacy balance path was disabled. Every candidate used a 25s steady evaluation (5–30s), then Stop. Model, motor and contact parameters were unchanged. Ground-truth states and contact forces scored the offline objective only. No dynamic observer/truth feedback hook was enabled.

Per-cycle statistics below use the explicit **target phase**, including its 40ms lead. Clearance is the minimum or 10th percentile of each complete swing peak, not a single best frame. Leg order is FL / FR / RL / RR. Contact is the fraction of middle-swing samples with contact.

| Candidate | Speed deg/s | Max roll / pitch deg | RMS roll / pitch deg | Tracking RMS deg | Cycle minima mm | Cycle p10 mm | Mid-swing contact % |
|---|---:|---:|---:|---:|---|---|---|
| wave3_9188efc5c9ec.json | 17.658 | 4.087 / 2.027 | 1.375 / 0.637 | 2.189 | 10.21 / 16.85 / 14.98 / 27.39 | 10.26 / 16.87 / 15.01 / 27.46 | 22.07 / 11.11 / 11.11 / 5.32 |
| wave3_c18893a9a449.json | 17.754 | 3.685 / 1.797 | 1.261 / 0.579 | 2.199 | 11.40 / 17.52 / 15.35 / 27.30 | 11.48 / 17.60 / 15.41 / 27.44 | 21.81 / 11.11 / 11.11 / 6.91 |
| wave3_2936db30fc0e.json | 17.750 | 6.558 / 3.050 | 1.704 / 0.779 | 2.219 | 3.03 / 1.08 / 7.76 / 5.97 | 8.14 / 14.91 / 14.23 / 24.30 | 23.40 / 12.43 / 11.64 / 8.51 |

Best-cost candidate `wave3_9188efc5c9ec` retained speed and RMS tilt limits, and completed Stop without a safety fault. It failed maximum roll (4.087° > 3°), per-cycle clearance (FL 10.21mm; RL 14.98mm < 15mm), middle-swing contact (FL 22.07%; FR/RL 11.11% ≥ 10%), and tracking (2.189° versus 1.864° baseline). It is not a validated replacement for the stable 22mm experiment.

The optimizer reached its fixed 60-evaluation budget; SciPy success=false reports the iteration limit. This is independent of gait validity: all 60 candidates had validated=false.

Every JSON records source SHA-256 before and after evaluation. Eight trials overlapped source edits by the parent (opt-in dynamic support was being added); these remain flagged and preserved, not silently excluded. The best and second-ranked candidates have unchanged before/after hashes. Their cases contain no `arc_dynamic_trial` key. The initial manifest and per-candidate hashes both remain available.

Artifacts: `wave3-manifest.json`, `wave3-optimization.json`, `wave3.log`, and all 60 `../arc-control-2026-09-12/wave3_*.json`. Script: `simulation/mujoco/scripts/tuning/optimize_arc_wave3.py`. This is MuJoCo estimated-physics validation only; no hardware readback, position-hold or full-motion hardware validation was performed.
