# Two entry steps before large recovery — 2026-09-17

Offline V6.2.7 candidate; no firmware changes or hardware deployment.
Both FR/RL and FL/RR entry swings retain the baseline policy. The added
J2=85/J3=104 recovery starts only after entry_phase=1, with zero initial
bump weight. The prior after-push experiment remains reproducible using
--after-push. This changes large recovery timing, not the stance push curve.

Host tests: 2 passed, including preservation throughout both entry swings.
Simulation: input 344, nominal timing unchanged, collision and tilt protection
enabled; 2 s wait, requested 8 s walk, 4 s stop. Actual walking cut short.

At video 4.22 s entry completes: commanded J1 is approximately -8.992°
for all legs. Actual J1 FL/FR/RL/RR: -9.126/-9.835/-7.876/-8.121°.
Thus target symmetry is achieved; exact actual symmetry is not established
by the phase gate (actual spread 1.96°). Roll is +1.95°.

At 4.68 s tilt protection activates: roll -16.81°, pitch -4.49°.
FR/RL J2 target ~84.99°, actual 84.64/85.77°. FR clearance is 64.5 mm,
whereas RL is in contact at -3.2 mm with 13.1 N normal force.
The body crosses the 90° topple observation threshold at 5.12 s.

Result: failed. Preserving both entry steps avoids the previous early
failure, but delaying the large recovery alone does not stabilize it.
Do not infer hardware limits or successful propulsion from this experiment.
Evidence: trajectory.json, summary.json, telemetry.csv, four-views.mp4.
