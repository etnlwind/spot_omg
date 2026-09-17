# J2 85° after support — 2026-09-17

Offline candidate based on preserved V6.2.7; not installed on hardware.
The first FR/RL entry is unchanged. FL/RR become eligible after their initial
support half-cycle; FR/RL after their first landing and support half-cycle.
Eligibility uses commanded phase history, not measured propulsion or contact.
The recovery bump concurrently approaches J2=85° and J3=104°. It does not
increase the preceding stance propulsion trajectory. A separate propulsion
trajectory change would require another experiment.

Input 344, nominal gait timing unchanged, 11.1 V pack model, collisions and
tilt protection enabled. Recording: 2 s wait, requested 8 s walk, 4 s stop.
The requested walking duration was not completed.

- Host regression: first FR/RL entry unchanged, subsequent pair eligibility,
  simultaneous J3 target, J1 preservation and stop bypass: 1 test passed.
- At video 3.40 s, FL/RR J2 targets ~82.0°, actual ~80.5/80.6°.
  Their clearances are 43.2/11.3 mm; both have zero ground load.
  Roll is +6.1°, with FR/RL carrying the support loads.
- At 3.54 s (walk +1.54 s), tilt protection activates, roll +16.46°.
  RR has recontacted the floor while RL has lost support.
- At 4.00 s the simulated body crosses the 90° topple observation threshold.

Outcome: failed. Excluding entry corrected the scheduling error but did not
make the large recovery motion stable. Joint feedback follows the large J2
command in this model; these results do not establish real hardware limits.
They also do not isolate the cause among support placement, dynamic reaction,
and recovery trajectory. Do not deploy this candidate as a gait fix.

Evidence: trajectory.json, summary.json, telemetry.csv, four-views.mp4.
