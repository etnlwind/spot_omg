# V77-T1 rear-leg diagnostic — 2026-09-17

User selected installed V6.2.5; isolate one rear leg while other three command targets remain S. Original RL dynamic probe completed; user observed continuous floor contact. Trace artifacts: `artifacts/v77-t1/rl01`. This observation is not a measured clearance value.

A subsequent static J3 +5 degree move was not the requested dynamic gait test. It is not evidence of gait clearance. J3 was restored to its S target before the next installation attempt.

## Lift extension candidate

Revision `s-native-v6-2-7-v77-t1-lift`, still diagnostic V77-T1, not a new registered gait. `rearprobe rl|rr [LIFT_MM]` optionally sets total nominal peak lift (16–40 mm); omitted argument retains original trajectory. V625 baseline is 12 mm. Only selected swing Z is increased after computing baseline J1 locks. X path, phase windows, J1 locks and protection logic remain. Other three final command targets are S. Probe requests stop after four seconds of drive, with eight-second abort bound.

Initial intended physical trial: RL 20 mm, original V625 input 344. Larger settings require separate observation; no clearance threshold has been established.

Validation: firmware build 318040 bytes, SHA256 d7b540a7b5c62c6d5017f78de278dabdb04b9d2d4a7ff71ba5b0f860dc18eb0e. C regression checks RL/RR 20/28/36 mm swing Z increments, unchanged J1 and X, unchanged nonselected Z, stop completion. Existing V625 C/Python tests: 13 passed. These are host tests, not physical clearance or stability verification.

## Installation blocked before OTA

Landing preparation stopped on motion tilt protection after 2430 ms. No OTA took place. Installed revision remains `s-native-v6-2-7-v77-t1`, profile V625. Follow-up readback: custom pose, torque on, safety OK, fault 0, stationary servos, 12.1–12.4 V. Cleared fault/status does not establish successful Landing. New dynamic 20 mm test has NOT run.

Evidence: `artifacts/v77-t1/lift-install/prepare.json`, `blocked-state.json`. Await level body support and clear folding space before retrying measured Landing, torque-off, OTA and post-install verification. Do not bypass failed Landing.

## Subsequent installation and RL 20 mm trials

User confirmed level support. Retried Landing completed; all 12 torque-off readbacks passed. OTA and post-boot Landing completed, persistent servo settings unchanged, V625 reselected. Installed revision is now `s-native-v6-2-7-v77-t1-lift`.

RL 20 mm trial 01 stopped normally at 6607 ms; IMU and joint traces downloaded. All nine nonselected joint targets had zero excursion. RL J2 actual excursion increased from 16.17 to 20.04 degrees; J3 from 9.49 to 13.10 degrees versus original 12 mm trial. J3 peak tracking error increased from 6.15 to 9.14 degrees. Peak absolute IMU roll/pitch 4.3/2.0 degrees. These are commanded/encoder motion comparisons, not proof of ground clearance. User missed seeing trial 01 and requested repeat.

Trial 02 stopped normally at 6612 ms. Artifacts: `artifacts/v77-t1/rl-lift20-01` and `rl-lift20-02`. Await user observation before increasing height further.

## 20 mm user observation and full-walk probe

User confirmed the RL foot left the floor on trial 02, then requested walking with this version. This confirms observable RL clearance under the supported single-leg test conditions; no minimum clearance height or all-leg stability is inferred.

Full-walk diagnostic revision `s-native-v6-2-7-v77-t1-walk` adds bounded `walkprobe` for idle V625 only: fixed 20 mm on all four swing trajectories, input 344, four-second stop request and eight-second abort bound. No S masking is applied. Ordinary registered V625 behavior remains unchanged; this command is a diagnostic, not a renamed/default gait model. Standard tracking, IMU protection, startup and stop paths remain active. C host tests now also cover all-leg lift and stop; existing V625 regression: 13 passed.

Full-walk firmware installed with completed Landing, torque-off readback, OTA and post-install persistent settings verification. `walk20-01` executed and stopped on tilt protection at 2938 ms; no automatic retry. IMU roll ranged -7.3 to +13.1 degrees; pitch -1.2 to +1.7. J3 peak sampled tracking errors FL/FR/RL/RR: 8.88/9.58/8.61/8.35 degrees. Observed J3 excursions: 13.10/13.62/14.24/13.80 degrees. Lowest sampled J3 voltage 11.5 V. These do not establish torque saturation. All joint/IMU trace downloads validated; baldiag prompt timed out with 20 lines retained. Final state custom, torque ON, safety tilt, fault14. S return not completed; await physical observation before any further motion.
