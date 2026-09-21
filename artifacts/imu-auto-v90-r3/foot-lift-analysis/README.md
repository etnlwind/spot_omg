# Front lift 30 mm investigation

Read-only iPhone trace confirms successful footlift save and robot readback FL=30 FR=30 RL=0 RR=0, profile attitudepd_v4, firmware V90-R3. Subsequent drive attempts report motion tilt safety limit reached. This does not establish the physical cause of lower front-foot clearance.

Host sampling: 1,000 phase samples for each direction at linear magnitudes 0.588 and 1.0, yaw=0, full startup amplitude. Current firmware locomotion_targets -> foot_lift_profile -> arc_foot, before IMU stabilization and servo dynamics. No IK rejections. Front added peak 30 mm, rear added peak 0 mm. Z span is model/body-coordinate range, not measured ground clearance. Actual movement, mounting/calibration, body pitch and tracking still need measured comparison. No robot movement or firmware changes performed.
