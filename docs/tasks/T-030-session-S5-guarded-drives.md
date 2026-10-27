# T-030: session S5, five guarded 1 m drives with the recorder running
Status: blocked (T-001 done, T-026, T-029; and the s2 gate in step 1)
Type: job
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-030-session-s5
Depends on: T-001 (done, including criterion 5's tool confirmed against CLAUDE.md s2), T-026, T-029
Design: session S5; "Safety (AC12, AC13, AC14)"; owner decision Q4

## Goal
Record five 1 m guarded drives, with start and stop marks, so that drift per metre (failure B) can be compared between
ground-plane VO, ORB-SLAM3 and Cartographer, and against T-001 criterion 5.

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC3, AC10 B (n of at least 5), AC12 (no new motion code; existing guarded tool only; owner present), AC13, AC1.

## Files expected to change (max 2 source files + their tests)
- None in the repo. Output: one bag, `marks.csv`, `resources.csv`, manifest; capture note
  `docs/test-reports/camera-spike-S5-capture.md`.

## Approach (step by step)
1. **Gate first.** T-001 criterion 5 uses a guarded 1 m tool (`calibrate_drive.py` is the candidate). Read the tool as it exists
   on the robot checkout and show it meets all four CLAUDE.md s2 requirements: clearance guard in the direction of travel checked
   every loop, a did-it-move check across several bearings, publishes zeros in a `finally`, and a leash derived from the distance
   (about 6 s for 1 m, not a flat constant). If it does not, S5 STOPS and the owner decides; this ticket writes no replacement
   (design). The tool publishes `cmd_vel` directly, so Nav2 must NOT be launched.
2. Job rules: **owner present at the robot**, clear 1.5 m or more ahead with a measured tape line, owner within reach of the
   power. Never run unattended; killing the `ssh` does not stop the robot (CLAUDE s2): the stop is the tool's own guard/finally,
   and the owner's hand on the robot.
3. Stack: drivers plus Cartographer, no Nav2, same Cartographer config as S1/S3. Start `res_sampler`, manifest, recorder, then
   `mark_cross.py` with `s` (start) and `e` (stop) before and after each leg.
4. Five legs, each 1 m along the tape, robot returned to the start by hand (slid) between legs, settle before each start. Measure the
   end point against the tape by ruler (three readings) after each leg and write it in the notes.
5. Shutdown: SIGINT recorder, wait for `metadata.yaml`, stop sampler, stop the stack with the motor node first, by PID.
6. Copy to the box after the session; sizes match before deleting on the Pi.
7. Report per leg: commanded distance, ruler end error, did-it-move lidar check value, any guard trips, drops.

## Tests required (and which gates must be seen to fail)
- The tool's clearance guard is seen to work (a hand or box placed in front trips it) BEFORE the first leg, with the wheels
  off the ground or the owner holding the chassis, per the owner's choice. A guard not seen to trip means S5 does not proceed.
- n = 5 legs completed and recorded, or the report says how many.

## Follow-ups (found out of scope)
- Yaw of the camera relative to base_link is fitted from these legs (T-031).

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
