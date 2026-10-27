# T-029: session S4, Set P hand-placed passes (at least 3, one bag each, with marks)
Status: blocked (T-026, T-028)
Type: job
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-029-session-s4
Depends on: T-026, T-028 (S3, so the stillness threshold is known before passes), T-020
Design: session S4; "Marking cross arrivals"; Risk R3; owner decision Q4

## Goal
Record at least three Set P passes, the robot hand-placed on crosses p1 to p10, so that the camera's stop-pose measurement
(failure C) can be compared with the tape and with Cartographer.

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC3 (single bag per pass, owner present, existing Set P method), AC4 (marks), AC6/AC7 inputs (at least the Q4 number of passes),
  AC1.

## Files expected to change (max 2 source files + their tests)
- None in the repo. Output: one bag, `marks.csv`, `resources.csv` and manifest per pass; capture note
  `docs/test-reports/camera-spike-S4-capture.md`.

## Approach (step by step)
1. Job rules: robot at the owner's hands, the owner present; no `cmd_vel` publisher is started by this ticket. If any drive tool is
   used it must not be: S4 is hand-placed only. Same launch and Cartographer config as S1/S3 (checked in the manifest).
2. Pre-pass: `df` gate (twice the estimate), start `res_sampler --arm rest` (or `B`), `session_manifest.py start`, start the
   recorder (spike mode) and wait for `ros2 bag info`-visible data, then `mark_cross.py`.
3. The pass follows the PLAN section 2 order p1..p10; the owner SLIDES (not lifts) the robot with the owner standing behind it
   (R3); press Enter at arrive and leave; `n <text>` for touches or re-placements. Wait at each cross for the pose to settle
   (1 to 4 min, FINDINGS), as in run 4.
4. Optional (Q5, cheap): ruler offset of chassis to cross at each placement (PLAN section 5 method), written to `tape.yaml`
   by hand after the pass.
5. After the last cross: SIGINT to the recorder, wait for `metadata.yaml`, then stop the sampler, then the stack (motor node first),
   by PID. Copy to the box; delete on the Pi only after sizes match.
6. Repeat for pass 2 and pass 3 (and more if time allows; more is allowed, fewer is not). Each pass is its own bag.
7. Report per pass: duration, bag size, drops (IMU/scan gaps), marks recorded versus expected (20 events), ssh drops, anything the
   owner noted.

## Tests required (and which gates must be seen to fail)
- Gate per pass: the marks file has the expected 20 `arrive/leave` events or the missing ones are listed (stationary windows recover
  the order); a bag without `metadata.yaml` is flagged and recovered/reindexed per the design before it counts.
- The count of passes is stated with n; fewer than 3 is reported as short of the Q4 requirement.

## Follow-ups (found out of scope)
- Lifting/legs-in-view effects are reported by analysis (T-033), not controlled here.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
