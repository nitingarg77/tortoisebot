# T-028: session S3, 10-minute rest recording on p10
Status: blocked (T-026)
Type: job
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-028-session-s3
Depends on: T-026 (cost known, any fallback decided by the owner)
Design: session S3; "Comparisons" bullet "Rest (S3)"; owner decision Q4

## Goal
Record 10 minutes with the robot untouched on cross p10 with the full stack, to measure each system's step and spread at rest,
fit the stillness threshold, and catch any Cartographer jump-type event if one occurs.

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC9 (step at rest), AC10 A (steadiness at rest over at least the Q4 rest duration; if no jump occurs the report says so),
  AC1.

## Files expected to change (max 2 source files + their tests)
- None in the repo. Output: bag in `captures/`; short capture note in `docs/test-reports/camera-spike-S3-capture.md`.

## Approach (step by step)
1. Job rules: robot at rest on p10, owner present, no motion at all, same launch and config as S1 (manifest checks), no Nav2 goal.
2. Settle at least 4 minutes before marking `start` (T-001 follow-up: settling 1 to 4 min). Record `start`/`stop` with `mark_cross.py`.
3. Record exactly 10 minutes in spike mode with `res_sampler --arm rest`. The owner does not touch the robot or walk in front of it
   (note anyone who does).
4. Copy to the box after `stop` (AC15); sizes match before deleting on the Pi.
5. Report only what was recorded: duration, gaps counted from IMU/scan, rest-phase Cartographer `/rosout` "differs by" lines
   if any (count, text). Analysis happens in T-031 to T-034.

## Tests required (and which gates must be seen to fail)
- Gate: `metadata.yaml` exists and `ros2 bag info` duration is within 5 s of 600 s; otherwise the session is repeated, not accepted.

## Follow-ups (found out of scope)
- If a jump-type event occurs, A becomes in scope per Q1; tell the owner.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
