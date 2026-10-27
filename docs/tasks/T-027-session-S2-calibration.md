# T-027: session S2, camera calibration capture (checkerboard held and moved; board on the floor)
Status: blocked (T-025)
Type: job
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-027-session-s2
Depends on: T-025
Design: session S2; "Calibration and time sync" rows on intrinsics, pitch/roll and height

## Goal
Record the footage that T-031 turns into intrinsics, distortion, tilt and height, with a printed checkerboard supplied by the
owner. The robot does not move.

## Acceptance criteria satisfied (AC numbers from the requirements)
- Open question 8 (calibration unknown). Supports AC5, AC8 (metric scale from a verified height).

## Files expected to change (max 2 source files + their tests)
- None in the repo. Output: bag in `captures/`, then `docs/test-reports/camera-spike-S2-capture.md` (short, new).

## Approach (step by step)
1. Job rules: robot at rest, drivers only, owner present, owner holds the board, no Cartographer needed. Recorder in spike mode,
   but the compressed topic is what is calibrated. S0 already recorded a raw clip for the JPEG comparison.
2. The owner supplies the board's inner-corner count and square size (printed measurement of the square with a ruler); write
   both in the manifest notes. Do not assume 9x6 / 25 mm.
3. About 2 min: hold the board at 8 or more different distances, tilts and image positions, including corners of the frame; steady
   for 1 s per pose. Then 30 s with the board flat on the floor at a measured position (tape-measure the position from the axle
   centre, three readings).
4. Record the raw clip again in the same pose set if S0's raw clip had no board (30 s).
5. Copy to the box as in T-025 step 8.
6. Report: what was recorded, board parameters, floor-board position readings (mean, spread), frame count with a detectable board
   (counted at the box in T-031). "Not verified": no calibration claim is made here.

## Tests required (and which gates must be seen to fail)
- Gate: before leaving the robot, run `T-011 detect_corners` on a handful of copied frames at the desk/box (light) to confirm the
  board is detectable; if fewer than 20 poses detect, repeat the capture. This is the "seen to work before leaving" check.

## Follow-ups (found out of scope)
- If calibration fails to converge, S2 is repeated with a larger board; not guessed.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
