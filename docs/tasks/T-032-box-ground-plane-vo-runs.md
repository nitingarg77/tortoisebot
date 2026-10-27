# T-032: box stage 3a: ground-plane VO on every pass (3 repeats)
Status: blocked (T-013, T-031)
Type: job
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-032-gp-vo-runs
Depends on: T-013, T-031
Design: "Candidate visual systems" (secondary, metric); "Offline analysis" paragraph on non-determinism and repeats; R11

## Goal
Run the ground-plane VO on S3, S4 (each pass) and S5, producing trajectories and tracking logs, and measure how much of the real floor
the tracker can actually follow.

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC5 (system and version named), AC7 (loss intervals, with where and how long), AC10 B inputs.

## Files expected to change (max 2 source files + their tests)
- None in the repo. Outputs under `vo/ground_plane_vo/<repeat>/` in the data path; summary in
  `docs/test-reports/camera-spike-gpvo-runs.md`.

## Approach (step by step)
1. Job rules: box CPU, light; ask the owner first; one job at a time; `nice`. No GPU. Each run is one process; a non-zero exit
   or partial output is discarded and re-run, never merged.
2. For each bag/pass run the VO with the S0 ruler height and the calibrated K (and, as a sensitivity row, height +/- 10 percent
   to show how scale error propagates; reported next to the main result, not instead of it).
3. 3 repeats per pass even though this VO is nearly deterministic: confirm by comparing the repeats (a spread of zero is itself a
   finding, stated as such).
4. Record per frame feature counts and inlier counts (`tracking.csv`) and the lost intervals; compute image sharpness per frame
   (Laplacian variance) for the blur question (R5), and frame brightness for the auto-exposure question.
5. Report: runtime, memory peak, loss intervals per pass, fraction of frames tracked, sharpness during turns vs straights.
   "Not verified".

## Tests required (and which gates must be seen to fail)
- Gate: on one real stationary window the VO's drift is reported against the tape's zero motion; a trajectory that moves more than
  1 cm at rest is flagged.
- Gate: repeats are compared; any repeat with a non-zero exit is excluded and counted.

## Follow-ups (found out of scope)
- Floor texture too poor to track is a finding, and the next step is the owner's call (more texture, tags, ORB-SLAM3 only).

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
