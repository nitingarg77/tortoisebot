# T-033: box stage 3b: ORB-SLAM3 monocular on every pass (3 repeats, loop closing on and off if possible)
Status: blocked (T-024, T-016, T-031)
Type: job
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-033-orbslam3-runs
Depends on: T-016, T-024, T-031
Design: "Candidate visual systems" (primary); "Offline analysis" repeats; R4, R8, R11; AC14

## Goal
Run ORB-SLAM3 on S3, S4 passes and S5 to learn whether a standard visual SLAM can track this robot, with each pass repeated 3 times
because the algorithm is multi-threaded and not deterministic.

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC5, AC7 (each lost sub-map reported with time and duration; only the largest aligned, others listed), AC14 (heavy, box only,
  owner asked first, one at a time).

## Files expected to change (max 2 source files + their tests)
- None in the repo. Outputs under `vo/orbslam3/<repeat>/`; summary in `docs/test-reports/camera-spike-orbslam3-runs.md`.

## Approach (step by step)
1. Job rules: heavy CPU job on the shared box. Ask the owner first; one run at a time; `nice`; time cap per run (the adapter's
   timeout); never while ComfyUI or another heavy job holds the machine; no GPU. A non-zero exit or partial TUM is discarded and
   re-queued, never merged.
2. For every pass (S3, each S4 pass, S5) run with loop closing on, and with it off if T-024 found a switch; 3 repeats each. Record
   the exact command and version in `run.json`.
3. Every tracking-lost event and new Atlas map is recorded with time, duration, nearest cross (T-009 `loss_intervals`).
4. Report per pass and per repeat: tracked fraction, number of maps, longest loss, runtime, memory peak. Spread across repeats is
   reported next to the spread across passes.
5. Say plainly if ORB-SLAM3 failed to initialise (low parallax, mostly floor): that is a result. Escalation to COLMAP or DPVO/DROID
   is not started; it is a decision for the owner, GPU only with the owner's agreement (Q14) and via `gpuq`.

## Tests required (and which gates must be seen to fail)
- Gate: the repeat count is verified from the output directory (3 per pass per setting), or the missing ones are listed.
- Gate: the T-016 log parsing is checked against a real run's stdout; a mismatch is reported as a follow-up code ticket.

## Follow-ups (found out of scope)
- COLMAP/GLOMAP, DPVO/DROID-SLAM, stella_vslam: separate tickets only on escalation.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
