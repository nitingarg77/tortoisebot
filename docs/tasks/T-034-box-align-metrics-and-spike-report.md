# T-034: box stages 4 and 5: alignment, metrics and the spike report in docs/test-reports
Status: blocked (T-017, T-026, T-031, T-032, T-033, T-001 done)
Type: job
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-034-spike-report
Depends on: T-017, T-026, T-031, T-032, T-033, and T-001's report (for the Cartographer figures to compare)
Design: "Scale and alignment (AC8)", "Comparisons (AC6, AC7, AC10)", report ending (AC11)

## Goal
Produce the one Phase 1 report: every visual trajectory compared with the tape and with Cartographer, under all five alignments,
with n, mean and spread, tracking losses, S1 cost, rest steps, B drift per metre, and an explicit recommendation. Thresholds blank,
so every verdict is "not judged".

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC1, AC2, AC4, AC6, AC7, AC8, AC9, AC10 (A, B, C), AC11, AC13, AC15. (AC5 satisfied by T-024, T-032, T-033; AC12 and AC14 by construction.)

## Files expected to change (max 2 source files + their tests)
- `docs/test-reports/camera-assisted-localisation-spike.md` (new; from `docs/templates/test-report.md`).
- Figures beside it only if matplotlib exists on the box; the report works without them.

## Approach (step by step)
1. Job rules: box CPU, light; ask first; reads datasets only.
2. For each pass and each VO system and repeat run `evaluate_pass` (T-017). Use the stillness threshold from S3 (T-031). Report per
   pass tables (T-014): cross, mark times, window bounds, offset between them, in-window spread, error under U0, U1, F-carto,
   F-tape-all, F-tape-LOO, and Cartographer error. Then n/mean/spread/worst per pass, across passes, across the 3 repeats.
3. C: per-cross error at stop under U1 and F-tape-LOO with the tape floor ("a few cm", Q5; ruler offsets from S4 if recorded)
   and an explicit statement on whether the error can tell 5 cm from 10 cm given that floor.
4. B: end-point error per metre for the 5 S5 legs from ground-plane VO, ORB-SLAM3 (U1) and Cartographer; compare with T-001 criterion 5.
5. A: step and spread at rest in S3 for each system and Cartographer; if no Cartographer jump occurred in any recording, say so.
6. Cost and side effects from S1 (AC2, AC13); calibration, extrinsics, time offsets from T-031; tracking losses with where and how long.
7. Result line, "Not verified" section (rolling shutter, auto-exposure effects, BNO055 latency, anything unmeasured), and a recommendation:
   tracks well / does not / inconclusive with what to measure next. It does NOT claim 5 cm (the renderer enforces that).
8. Hand-computed spot check: three cells recomputed by hand from the CSVs and noted in the report.

## Tests required (and which gates must be seen to fail)
- Gate: the report generator is run on the T-017 synthetic data first and its known answers are matched, then on real data.
- Gate: every figure in the report traces to a CSV in `results/<session>/` listed in the report appendix.
- Reviewer reruns the generator with one cross deliberately dropped from the plan and sees the unmatched/NotAvailable handling.

## Follow-ups (found out of scope)
- Phase 2 scope, tags, a better tape reference (Q5), COLMAP/learned systems: each requires the owner's decision after reading the report.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
