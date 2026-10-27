# T-017: pass evaluation pipeline and the end-to-end desk run on synthetic data
Status: blocked (T-003, T-005, T-006, T-007, T-008, T-009, T-010, T-014, T-015)
Type: code
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-017-pipeline-e2e
Depends on: T-003, T-005, T-006, T-007, T-008, T-009, T-010, T-014, T-015
Design: "Test strategy": "An end-to-end desk run: SyntheticBag through ScriptedVO, alignment, metrics and report"

## Goal
Glue the pure stages into `evaluate_pass(...)` and a small CLI, and prove the chain with known answers before any real bag
exists, so that a surprise on real data is about the data and not the code.

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC6, AC7, AC8 (per-cross comparison with fits, per-pass n/mean/spread, before/after).
- AC9 / AC11 (thresholds blank -> "not judged"; report ends as required).

## Files expected to change (max 2 source files + their tests)
- `analysis/camera_spike/camera_spike/pipeline.py` (new)
- `analysis/camera_spike/camera_spike/__main__.py` (new, thin CLI: `python3 -m camera_spike evaluate --dataset D --vo-traj T --marks M --plan P --out O`)
- Test: `analysis/camera_spike/test/test_end_to_end.py`

## Approach (step by step)
1. `evaluate_pass(dataset_dir, traj: Trajectory, tracking: TrackingLog, marks: list[Mark], plan_xy: dict[str, tuple[float,float]],
   thresholds) -> PassResult` does, in order: load `scans.npz` and `carto.csv`; `fit_threshold` is taken as an argument (from the
   rest session); `stationary_windows` (T-006); `associate`; for each matched cross the median pose from `traj` and from
   `carto` in the window (T-009 `pose_in_window`); `align_all_labels` (T-008); errors in cm before and after for every label;
   `loss_intervals`; `step_at_rest` over the p10 rest window; `PassResult` holds everything the report tables need.
2. `plan_xy` is loaded from a YAML (`tape.yaml`, the Set P plan, written by hand from `run4-precision-5cm-plan.md` section 2;
   optional ruler offsets per placement). This ticket ships a parser and a sample file under `test/data/`; the real plan values
   are not guessed here.
3. CLI prints the report (T-014) to stdout, writes figures only if matplotlib imports (no failure if absent).
4. Nothing here imports ROS.

## Tests required (and which gates must be seen to fail)
- Build a `SyntheticBag` of a 10-cross hand-placed pass (stationary windows at known plan positions, moves between them) and a
  `ScriptedVO` with scale 2.0, rotation 30 deg, a 1 s lost interval. `evaluate_pass` must: match all 10 crosses; show large
  U0 error and small U1/F-tape-LOO error; list the lost interval with the nearest cross; produce a report ending with
  `Result`, `Not verified`, `Recommendation`, with every judged cell `not judged`.
- Same run with a `carto_jump` at rest: the A-type event is reported (jump size within 1 cm of injected); a run with none says
  "none occurred" (AC10 A wording).
- With only 2 matched crosses: F-tape labels are `NotAvailable` and the report says why; no crash.
- **Gate seen to fail:** break one stage on purpose (e.g. associate offset by 100 s) and confirm the end-to-end assertions
  fail; restore.

## Follow-ups (found out of scope)
- Real-data glue (file paths per session) is exercised in T-034.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
