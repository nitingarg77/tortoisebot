# T-009: spike metrics (per-cross error, n/mean/spread/worst, rest step, lost intervals, drift per metre)
Status: blocked (T-002)
Type: code
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-009-spike-metrics
Depends on: T-002
Design: "Comparisons (AC6, AC7, AC10)"

## Goal
Every number the report prints, computed by one tested module, so the report never contains a hand-computed figure.

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC6 (position error per cross in cm, visual and Cartographer).
- AC7 (n, mean, spread; tracking-loss intervals with where and how long, not dropped).
- AC9, AC10 A/B/C (rest step, drift per metre, stop error).

## Files expected to change (max 2 source files + their tests)
- `analysis/camera_spike/camera_spike/spike_metrics.py` (new)
- Test: `analysis/camera_spike/test/test_spike_metrics.py`

## Approach (step by step)
1. Signatures (all lengths in metres in, centimetres out where the name says `_cm`):
   ```python
   @dataclass(frozen=True) class Stats: n: int; mean: float; spread: float; worst: float; best: float   # spread = sample std (ddof=1); NaN if n < 2
   def summarise(values) -> Stats
   def per_cross_error_cm(est_xy: np.ndarray, plan_xy: np.ndarray) -> np.ndarray          # Euclidean distance, NaN rows kept as NaN and excluded from Stats
   def pose_in_window(t_ns, xy, window: Window) -> tuple[np.ndarray, float, int]           # (median xy, in-window spread cm, n samples); empty -> NotAvailable
   def step_at_rest(t_ns, xy, window: Window) -> tuple[float, float]                       # (max consecutive step cm, max spread from median cm)
   @dataclass(frozen=True) class Loss: start_ns: int; end_ns: int; duration_s: float; nearest_cross: str | None
   def loss_intervals(t_ns, state, min_gap_ns: int, cross_times: dict[str, int]) -> list[Loss]
   def drift_per_metre(start_xy, end_xy, tape_len_m: float) -> float                        # cm of end-point error per metre of tape
   def pass_summary(per_cross_cm) -> Stats ; def across_passes(list[Stats]) -> Stats ; def across_repeats(list[Stats]) -> Stats
   ```
2. `loss_intervals`: a loss is a run of `state == 'lost'` rows or a time gap above `min_gap_ns` between consecutive rows;
   never interpolates; `nearest_cross` is the label whose time is closest to the interval midpoint.
3. `drift_per_metre` uses the 2D distance between measured end-point error against the tape: `|end - start - tape_vector|` is
   not available without direction, so define it as `| |end-start| - tape_len | / tape_len * 100` AND also return lateral error
   via a second function `lateral_error_cm(start_xy, end_xy, heading_xy)`. State both in the docstring so the report names which
   one it quotes (T-001 criterion 5 reports "pose error against the tape"; match it, and say which).
4. Spread uses sample std and the docstring says so (CLAUDE s1: n, mean, spread).

## Tests required (and which gates must be seen to fail)
- Hand-computed cases: errors [3, 4, 5] cm -> n 3, mean 4.0, spread 1.0, worst 5.
- `summarise` of one value gives spread NaN, not 0.0 (a single run has no spread; gate: change to 0.0, test fails).
- `step_at_rest` on a trajectory with a 0.41 m step injected inside the window returns about 41 cm.
- `loss_intervals`: input with a 2.0 s hole gives one interval of 2.0 s with the right nearest cross; **gate seen to fail:**
  linearly interpolate across the hole before measuring and the test must fail (lost intervals are not interpolated).
- `across_passes` of three `Stats` uses the pass means as the samples.
- NaN rows are excluded from n and counted in a returned `n_nan` (add field to `Stats`).

## Follow-ups (found out of scope)
- Choosing which drift definition the owner prefers is an owner decision (flagged in the index).

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
