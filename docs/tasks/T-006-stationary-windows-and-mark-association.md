# T-006: stationary windows from /scan and mark-to-window association (spike_windows)
Status: blocked (T-002, T-003)
Type: code
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-006-stationary-windows
Depends on: T-002, T-003
Design: "Marking cross arrivals (AC4)", source 2; "Failure handling" row "Stationary window not found or ambiguous"

## Goal
Find the stretches where the robot sat still, fit the stillness threshold from the rest session (not picked by hand), and
match each owner mark pair (arrive/leave) to the window it overlaps most. Unmatched and ambiguous are explicit results.

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC4 (cross arrival times matched to trajectory samples, with the match error reported).
- AC6, AC7 (the per-cross pose is the median over the matched window; unmatched crosses are reported, not dropped).

## Files expected to change (max 2 source files + their tests)
- `analysis/camera_spike/camera_spike/spike_windows.py` (new; the design's "spike_time" second half, split to keep tickets small)
- Test: `analysis/camera_spike/test/test_spike_windows.py`

## Approach (step by step)
1. Signatures:
   ```python
   def scan_motion_stat(ranges: np.ndarray) -> np.ndarray      # (N,B) -> (N-1,): median |r[i+1]-r[i]| over beams finite in both
   def fit_threshold(rest_ranges: np.ndarray, quantile: float = 0.99, margin: float = 1.5) -> float   # quantile of the stat on a known-still run, times margin
   def stationary_windows(scan_t_ns: np.ndarray, ranges: np.ndarray, threshold: float, min_duration_s: float = 5.0) -> list[Window]
   @dataclass(frozen=True) class CrossMatch: label: str; status: str; window: Window | None; offset_s: float | None; candidates: int
   def associate(marks: list[Mark], windows: list[Window]) -> list[CrossMatch]
   ```
2. `scan_motion_stat` ignores inf/NaN beams (design: NaN and inf excluded from stillness statistics) and returns NaN
   for a pair with no common finite beams; NaN counts as "not still".
3. `stationary_windows`: runs of consecutive stat values below `threshold`, converted to `[t_i, t_j]`, kept if length >=
   `min_duration_s`.
4. `associate`: pair each `arrive` with the next `leave` for the same label. Overlap of `[arrive, leave]` with each window
   (seconds). Best overlap wins: `status='matched'`, `offset_s` = window.start minus arrive time. If the second-best overlap
   is at least 50 percent of the best, `status='ambiguous'` and `candidates` is the count. No overlap: `'unmatched'`. A
   label with `arrive` but no `leave` pairs to end of data. Notes (`event='note'`) never form a pair. Record the 50 percent
   and 1.5 margin rules in the docstring as implementer choices, with the reason.
5. Never drop a mark: output has one `CrossMatch` per `arrive`.

## Tests required (and which gates must be seen to fail)
- `SyntheticBag` path: 3 stationary spans (40 s, 20 s, 3 s) separated by moves; `stationary_windows` returns the first two
  within one scan period of `GroundTruth.stationary_intervals()` and not the 3 s one (below `min_duration_s`).
- `fit_threshold` on a still run, then used on the moving/still bag: no false windows during motion.
- `associate`: a mark pair inside one window -> matched; a pair between windows -> unmatched; a pair spanning two windows of
  similar length -> ambiguous with `candidates == 2`; arrive without leave handled; output length equals the number of arrives.
- NaN/inf beams (`nan_beams=20`) do not change the windows.
- Gate seen to fail: make `associate` silently drop unmatched marks; the length test must fail. Restore.

## Follow-ups (found out of scope)
- The stability of the fitted threshold across Pi load is an S3 / S4 finding, not a desk test.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
