# T-010: time offset between two rate signals (cross-correlation lag with confidence)
Status: blocked (T-002, T-003)
Type: code
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-010-time-offset
Depends on: T-002, T-003
Design: "Calibration and time sync" rows "Camera to IMU offset" and "Camera to Cartographer offset"

## Goal
Estimate the lag between gyro z and a visual or Cartographer yaw rate, and say how sure it is. "Not established" is a
valid result.

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC4 (time matching and its error), and the open-question-8 time-sync unknown the spike must report.

## Files expected to change (max 2 source files + their tests)
- `analysis/camera_spike/camera_spike/time_offset.py` (new)
- Test: `analysis/camera_spike/test/test_time_offset.py`

## Approach (step by step)
1. Signatures:
   ```python
   @dataclass(frozen=True) class LagResult: lag_s: float | None; peak: float; width_s: float; established: bool; reason: str
   def yaw_rate_from_yaw(t_s: np.ndarray, yaw: np.ndarray) -> np.ndarray     # unwrap, then central difference
   def lag_by_xcorr(t_a, a, t_b, b, max_lag_s: float = 0.5, grid_hz: float = 100.0, min_peak: float = 0.5) -> LagResult
   ```
   Convention: positive `lag_s` means signal `b` happens later than `a` (document with an example).
2. Resample both to a common grid by linear interpolation over the overlap, remove mean, normalise, cross-correlate within
   +/- `max_lag_s`. `width_s` is the full width at half maximum of the peak. `established = peak >= min_peak and width_s <
   max_lag_s / 2`; otherwise `lag_s = None` and `reason` says why ("no clear peak").
3. No SciPy dependency: use `np.correlate`/FFT.

## Tests required (and which gates must be seen to fail)
- Synthetic gyro (from `SyntheticBag` turning path) vs the same yaw-rate shifted by 40 ms: recovered lag within one grid
  sample (10 ms) and `established`.
- Sign test: shifting the other way gives -0.040.
- Pure noise vs pure noise (seeded): `established == False`, `lag_s is None` (the "no clear peak" result is produced).
- A signal with no turning (stationary bag): `established == False`.
- Gate seen to fail: invert the sign convention in the code; the sign test must fail. Restore.

## Follow-ups (found out of scope)
- The BNO055 internal latency cannot be separated from the stamping lag with this data (design, "cannot be established").

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
