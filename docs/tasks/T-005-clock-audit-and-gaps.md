# T-005: clock audit and gap detection (spike_time, part 1)
Status: blocked (T-002, T-003)
Type: code
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-005-clock-audit
Depends on: T-002, T-003
Design: "Calibration and time sync" row "Header stamp base and latency per topic"; "Failure handling" rows on dropped data and clock base

## Goal
Pure functions that say, per topic, whether header stamps share the receive clock, how large the latency jitter is, and
where messages are missing. Open question 8 and the "camera_ros header.stamp is sensor time or now()" risk (R1) are
answered from this output.

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC4 (the report states how time was matched and its error: this supplies the per-topic clock facts).
- AC1 (dropped-message counts per topic, rate vs expected).
- AC13 support (IMU gaps are the drop signal under recording load, per the design).

## Files expected to change (max 2 source files + their tests)
- `analysis/camera_spike/camera_spike/spike_time.py` (new)
- Test: `analysis/camera_spike/test/test_spike_time.py`

## Approach (step by step)
1. Signatures:
   ```python
   @dataclass(frozen=True) class ClockAudit: n: int; offset_median_ns: int; jitter_ns: int; base_mismatch: bool; max_abs_dev_ns: int
   def clock_audit(recv_ns: np.ndarray, header_ns: np.ndarray, mismatch_threshold_ns: int = 1_000_000_000) -> ClockAudit
   @dataclass(frozen=True) class Gap: start_ns: int; end_ns: int; missing_est: int
   def find_gaps(recv_ns: np.ndarray, nominal_period_ns: int, factor: float = 2.0) -> list[Gap]
   def rate_check(recv_ns: np.ndarray, nominal_hz: float, duration_s: float) -> tuple[int, int, float]   # (received, expected, ratio)
   ```
2. `offset = recv - header`; `offset_median_ns` is its median; `jitter_ns` is the median absolute deviation about the
   median (robust, so a few late frames do not hide a constant); `base_mismatch = abs(offset_median) > mismatch_threshold`.
   Raise `InsufficientPairs` if fewer than 3 samples.
3. `find_gaps`: consecutive differences above `factor * nominal_period_ns`; `missing_est = round(dt / period) - 1`.
   Input must be sorted; unsorted input raises `SpikeError` (do not sort silently).
4. Duplicated receive times (dt == 0) are counted by a third helper `count_duplicates(recv_ns) -> int` and are not gaps.

## Tests required (and which gates must be seen to fail)
- On `SyntheticBag` with `header_base_offset_ns=1.7e18`: `base_mismatch` is True and `offset_median_ns` is within one
  period of the injected value; on a default bag it is False.
- On a bag with `gaps=[('imu', 10, 12)]` the IMU gap is found with `missing_est` within 1 of 100, and the camera
  topic returns `[]`.
- Hand-computed small arrays for `clock_audit` jitter (e.g. offsets 5, 5, 5, 6, 40 -> median 5, MAD 0).
- `rate_check` on a 60 s bag: ratio within 0.02 of 1.0.
- Gate seen to fail: change `base_mismatch` to compare `offset_median` to `mismatch_threshold` without `abs` and confirm the
  negative-offset test fails; restore.

## Follow-ups (found out of scope)
- Applying a constant correction is a later decision: the audit only reports. The report ticket (T-014) says "excluded from
  timing claims" when the offset is not constant.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
