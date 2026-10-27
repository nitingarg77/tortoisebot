# T-008: leave-one-out alignment error and floor-plane fit with height-derived scale
Status: blocked (T-007)
Type: code
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-008-loo-floor-plane
Depends on: T-007
Design: label F-tape-LOO; "Scale and alignment" paragraph on ORB-SLAM3 U1 scale; "Calibration" rows on pitch/roll/height

## Goal
(1) The fair version of the tape fit: each cross is scored by a fit that did not use it. (2) A floor-plane fit to 3D points
that gives camera height and tilt, and from the known camera height a metric scale for a monocular trajectory.

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC8 (scale handling and before/after error; LOO is what keeps the fit honest).
- AC10 C (per-cross error at stop against the tape under U1 and F-tape-LOO).

## Files expected to change (max 2 source files + their tests)
- `analysis/camera_spike/camera_spike/spike_align.py` (add `loo_errors`, `align_all_labels`)
- `analysis/camera_spike/camera_spike/floor_plane.py` (new)
- Tests: add to `test/test_spike_align.py`; new `test/test_floor_plane.py`

## Approach (step by step)
1. In `spike_align.py`:
   ```python
   def loo_errors(src: np.ndarray, dst: np.ndarray, with_scale: bool = True) -> np.ndarray   # (N,) 2D distance; point i is scored with a fit that excludes i
   def align_all_labels(est_xy, est_yaw, est_t_ns, cross_est_xy, cross_plan_xy, carto_xy_at_est_t, p10_window, height_scale: float | None) -> dict[str, AlignResult]
   ```
   `AlignResult` = `Sim2 | NotAvailable-with-reason`, plus `error_before`/`error_after` arrays per cross (cm). Labels exactly:
   `U0, U1, F-carto, F-tape-all, F-tape-LOO`. U0 applies nothing. U1 anchors to `p10_window` and applies `height_scale`
   (None -> `NotAvailable("no floor plane / no scale")`). LOO needs at least 4 crosses (3 to fit plus the held-out one),
   otherwise `NotAvailable`.
2. In `floor_plane.py` (numpy only):
   ```python
   @dataclass(frozen=True) class Plane: n: np.ndarray; d: float; inliers: np.ndarray   # n.x + d = 0, |n| = 1
   def fit_plane(points: np.ndarray, iters: int = 200, tol: float = 0.01, seed: int = 0) -> Plane   # RANSAC then SVD refit; InsufficientPairs if < 10 points
   def camera_height(plane: Plane, cam_origin=(0.0, 0.0, 0.0)) -> float
   def plane_tilt_deg(plane: Plane, cam_down_axis=(0.0, 1.0, 0.0)) -> float
   def scale_from_height(known_height_m: float, measured_height_units: float) -> float
   ```
3. Sign and axis conventions go in the docstring (optical frame: z forward, y down).

## Tests required (and which gates must be seen to fail)
- LOO error on noisy synthetic crosses is greater than the all-in fit residual, on 20 seeds, at least 18 times.
- **Gate seen to fail (design requirement):** change `loo_errors` so the held-out point is included in its own fit, and the
  "LOO >= all-in" test must fail; restore and see it pass.
- `align_all_labels` returns exactly the five labels; U1 is `NotAvailable` when `height_scale=None`; fewer than 4 crosses
  makes LOO `NotAvailable` while F-tape-all still works with 3.
- `fit_plane` on 500 points on a plane with 20 percent outliers recovers n and d (angle < 0.5 deg, d < 2 mm); recovers
  camera height 0.134 m and tilt 4.5 deg from a rendered-geometry plane; raises on 5 points.
- `scale_from_height(0.134, 0.20)` is 0.67.

## Follow-ups (found out of scope)
- Which height is true (R9) is decided in S0 by ruler; this module only consumes it.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
