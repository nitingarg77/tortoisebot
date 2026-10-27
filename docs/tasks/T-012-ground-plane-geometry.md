# T-012: ground-plane geometry (pixel to floor coordinates, planar rigid motion from point pairs)
Status: blocked (T-002, T-004, T-007)
Type: code
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-012-ground-plane-geometry
Depends on: T-002, T-004, T-007
Design: "Candidate visual systems", row "Ground-plane homography VO"; "Pure modules", `ground_plane_vo`

## Goal
The metric core of the ground-plane VO with no image processing: given matched pixel pairs from two frames, K, camera tilt
and height, return the planar motion (dx, dy, dyaw) in the robot ground frame.

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC5 (a visual system run offline; this is the metric one the design chose for B).
- AC8 (this system is metric by construction: height gives scale; a wrong height must visibly change the result).

## Files expected to change (max 2 source files + their tests)
- `analysis/camera_spike/camera_spike/ground_plane_geometry.py` (new)
- Test: `analysis/camera_spike/test/test_ground_plane_geometry.py`

## Approach (step by step)
1. Signatures (numpy only; cv2 not needed):
   ```python
   @dataclass(frozen=True) class CamMount: height_m: float; tilt_deg: float; forward_m: float = 0.0   # tilt positive = looking down
   def pixel_to_ground(uv: np.ndarray, K: np.ndarray, mount: CamMount) -> tuple[np.ndarray, np.ndarray]   # (N,2) ground xy in the camera-foot frame, and a (N,) valid mask (ray hits the floor in front)
   @dataclass(frozen=True) class PlanarMotion: dx: float; dy: float; dyaw: float; n_inliers: int; n_pairs: int; rms_m: float
   def planar_motion(uv_prev, uv_curr, K, mount, iters: int = 100, tol_m: float = 0.01, seed: int = 0) -> PlanarMotion
   ```
   `planar_motion` back-projects both point sets to the floor, then RANSAC-fits a 2D rigid transform (T-007 `se2_fit`
   with scale fixed) and refits on inliers. Returns the motion of the camera foot (frame k-1 to frame k), expressed in the
   frame-k-1 axes. Raise `InsufficientPairs` if fewer than 6 valid pairs.
   (Design deviation, minor: the design says "homography decomposed with known K, tilt and height". Back-projecting to the
   known plane and fitting a rigid transform is the same geometry with fewer failure modes and a direct unit test. Record this in
   the module docstring and in the test report.)
2. Rays that do not hit the floor (at or above the horizon) are masked invalid, not clipped.
3. Document axes: robot x forward, y left; optical z forward, y down; `tilt_deg` rotates about the camera x axis.

## Tests required (and which gates must be seen to fail)
- With `render_floor` (T-004) at `height 0.134, tilt 4.5`: render two views 0.10 m apart, hand-pick 30 known floor dots by
  projecting them analytically (no image detection here), `planar_motion` returns dx = 0.10 +/- 0.002, dy and dyaw about 0.
- Pure rotation of 20 deg about the camera foot returns dyaw within 0.3 deg.
- 30 percent outlier pairs (random pixels) still recovers the motion.
- Fewer than 6 valid pairs raises `InsufficientPairs`; points above the horizon are masked.
- Gate seen to fail: pass `height_m` 10 percent wrong and the 0.10 m test must fail by about 10 percent (proves scale comes
  from height); pass `tilt_deg` wrong by 3 deg and see the error. Restore.

## Follow-ups (found out of scope)
- The true height and forward offset come from S0 (ruler). Until then tests use the URDF values (0.134 m, 4.5 deg) as inputs only.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
