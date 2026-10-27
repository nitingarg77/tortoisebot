# T-004: render synthetic floor frames and checkerboard frames through a known camera
Status: blocked (T-002)
Type: code
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-004-render-frames
Depends on: T-002
Design: "Interfaces and contracts" row "Camera images" (the `cam_ws/src/product_counter` synthetic-frame pattern)

## Goal
A renderer that produces images of a textured floor plane (for the ground-plane VO) and of a checkerboard (for
calibration), seen through a camera with known K, distortion, tilt, height and pose. Lets T-011, T-012 and T-013 check
against ground truth with no camera.

## Acceptance criteria satisfied (AC numbers from the requirements)
Enabling ticket for AC5, AC6, AC8 (known-answer visual input). Supports the "calibration" and "time sync" unknowns the
design assigns to the spike (open questions 8, 9).

## Files expected to change (max 2 source files + their tests)
- `analysis/camera_spike/camera_spike/render.py` (new)
- Test: `analysis/camera_spike/test/test_render.py`

## Approach (step by step)
1. Read how `cam_ws/src/product_counter` builds its synthetic frames (look for its test fixtures) for the style only.
2. Implement, imports `numpy`, `cv2`:
   ```python
   def make_floor_texture(seed: int, size_m: tuple[float, float] = (6.0, 4.0), px_per_m: int = 400) -> np.ndarray   # uint8 gray, random blobs + lines (wood-grain-like)
   def camera_K(width=640, height=480, hfov_deg=53.5) -> np.ndarray     # fx = (w/2)/tan(hfov/2); principal point at centre
   def render_floor(texture, px_per_m, K, *, tilt_deg, height_m, pose_xy_yaw, size=(640, 480), dist=None) -> np.ndarray
   def render_board(K, dist, rvec, tvec, *, pattern=(9, 6), square_m=0.025, size=(640, 480)) -> np.ndarray
   ```
   `render_floor`: camera sits at `(x, y)` of the floor plane, `height_m` above it, yaw from `pose_xy_yaw`, looking forward
   and pitched down by `tilt_deg`. Build the homography from floor coordinates to image with `K [r1 r2 t]` and use
   `cv2.warpPerspective` with `WARP_INVERSE_MAP`. If `dist` is given apply it by `cv2.undistortPoints`/remap inverse
   (simple radial k1, k2 is enough). Document the camera axis convention used (x forward, y left, z up for the robot;
   the usual optical x right, y down, z forward for the camera).
   `render_board`: draw a black/white board as a planar texture and warp it with the pose; it must be detectable by
   `cv2.findChessboardCorners` for a tilt up to 40 deg.
3. Tests may use small sizes (320x240) to stay fast. The whole file must run in a few seconds.

## Tests required (and which gates must be seen to fail)
- A point on the floor at a known ground position projects to the pixel the renderer drew it at (render a single bright
  dot texture, find it with `cv2.minMaxLoc`, compare to the analytic projection within 1 px).
- Moving the pose forward 0.10 m moves a floor dot in the image in the expected direction by the expected amount.
- `render_board` output: `cv2.findChessboardCorners` succeeds on at least 8 of 10 random poses.
- Gate seen to fail: pass `height_m` 20 percent wrong to the analytic projection in the first test; it must fail.

## Follow-ups (found out of scope)
- Rolling shutter and motion blur are not simulated. The report must not claim robustness to either from synthetic data.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
