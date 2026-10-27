# T-016: ORB-SLAM3 subprocess adapter (settings file, run, parse trajectory and tracking log)
Status: blocked (T-002, T-003, T-011)
Type: code
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-016-orbslam3-adapter
Depends on: T-002, T-003, T-011
Design: "Candidate visual systems" (ORB-SLAM3 monocular, tag v1.0); "Interfaces" row "Visual odometry"; failure rows on lost / new map / NaN

## Goal
A `VisualOdometry` implementation that drives an external ORB-SLAM3 binary and turns its output into `Trajectory` and
`TrackingLog`. The binary is not built or run in this ticket; the desk tests use a fake executable.

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC5 (system and version named: the adapter reads the tag/commit from a `version.txt` beside the binary and records it).
- AC7 (every Atlas sub-map after a tracking loss is reported with time and duration; only the largest is aligned, the others
  listed, not dropped).
- AC14 (never runs on the Pi: the adapter refuses if `/boot/firmware` or an aarch64 `platform.machine()` is detected).

## Files expected to change (max 2 source files + their tests)
- `analysis/camera_spike/camera_spike/orbslam3_adapter.py` (new)
- Test: `analysis/camera_spike/test/test_orbslam3_adapter.py`

## Approach (step by step)
1. `class OrbSlam3VO(VisualOdometry)`: `__init__(self, binary: Path, vocab: Path, loop_closing: bool = True, timeout_s: int = 7200)`.
2. `write_settings(calib: CameraModelLike, fps: float, out: Path)`: the ORB-SLAM3 monocular YAML (`Camera.fx, fy, cx, cy,
   k1, k2, p1, p2`, `Camera.fps`, `Camera.RGB`, ORB parameters at the upstream defaults). Cite the v1.0 example file in a
   comment. If the pinned version has no switch for loop closing, `loop_closing=False` raises `NotAvailable` (design: record it).
3. `run(dataset_dir, calib, out_dir)`: write an image list (`recv_ns` converted to seconds relative to the first frame; keep the
   offset to map back), run `subprocess.run([binary, vocab, settings, dataset_dir/images, times.txt, out_dir], timeout=...)`
   with `cwd=out_dir`, capture stdout/stderr into `out_dir/run.log`, store the exact command in `out_dir/run.json`.
   Non-zero exit or timeout: raise `SpikeError` and leave the partial files marked `PARTIAL` so they are never merged
   (failure row "VO job dies").
4. Parse `CameraTrajectory.txt` (TUM) per Atlas map: ORB-SLAM3 writes one trajectory; sub-maps are recognised by the
   `tracking` log it emits ("Creating new map", "Lost"). Parse those stdout lines into `TrackingLog`; document the regexes
   against the pinned version's log text (checked in T-024 on the real binary, noted `unverified` until then).
5. Non-finite rows are dropped and counted. Output times are mapped back to the bag receive clock.
6. The fake for tests is a small Python script at `test/fake_orbslam3.py` made executable by the test (counts as test
   support, not a source file) that writes a known TUM file and known log lines.

## Tests required (and which gates must be seen to fail)
- With the fake binary: trajectory rows, count, time mapping (to 1 ms) and tracking log match what the fake wrote; two sub-maps
  produce two listed maps, the largest selected, none dropped.
- Fake exits 1: `SpikeError` and a `PARTIAL` marker; fake sleeps past a 1 s timeout: same.
- A NaN row is dropped and counted.
- `loop_closing=False` with a binary whose `version.txt` says "no-switch" raises `NotAvailable`.
- Refuses on a faked `platform.machine() == 'aarch64'`.
- Gate seen to fail: have the adapter fill the lost interval by repeating the last pose, and the lost-interval test must fail.

## Follow-ups (found out of scope)
- The log-line regexes are confirmed against the real build in T-024 and T-033.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
