# T-019: recorder: spike topic list, 1 GB splits, disk gate and `--dry-run`
Status: blocked (T-018)
Type: code
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-019-recorder-spike-mode
Depends on: T-018
Design: "Recording design": Topics and rates, Bag storage; Risk R7 (`--dry-run` is the reviewable part)

## Goal
Make `record_capture.sh` produce the spike's bag exactly as designed and make that command inspectable without ROS or the
robot, so the review and the desk test can read what would run.

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC1 (camera images, IMU, `/scan`, `/tf`, `/tf_static` recorded at modest rates).
- AC3 (single bag per pass; 1 GB splits stay one bag directory).
- AC13 (nothing in this script changes control launch; it only subscribes).

## Files expected to change (max 2 source files + their tests)
- `tortoisebot_firmware/scripts/record_capture.sh`
- Test: `tortoisebot_firmware/test/test_record_capture_dryrun.py`

## Approach (step by step)
1. Add `--dry-run`: prints the exact `ros2 bag record ...` command line, one argument per line prefixed `ARG:`, and exits 0
   before any ROS call, silence check, mkdir or disk write. Everything else unchanged in behaviour.
2. Add `--spike` (or make the topics configurable by `--topics-file`; pick whichever fits the script's existing style, state the
   choice in the commit). The spike topic list must be exactly:
   `/camera_node/image_raw/compressed /camera_node/camera_info /imu /scan /tf /tf_static /floor_scan /scan_fused /rosout`.
   `/camera_node/image_raw` (raw) must NOT be in it. A separate `--raw-clip SECONDS` mode records only
   `/camera_node/image_raw` for the S0 30 s clip, refusing durations over 60 s.
3. Add `--max-bag-size 1000000000`, `--storage sqlite3`, no `--compression-mode`.
4. Add `--min-free-gb N`: before recording, `df -P --output=avail -B1G` of the output filesystem must be at least N or the
   script refuses (non-zero exit, clear message). Default N is a required argument in spike mode (no hidden default).
5. Keep: refuses to start if the compressed topic is silent for 30 s, refuses to overwrite a directory, SIGINT closes the
   bag and prints `ros2 bag info`. Do not weaken any of them. The output directory name is `spike_<YYYYMMDD>_<session>`
   (take `<session>` as an argument; reject names outside `[A-Za-z0-9_]`).
6. Output to stderr/stdout only; the script never publishes anything.

## Tests required (and which gates must be seen to fail)
- `--dry-run --spike` output contains each of the nine topics, `--max-bag-size 1000000000`, `--storage sqlite3`,
  does not contain `/camera_node/image_raw ` (raw, with trailing space or end of line), and does not contain `--compression`.
  **Gate seen to fail:** add the raw topic or drop `/imu` in a copy and the test must fail.
- `--dry-run` with an existing output directory still exits 0 (it creates nothing) while a real run refuses: assert only the
  dry-run side at the desk; the refusal is read from the script and exercised in S0.
- Invalid session name (`a b`, `../x`) exits non-zero.
- `--raw-clip 120` exits non-zero.
- The test puts a fake `ros2` on PATH that records any invocation to a file and asserts it was NOT called by `--dry-run`.
- The fast suite count is above zero for this directory (silent-zero check).

## Follow-ups (found out of scope)
- Real behaviour (silence guard, SIGINT close, disk gate) is verified in S0.
- `ros2 bag reindex` availability on Humble is checked in S0 for the crash-recovery row.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
