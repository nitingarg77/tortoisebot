# T-025: session S0 preflight on the robot (versions, recorder first test, raw clip, ruler height)
Status: blocked (T-001 done including its robot measurements, T-019, T-021, T-022, T-023)
Type: job
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-025-session-s0
Depends on: T-001 (finished, its report written), T-019, T-020, T-021, T-022, T-023
Design: "Sessions" row S0; "Recording design"; Risk R1, R7, R9

## Goal
Learn what the design could not read from the box: `camera_ros` behaviour, the real size and rate of the compressed stream, the
recorder's first deliberate test, and the true camera height above the floor.

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC1 (per-topic rate and size measured), AC13 (recording without changing the control launch), enabling AC2/AC4.
- Requirements open questions 8, 9, 10.

## Files expected to change (max 2 source files + their tests)
- None in the repo. Output: `docs/test-reports/camera-spike-S0-preflight.md` (new) and a copy of the manifest in `captures/`
  on the Pi (not committed).

## Approach (step by step)
1. Job rules: robot, **owner present**, no motion (the robot stays at rest, drivers only: no Cartographer, no Nav2). Follow CLAUDE.md
   s2/s4 and the run recipe. Kill by PID, never `pkill` by name. This session comes AFTER T-001's robot measurements; the
   recorder was not run during them.
2. Pre-checks: `df -h ~` shows at least twice the estimated size free (S0 total expected well under 1 GB). `timedatectl show`.
3. Read versions: `dpkg -l | grep -E 'camera-ros|rosbag2|libcamera'`, OS, RAM, the installed `camera_ros` source at that
   version for (a) whether `image_raw/compressed` is produced only while subscribed, (b) which thread encodes, (c) the `jpeg_quality`
   default, (d) whether `header.stamp` is sensor time or `now()` (CLAUDE s1: read the installed version, not the branch head).
4. `ros2 param get /camera_node jpeg_quality` (record the live value). Do not change it in this ticket.
5. `record_capture.sh --dry-run --spike ...`, then a real 60 s spike recording at rest with `session_manifest.py start/finish` and
   `res_sampler.py --arm rest`. Confirm: the silence guard, refuse-overwrite, SIGINT closes the bag (metadata.yaml exists),
   `ros2 bag info` output. Check whether `ros2 bag reindex` exists on the installed Humble.
6. 30 s raw clip via `--raw-clip 30`.
7. Ruler: camera lens height above the floor and its forward offset from the axle, three measurements, mean and spread; state
   whether `base_link` is at floor or axle height (the open R9 unknown).
8. Copy the bag to the box by hand (`rsync`), compare sizes and `ros2 bag info` on both, delete on the Pi only after they match.
   Do this only after the recording has ended (AC15).
9. Report: per-topic rate/size (n, mean, spread over the 60 s), JPEG bytes per frame, which parts of the design are confirmed or
   falsified, estimate for total storage, and the real Cartographer log line that holds the config basename (T-022 regex
   check). "Not verified" section.

## Tests required (and which gates must be seen to fail)
- The recorder's refuse-overwrite is seen to fail by starting a second recording into the same directory name (expected: it
  refuses). The silence guard is seen to fire by running with the camera node stopped (owner agrees; expected refusal after 30 s).
- Gate: if any desk script misparses a real value (manifest field null, `res_sampler` empty cell), that is reported and a fix
  ticket is raised, not patched here.

## Follow-ups (found out of scope)
- If JPEG cost is already clearly over budget, bring the `jpeg_quality` / nice / raw fallbacks to the owner before S1.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
