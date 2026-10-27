# T-024: build ORB-SLAM3 v1.0 on the box (heavy job)
Status: blocked (T-016; owner approval to run a heavy build)
Type: job
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-024-orbslam3-build
Depends on: T-016
Design: "Candidate visual systems" row ORB-SLAM3; Risk R8; AC14

## Goal
Produce a pinned, working ORB-SLAM3 monocular binary on the box, outside the repo, and record exactly what was built. Confirm the
log lines T-016 parses.

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC5 (system and version named), AC14 (heavy visual SLAM only on the box, one heavy job at a time, owner asked first).

## Files expected to change (max 2 source files + their tests)
- None in the repo except the report `docs/test-reports/camera-spike-orbslam3-build.md` (new).
- If a parser regex in `orbslam3_adapter.py` needs correcting, that is a follow-up code ticket, not done here.

## Approach (step by step)
1. Job rules: heavy, box, never inline. Ask the owner first; one heavy job at a time; use `nice` and a time cap; do not run while
   ComfyUI or another heavy job holds the machine (oomd kills at memory pressure). CPU build only, no GPU, no `gpuq` needed
   (a CUDA-free build). Never kill a build mid-JIT; if it must stop, tell the owner.
2. Build under `/workspace/robotics/third_party/ORB_SLAM3` (or the path the owner names; do not put it under `tb_ws/src`
   without a `COLCON_IGNORE`). Pin tag v1.0 and record the commit hash in `version.txt` beside the binary.
3. Expect the known Ubuntu 22.04 patches (C++ standard, Pangolin, OpenCV version). Record every patch as a file diff stored in the
   report. Do not change the algorithm.
4. Run the shipped monocular example on a public sequence ONLY if one is already on the box; otherwise run the binary
   with `--help`/no args and record the usage message. No download in this ticket without asking.
5. Run it on the T-004 synthetic frames dumped to a temp dataset (a tiny script run at the desk to produce 200 frames) to see that
   it starts, and capture its stdout. Compare the "Creating new map"/"Lost" text with the regexes in T-016; list differences.
6. Record whether this version has a switch to disable loop closing (design wants runs with and without).
7. Write the report: version, patches, build time, binary path, memory peak, loop-closing switch yes/no, "Not verified".

## Tests required (and which gates must be seen to fail)
- The synthetic-frame start-up run is the gate: the binary must be seen to start and produce `CameraTrajectory.txt`, or the
  report says it did not. A failed build is a valid result and is reported with the compiler error.

## Follow-ups (found out of scope)
- stella_vslam, COLMAP, DPVO/DROID are escalations, each its own ticket, only if CPU systems lose tracking and the owner allows GPU (Q14).

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
