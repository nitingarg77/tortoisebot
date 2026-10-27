# T-026: session S1, Pi cost of recording (arms A0, A1, B, twice, interleaved)
Status: blocked (T-025)
Type: job
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-026-session-s1
Depends on: T-025
Design: "Pi cost budget and how it is measured (AC1, AC2, AC13)"; session S1

## Goal
Measure what recording costs the Pi, and whether the existing consumers (`floor_scan`, `/scan_fused`) and the control loop
(Cartographer, Nav2) change, against a baseline taken the same way.

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC1 (CPU, load average, temperature, with n and spread), AC2 (extra cost against a baseline, owner limit 0.25 cores),
  AC13 (control loop unchanged), AC15 (rates measured before any copy).

## Files expected to change (max 2 source files + their tests)
- None in the repo. Output: `docs/test-reports/camera-spike-S1-cost.md` (new). Raw files stay in `captures/` (Pi) and the box copy.

## Approach (step by step)
1. Job rules: robot at rest on cross p10, owner present, no motion, same launch scripts/args as the Set P sessions, no Nav2
   goal. Use the Cartographer config T-001 recommends (D3); the manifest records the config basename read from the Cartographer log.
2. Arms, 5 min each, in this order: A0, A1, B, A0, A1, B (design). A0 = stack plus `res_sampler` only. A1 = A0 plus a light bag of
   `/floor_scan`, `/scan_fused`, `/camera_node/camera_info`. B = A0 plus the full spike bag (T-019 `--spike`). Label rows with `--arm`.
3. Take `res_sampler` at 2 s; kill by PID between arms; wait for temperature to be within the same band before starting the next
   arm (record the starting temperature).
4. Compute (on the box, from the copied `resources.csv` and bags, never `ros2 topic hz` on the Pi): per-process cores
   (`camera_node`, recorder, Cartographer, Nav2 servers), whole-Pi busy cores, load average, temperature, throttled bits; each with n,
   mean and spread; extra cost = B minus A0; `camera_node` minus its A0 value separately; `/floor_scan`, `/scan_fused`, `camera_info`
   rates in B against A1 (within the A1 block-to-block spread); Cartographer rest spread and step count (FINDINGS Table 1 method),
   work-queue lines count, Nav2 "missed its desired rate" counts; IMU/scan gap counts per arm as the drop signal.
5. State plainly if n = 2 blocks per arm cannot separate a change from noise.
6. If B is over 0.25 cores, list the design's fallbacks in order and what each would cost; do NOT apply one. Applying
   `jpeg_quality` change needs the owner's agreement; record the live value before and after (`ros2 param get`).

## Tests required (and which gates must be seen to fail)
- Gate: the interleaving and arm labels are checked from the data (each arm appears twice, rows labelled correctly); a run in
  which an arm is missing is reported incomplete, not averaged.
- The measured "extra cost" has a stated n and spread; the report prints "not judged" for pass/fail if the owner's 0.25 limit
  is not yet in `thresholds.yaml` (it is pre-filled; the report uses it).

## Follow-ups (found out of scope)
- Decision on `nice` for the recorder, taken from this data.
- A fan (T-001 follow-up) is not part of this ticket.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
