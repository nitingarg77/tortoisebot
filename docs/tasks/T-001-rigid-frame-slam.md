# T-001: rigid-frame SLAM variant for goal runs (no global loop closure)
Status: ready
Type: job
Feature: precision-5cm        Branch: feature/T-001-rigid-frame-slam
Depends on: none (follows A-005, `docs/test-reports/run4-localisation-findings.md`)

## Goal
Find out whether `map -> odom` stays fixed to the start pose, with no jumps, when Cartographer's global
pose-graph optimisation is switched off, and how much local drift remains while the robot drives. This is the
precondition for judging any 5 cm goal on the SLAM stack. On 2026-10-27 the frame jumped 0.41 m at rest, with
loop-closure constraints disagreeing with the local estimate by 0.41 m and 0.14 m in the log.

This ticket measures; it does not claim 5 cm. Hypothesis, not yet shown: with global SLAM off the jumps stop and
local drift over a 1 m path is small enough to use. Either result is a finding.

## Acceptance criteria satisfied (AC numbers from the requirements)
There is no requirements document yet; these are this ticket's own criteria.
1. A variant config exists that differs from `slam_real.lua` only in the setting that turns global SLAM off, and
   a desk test pins that (every other line identical).
2. The default launch is unchanged: with the variant not selected, Cartographer loads `slam_real.lua` as before.
3. On the robot, with the variant selected and the robot untouched on the start cross for at least 10 minutes,
   the logs contain no `constraint_builder_2d` "differs by" lines, and `map -> base_link` shows no step above
   2 cm (report n, max step, spread).
4. Cross 10 (0, 0) is recorded after the robot has been moved around Set P, and again after a second pass;
   report each against the plan, n of at least 3 passes, mean and spread.
5. Drift per metre: the robot is driven 1 m along x by a guarded script, 5 times, and the pose error against the
   tape is reported (CLAUDE.md section 1: n, mean, spread).
6. The report states plainly if the jumps did not stop, or the drift is too large to use.

## Files expected to change (max 2 source files + their tests)
- `tortoisebot_slam/config/slam_real_rigid.lua` (new): a copy of `slam_real.lua` with one setting changed.
- `tortoisebot_slam/launch/cartographer.launch.py`: a launch argument that selects the variant; default off.
- Test: a new file under `tortoisebot_navigation/test/` or `tortoisebot_slam/test/` (whichever `run_tests.sh`
  collects; add the directory to `run_tests.sh` if needed) comparing the two `.lua` files line by line.

## Approach (step by step; exact paths, signatures, example inputs/outputs)
1. Read `tortoisebot_slam/config/slam_real.lua`. It currently has `POSE_GRAPH.optimize_every_n_nodes = 90` and
   `POSE_GRAPH.constraint_builder.min_score = 0.65`. Check, against the installed Cartographer version
   (`dpkg -l | grep cartographer`) and its source or docs at that version (CLAUDE.md section 1: the installed
   version, not the branch head), what turns global optimisation off. The expected setting is
   `POSE_GRAPH.optimize_every_n_nodes = 0`; do not assume it.
2. Create `slam_real_rigid.lua` as a copy with only that setting changed, and a comment giving the reason and the
   source of the setting.
3. In `cartographer.launch.py`, add a launch argument (name chosen to match the file's existing style, default
   false) that picks `slam_real_rigid.lua` when it is true and `slam_real.lua` otherwise. The existing selection
   is at the `lua_basename` branches (is_odom_only, use_sim_time). Do not change the other branches.
4. Add the desk test: parse both files as lines, assert the only differing lines are the intended one(s). Mutate
   the variant to see the test fail, then restore (the "gate seen to fail" requirement).
5. Run `env PATH=/usr/bin:/bin:$PATH PYTHONDONTWRITEBYTECODE=1 ./run_tests.sh`; expect the existing 245 plus the
   new tests.
6. On the robot (owner present; heavy job, ask first): pull, then check the value landed by looking at the
   Cartographer log for the loaded configuration basename and the running node's parameters.
7. Run the measurements in acceptance criteria 3 to 5. Reuse the scratch tools `~/logs/drift.py` and
   `~/logs/readpose.py` and `nav_course.py --pose-source tf` on the robot. Settle for at least 4 minutes before
   reading after any move (2026-10-27: 1 to 4 minutes).
8. Write the results to `docs/test-reports/`, as a new report, with a result line and an "Not verified" section.
9. Rollback: launch without the argument. Nothing else changes.

## Tests required (and which gates must be seen to fail)
- The variant-vs-original comparison test, seen to fail when the variant is edited to differ elsewhere, then
  restored and seen to pass.
- A launch-argument test only if the selection logic can be pulled into a ROS-free function (CLAUDE.md
  section 3); otherwise the launch file is covered by criterion 2 and 6 on the robot, and the report says that
  no test covers it.
- Not covered by any desk test: Cartographer's behaviour, so criteria 3 to 5 are robot measurements.

## Follow-ups (found out of scope)
- If the jumps stop but drift is too large: Cartographer pure localisation on a saved pose graph, or a cleaner
  map for AMCL (AMCL's spread was 0.26 to 0.41 m on 2026-10-27).
- The exploration supervisor `~/logs/explore_run.py` did not flag a 5-minute stall (its `frontier=` field was
  always `None`). It lives on the robot only, outside the repo.
- Pose settling after a move depends on the Pi's CPU load (load average 9 to 18 with Cartographer running); a
  fan was suggested after the run-3 temperature of 82.7 C.
- The plan `run4-precision-5cm-plan.md` needs its preflight (expect 0.03) and pose-source notes updated.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
