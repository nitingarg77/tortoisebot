# Test report: run 4 preparation, localisation findings (2026-10-27, ticket A-005)

Result: **NOT RUN, BLOCKED.** Set P (the 10-goal 5 cm gate) was not driven. The localiser, not Nav2, stops the
5 cm target from being verifiable: neither Cartographer in SLAM mode nor AMCL on a freshly saved map held a
fixed frame to within 5 cm. Nothing here shows whether the 0.03 m tolerance and the other Nav2 changes work.
Command: stack on the robot via `~/logs/run_robot.sh` + `~/logs/run_compute.sh` (SLAM) and
`~/logs/run_compute_amcl.sh` (AMCL, scratch, robot only); owner present throughout; one exploration run of
7 min 13 s, otherwise the robot was moved by hand.
Collected / passed / failed / skipped: goals sent by a script: **0**. Desk checks (`./run_tests.sh`): 245 passed.

Branch `feature/precision-5cm`, robot and box at `5cd966c` before the stack was started. Plan:
`run4-precision-5cm-plan.md`. Raw logs on the box: `/workspace/robotics/tb_ws/robot_logs/run4pre/` (SLAM
drift runs, exploration run, saved map) and `.../run4set/` (Set P recording, compute and robot logs).

## What was checked and what it showed

| # | Check | Result |
|---|---|---|
| 1 | Live parameters on `/controller_server` | `general_goal_checker.xy_goal_tolerance` 0.03, `FollowPath.xy_goal_tolerance` 0.03, `FollowPath.max_vel_x` 0.10, `trans_stopped_velocity` 0.02, `progress_checker.plugin` `nav2_controller::PoseProgressChecker`. The plan's preflight expected 0.05; the file has had 0.03 since `b553bc2`. |
| 2 | Stationary drift, SLAM mode, 5 runs of 60 s | See table 1. Max spread within a run 1.6 to 2.6 cm. |
| 3 | Exploration run (frontier, new speed cap and `occ_threshold` 50), started 10:27:50 | Stopped by the owner's SIGINT at t = 433 s: the robot had hovered at about (2.2, 0.65) in front of a wall since t of about 130 s. Distance 4.41 m. 9 "Failed to make progress" lines in the compute log. The supervisor's `frontier=` field stayed `None` and it raised no stall. The supervisor's own map save failed (rc 1); saving by hand worked. |
| 4 | AMCL on the hand-saved map | Did not converge. See table 2. |
| 5 | Set P crosses, recorded with `nav_course.py --record --pose-source tf` | 9 of 10 recorded within 3 cm of the plan once settled. Cross 10 not recorded. See table 3. |
| 6 | Frame stability while the robot sat on (0, 0) | The `map` frame jumped by **0.41 m** in y with the robot not touched. See "The jump". |

### Table 1: stationary drift, SLAM mode (`map -> base_link`, robot untouched, 10:19 to 10:24, stack up since about 10:10)

| Run | n | Mean x, y (m) | Max excursion from mean (cm) | Max spread (cm) |
|---|---|---|---|---|
| 1 | 190 | 0.0126, 0.0201 | 1.74 | 2.57 |
| 2 | 177 | 0.0109, 0.0202 | 1.02 | 1.77 |
| 3 | 180 | 0.0131, 0.0201 | 0.92 | 1.78 |
| 4 | 181 | 0.0065, 0.0186 | 0.85 | 1.59 |
| 5 | 181 | 0.0090, 0.0199 | 1.10 | 1.68 |

This is at rest only. It does not describe the pose after the robot has been moved (below).

### Table 2: AMCL on `~/maps/run4_20261027.yaml` (187 x 131 cells, 5 cm, 17.6 m^2 free)

| Moment | Pose x, y, yaw | Std dev x, y (m) |
|---|---|---|
| After initial pose (0, 0, 0), robot at rest | -0.075, 0.116, 0.00 | 0.34, 0.55 |
| After a hand push of about 20 cm and a turn | 0.286, -0.069, -0.71 | 0.25, 0.20 |
| Initial pose re-sent, robot at rest | -0.025, 0.022, 0.00 | 0.21, 0.21 (the initial covariance) |
| After a second push (14 updates) | 0.211, -0.076, -0.59 | 0.36, 0.21 |
| After a hand-driven loop of about 2 m, robot back on the start cross (true pose 0, 0, 0) | -0.086, -0.120, 0.00 | 0.26, 0.41 |

One reading each, so these are observations, not measurements. A stray `initpose.py 99 99 0` was published to
`/initialpose` by mistake between rows 2 and 3 (operator error, not intended); the later rows follow a deliberate
re-send of (0, 0, 0). Causes were not tested. Candidates: the long open corridor (3 m or more free ahead),
doubled and noisy walls in the saved map (the map put the wall near goal 8 at 0.28 m; the tape says 0.40 m),
and CPU starvation (load average 13 to 18 with AMCL and Cartographer both running).

### Table 3: Set P crosses recorded in SLAM mode (pose source `tf`), against the plan

| Cross | Plan x, y | Recorded x, y | Note |
|---|---|---|---|
| p1 | 0.50, 0.00 | 0.521, 0.005 | recorded at once |
| p2 | 1.00, 0.00 | 1.021, 0.019 | recorded at once |
| p3 | 1.00, 0.50 | 0.985, 0.512 | first reading **0.99, 0.79** (tape: cross is at y = 0.50); steady at that wrong value for 6 s; correct after moving the robot away and back |
| p4 | 0.50, 0.50 | 0.492, 0.516 | read 0.747, 0.636 then 0.599, 0.571 then spread 12 cm, settled after about 3.5 min |
| p5 | 0.00, 0.50 | 0.001, 0.504 | settled after about 4 min |
| p6 | 0.00, -0.50 | -0.003, -0.485 | single-sample jumps of 7 to 10 cm in two of four read windows, means agreed |
| p7 | 0.50, -0.50 | 0.496, -0.496 | settled |
| p8 | 1.00, -0.50 | 0.978, -0.478 | tape: 0.40 m to the nearest wall; map said 0.28 m |
| p9 | 0.50, 0.00 | 0.514, 0.008 | y read 0.10 to 0.23 for about 2 min, then settled |
| p10 | 0.00, 0.00 | not recorded | see below |

Poses are the SLAM estimate while the owner placed the chassis mid-axle on a taped cross by hand (placement good
to a few cm). The saved file is `places_run4.yaml` on the robot, copied to `robot_logs/run4set/`; it has p1 to p9.

## The jump

At cross 10 the robot sat on (0, 0). Three reads from 11:36:30 to 11:37:14 gave (0.00, 0.016). Reads at 11:37:35,
11:38:10 and 11:38:33 gave (-0.02, 0.41), (-0.03, 0.43) and (-0.03, 0.43), steady. The owner confirmed the robot
was not touched and was on (0, 0). The compute log (`run4.set.compute.log`) shows what Cartographer was doing: from
11:37:51 many `constraint_builder_2d.cc:275` lines of the form "Node (0, 2566) ... on submap (0, N) differs by
translation 0.41", and at 11:39:24 "differs by translation 0.14". These are loop-closure constraints disagreeing
with the local estimate by 0.41 m, then 0.14 m. Cartographer applies such corrections by moving `map -> odom`.

The same stack shows its pose-graph work queue backed up (114 items at 11:07:33, 0 later) around the time cross 4
was slow to settle. That the backlog caused the slow settling is a hypothesis: it fits the timing, the other
crosses were not matched to queue length, and the backlog was not instrumented.

So the `map` frame on the SLAM stack is not rigid to the floor. It can move by tens of cm at any time, including
at rest. A goal reached "within 5 cm" in that frame says nothing about the floor.

## Changes made and checked

- `nav_course.py` gained `--pose-source {amcl,tf}` (`2ef1269`, pushed, robot pulled; default `amcl`). It exists
  because `--record` and the goal runs read only `/amcl_pose`, which the SLAM stack does not publish.
- Desk: `./run_tests.sh` 245 passed (firmware 37, navigation 193, imu 15), names check clean over 54 files, ruff
  clean. No test covers the TF path. On the robot it supplied the pose for all 9 recordings above, so it works
  there; goal runs with it were not done.

## Gates seen to fail

None applicable. No test gate was added or changed, so there was nothing to break and restore.

## Failures

- No Set P goal was sent, so no goal failed. The failures are in the localiser, tables 2 and 3 and "The jump".
- The exploration run hovered near a wall for about 5 minutes without the supervisor noticing; the owner saw it.
  Run 3 showed the same pattern (9 "Failed to make progress"). With `PoseProgressChecker` and the 0.10 m/s cap,
  this run was not better on that count, but one run is not a measurement.

## Not verified (and why)

- Whether 0.03 m or any of the Nav2 changes reach a goal within 5 cm: no Set P goal was run, because the frame
  moves.
- The coast distance at the 0.10 and 0.15 m/s caps: not measured.
- AMCL drift at rest, and AMCL with a cleaner map or a lighter CPU load: not measured.
- Whether sliding versus lifting the robot changes how long the pose takes to settle: not tested.
- Whether switching off Cartographer's global constraints stops the jumps and what local drift remains while
  driving: not tried.
- Set E as a gate: the owner decided it is reported only, not a gate.

## Plan corrections

- Section 3's preflight expects 0.05; it is 0.03 since `b553bc2`. The robot checkout is no longer at `690c45c`,
  and the install is symlinked, so no `colcon build` is needed to deploy config or scripts.
- Section 4 and the run order assume `nav_course.py` works in SLAM mode (step 4). It did not until `2ef1269`.
- Section 5 assumes the pose can be trusted once Nav2 reports success. On this stack it cannot: after a move the
  pose needs minutes to settle, and a frame jump of 0.41 m occurred at rest. Settle checks (two reads 30 s apart
  agreeing within 1 cm) were used for the recordings and cost 1 to 4 minutes per cross.

## Next experiment (proposed, not started)

A rigid-frame SLAM variant: stop Cartographer applying global loop-closure constraints for goal runs, so that
`map -> odom` stays fixed to the start pose and only local scan matching moves the pose. The expected effect is no
jumps; the unknown is local drift over a 1 m path with no wheel encoders. The plan deliberately left
`slam_real.lua` alone because it carried run 1 (25.5 m, 21 goals); this would be a separate variant file, with
its own measurements: rest drift, a repeat of cross 10 and Set P, and a drift-per-metre figure over repeats
(CLAUDE.md section 1: n, mean, spread). The exact option names must be read against the installed Cartographer
version before editing. Alternatives: Cartographer pure localisation on a saved pose graph, or a cleaner map for
AMCL.
