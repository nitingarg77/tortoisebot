# Working standards for this repository

This is a real robot with motors, a lidar and no wheel encoders. It drives into
things. Most of the rules below exist because something here was asserted
without being measured, and the robot paid for it.

Read `MODULE.md` for what the code does and where the traps are. This file is
about *how* to change it.

---

## 1. Evidence before assertion

**State only what you have measured, and say which measurement.** A claim in a
commit message, a comment or a message to the user is a claim about the robot,
and the robot is checkable.

The failures this comes from, all on 2026-09-28:

- Exploration stalled. CPU load was high, so load was blamed, `176a4d1` was
  committed to reduce it, and it changed nothing. The cause was a second
  costmap observation source. The load was real and irrelevant.
- The left-motor trim was reported as working after one run came out six times
  straighter than the previous one. The node read the parameter once at
  startup, so the value had never reached the motors at all. See `bc38cd7`.
- A blind arc in the lidar was attributed to something mounted on the robot,
  "proved" by a rotation test. The robot had not rotated. The evidence was two
  identical readings, which proves only that nothing moved.

Practical rules:

- **One run is not a measurement.** This robot's drift-per-metre varied 8-fold
  between consecutive identical runs. Report `n`, the mean and the spread.
  `tortoisebot_firmware/scripts/calibrate_drive.py` does this.
- **A number that did not change is not evidence that something is fixed.** It
  is equally evidence that nothing happened. Distinguish the two before
  reporting.
- **Check the value landed.** A parameter set at runtime, a file deployed to
  the robot, a rebuilt package: confirm it is live before measuring against it.
  `ros2 param get` after `ros2 param set`; `grep` the installed file, not the
  source.
- When a hypothesis is wrong, say so plainly and record it. `MODULE.md` keeps
  the withdrawn TF-remap theory for this reason.

## 2. Anything that commands robot motion

Every tool, test or script that publishes to `cmd_vel` must have all three:

1. **A clearance guard** in the direction of travel, checked every loop, that
   stops the robot. A deadband test without one sent the robot 6.8 m down a
   corridor when 3 m was asked for, stopping 0.68 m from a wall by luck.
2. **A did-it-actually-move check.** There are no encoders. Commands fail
   silently: `differential.py` holds the last duty cycle and reports nothing.
   Compare the lidar before and after across several bearings, or read the IMU.
   `jog.py`-style single-bearing checks are not enough — a cone pointing
   through a doorway once reported *negative* progress and the tool concluded
   "motors not responding" while the robot was driving.
3. **A stop on every exit path**, including exceptions and timeouts. Publish
   zeros in a `finally`, and tolerate the context already being torn down.

Do not use a lidar cone as an odometer when the robot may be turning: the cone
sweeps onto other surfaces and the distance is meaningless. Use it only for
straight-line motion, take a median rather than a single beam, and cross-check
against a second bearing.

## 3. Testing without the robot

`MODULE.md` lists "no CI and no tests" as a known gap. Do not widen it.

- **Pure logic goes in a module that imports no ROS**, and the node imports
  that module. `tortoisebot_navigation/floor_geometry.py` and
  `tortoisebot_firmware/motor_math.py` are the pattern; `floor_scan.py` and
  `differential.py` are thin shells over them.

  The rule exists because putting the function in the node file is not enough.
  Both of those functions started inside nodes that import `rclpy` and
  `RPi.GPIO` at module scope, so neither could be reached without a robot, and
  both were "verified" against logic pasted into a throwaway script. A copy
  proves the algorithm; it says nothing about the code that ships, and the two
  drift apart silently. The tests now import the module the robot runs.

- **Pin what you replace.** When a rewrite is meant to preserve behaviour, keep
  the old implementation in the test file and assert they agree:
  `test_motor_math.old_duty` and `test_floor_geometry.loop_reference` do this,
  the second across 40 random frames covering every run length and horizon.
- **New nodes take their inputs through an interface, not a hard-wired
  subscription.** A node that can be handed a synthetic `LaserScan` can be
  tested; one that can only subscribe to `/scan` can only be tested on the
  floor, at night, with the robot wedged in a doorway.
- **Provide a mock for every hardware and third-party boundary**: lidar, IMU,
  GPIO, camera, and any model API. `cam_ws/src/product_counter` already does
  this — its 19 counting tests run on synthetic frames with no camera.
- A bug found on hardware that could have been found at a desk should come with
  the test that would have found it.

## 4. Concurrency and determinism

- **Assume callbacks are starved.** A Nav2 node's transform listener stopped
  being served and its buffer froze at one timestamp for fifteen minutes while
  `/tf` on the wire stayed at 66 Hz. Nothing logged an error. Long or blocking
  work does not belong in a callback.
- **Check for silent staleness, not just for exceptions.** `MODULE.md` records
  the signature: a *frozen* `Transform time` is a hang; a *moving* one is
  ordinary lag. Nav2 Humble's `isGoalReached()` discards the return value of
  `transformPose`, so a failed lookup becomes a goal at the origin and a false
  success. Assume upstream ignores errors and check the state yourself.
- **One publisher per topic, or know why not.** `/cmd_vel` had six publishers
  — `behavior_server`, `velocity_smoother`, `pose_watchdog` — and anything
  written directly to it was overwritten with zeros within milliseconds while
  the tool reported success. Inject at `/cmd_vel_nav` when Nav2 is up.
- **Parameters that exist to be tuned must apply at runtime.** Reading a value
  once in `__init__` makes `ros2 param set` silently do nothing, which looks
  exactly like a change that had no effect.

## 5. Resources

- **Drive outputs to a safe state on every exit**, and prefer that to library
  cleanup. `cam_ws`'s `gpio_io.close()` deliberately does not call
  `GPIO.cleanup()`, because cleanup left the pin floating high.
- RPi.GPIO's PWM is a software thread in-process: exiting mid-cycle can latch an
  enable pin high, which is full speed. Zero the duty cycle, then stop.
- The Pi 4 has no headroom. Cartographer alone takes 1.6 cores. Before adding a
  node, measure what it costs; a camera left free-running at 30 fps was
  publishing 900 kB frames to a consumer that looked at 5 a second.

## 6. When a design comes first

- **Bug fix in existing code:** go straight to the smallest diff that fixes it,
  with the measurement that justifies it in the commit message. Removing the
  duplicated voxel layer was four lines and took the stack from 0/8 goals to
  2/0. A design document would have helped nobody.
- **New node or subsystem:** design first — data flow and concurrency, then
  interface contracts with mocks, then implementation with tests. This is where
  the cost of getting it wrong is a rewrite rather than a revert.

## 7. Calibration belongs to this chassis

Measured values are properties of *these* motors, *this* camera and *this*
building, not of the design. Keep them in config with the measurement that
produced them and the tool that reproduces it — `left_trim: 1.28` in
`autobringup.launch.py` is the pattern. Anyone porting to another chassis needs
to know which numbers to re-measure and how.

## 8. Commits

State what was measured and what is still unverified. Name the alternatives
ruled out and how. If something is a hypothesis, label it: `5ca83df` says
"Written as indicated, not proven" because the A/B behind it was one run each.
