# Needle 3 on TortoiseBot — spike results

**Verdict: 21-22/32 (66-69%) on the base model. Genuinely promising, not yet
deployable. The blocker is refusing things it cannot do (3/7), not accuracy.**

Measured 2026-09-23, `cactus-needle` 3.0.4, Needle 3 base weights, CPU only.
Same harness as the cobot-cell spike (separate repo: `cobot_ws/spike_needle_nl`), so the
two are directly comparable.

## Headline

| | TortoiseBot | Cobot cell |
|---|---|---|
| Total | **22/32 (69%)** | 13/35 (37%) |
| `read` | 6/8 (75%) | 4/12 (33%) |
| `tune` | 2/2 (100%) | 6/8 (75%) |
| `act` | 8/12 (67%) | 1/5 (20%) |
| `multi` — two calls in one utterance | 2/3 (67%) | 0/3 (0%) |
| `oos` — must produce **no call** | 3/7 (43%) | 2/7 (29%) |

Across repeated runs of the same code, TortoiseBot scored 21/32 twice and 22/32
three times, and the cobot cell 13/35 three times and 14/35 twice — **±1 case**.
Not fully deterministic across processes, but stable enough that the category
pattern is real; the wild pre-reset swings are gone.

TortoiseBot beats the cobot cell by 32 points on the same model and the same
harness, which is the result the spike was designed to test. The two reasons were
predicted in advance and both hold up: this is Pi-class hardware where an 8–29 MB
model is the point rather than a handicap, and "go to the kitchen" / "turn left" /
"stop" sit inside the mobile-action distribution Needle was trained on, where
"suspicion threshold" and "audit fraction" did not.

## The harness bug that was costing 32 points

**`Needle.complete()` is not stateless, and nothing in the README says so.** The
engine keeps conversation state across calls, and a prior unrelated query changes
the answer to the next one — deterministically, not as noise:

```
cold process, "head over to the living room"
  -> go_to_place(place='living_room')   conf 0.9968   (identical in 4/4 processes)

same query, after 4 unrelated queries in the same process
  -> get_nearest_obstacle()             conf 0.6025   (identical in 3/3 processes)
```

`Needle.reset()` exists (it calls `needle_reset()` in the engine) and clears it.
Calling it before each utterance took this eval from 34% to 66-69%, and the cobot
eval from a wobbling 14-29% to a stable 37-40%.

Everything measured before that fix was measuring accuracy under accumulated
unrelated history, which is not what a one-shot operator command is. It also
explains the "attractor tool" behaviour seen earlier — one tool appearing to
swallow unrelated queries — and the run-to-run variance, which was never variance
at all. `console.py` calls `reset()` per utterance for the same reason.

**If you take one thing from this spike: call `reset()` between independent
requests.** It is the difference between unusable and promising.

## What still fails, and whether it matters

Out-of-scope refusal, 3/7. The remaining four, at full confidence:

| Utterance | Decoded as | Confidence |
|---|---|---|
| "open the door" | `get_robot_pose()` | 1.00 |
| "what's the weather in Lagos" | `get_nearest_obstacle()` | 1.00 |
| "make me a new map of the house" | `go_home()` | 0.88 |
| "climb the stairs" | `go_home()` | 0.90 |

**Confidence still does not separate right from wrong** — 1.00 on "what's the
weather in Lagos". There is no threshold that keeps the good calls and drops
these, so the vendor's recommended confidence routing does not work here either.

The difference from the cobot cell is consequence, not correctness. There, a
misfire reached for `fire_nozzle` on a production line. Here the worst case is the
robot driving to its dock when you asked something silly, caught by a `[y/N]`
prompt. Same failure rate, survivable instead of not.

Also still wrong: `act-02` "drive to the bedroom" emits no call while "go to the
kitchen" and "head over to the living room" both work — the verb "drive" collides
with the `drive` tool. That is a tool-naming problem I would fix by renaming
`drive` to `nudge` before doing anything else. And one genuinely odd output:
"back up 2 metres" produced reasoning reading *"'back up 2 metres' means go to
charging dock"* at 0.9856 confidence, with no call emitted.

## Runtime, measured

| | |
|---|---|
| Model load | 3.4 s |
| Latency, median | 286 ms |
| Latency, worst | 8.3 s — unexplained multi-second outliers persist |
| Peak RAM | 98 MB |

98 MB resident on a Pi 4/5 is fine. Note it is not the advertised 8–29 MB; that
is the weight file, not the process.

## The bridge, run for real

`Nav2Backend` was exercised against a live stack on 2026-09-23: Ignition Fortress
headless (`nav2_test_world.sdf`), full Nav2 map-based navigation
(`explored_map.yaml`), AMCL localised at the origin, and the whole path
utterance -> Needle -> gate -> ROS 2 running in one process.

**Working against the real stack**, verified:

| | |
|---|---|
| `get_robot_pose` | reads live `/amcl_pose`, resolves nearest named place |
| `get_nearest_obstacle` | reads live `/scan`, 2.77 m at -141.9 deg |
| `get_navigation_status` | reads real goal status |
| `list_places` | static |
| `set_max_speed` | writes `FollowPath.max_vel_x` on the real `controller_server` |
| `go_to_place` | submits a real `navigate_to_pose` goal, accepted by the server |
| `stop` | cancels the real goal and zeroes `/cmd_vel` |

The gate behaved as designed end to end: `go_to_place` and `stop` both stopped at
a `[y/N]` prompt before reaching ROS; the reads ran without one. Navigation itself
was then validated to all five places -- see below.

### Three bugs that only a live stack could find

The mock backend cannot surface any of these, which is the argument for doing
this rather than trusting the eval:

1. **`/scan` QoS mismatch.** The bridge publishes BEST_EFFORT (set explicitly in
   `ignition_sim.launch.py`); a default RELIABLE subscriber is INCOMPATIBLE and
   receives nothing at all. rclpy only warns. Fixed with
   `qos_profile_sensor_data`.
2. **`/amcl_pose` durability.** AMCL publishes TRANSIENT_LOCAL and only when the
   estimate updates, so a VOLATILE late-joining subscriber gets nothing while the
   robot sits still -- indistinguishable from "localisation is down". Fixed by
   matching durability with depth 1.
3. **`navigation_status` reported terminal goals as in progress.** It only
   checked whether a goal handle existed, so a SUCCEEDED, CANCELLED or ABORTED
   goal still read as `navigating: True` -- backwards for the one question an
   operator asks after a failure. Now maps `GoalStatus` properly and clears the
   handle on a terminal state.

### Navigation, validated end to end

**5/5 named places reached, worst position error 0.15 m.**

| Place | Result | Error | Time |
|---|---|---|---|
| kitchen | arrived | 0.09 m | 18 s |
| living_room | arrived | 0.04 m | 16 s |
| bedroom | arrived | 0.13 m | 24 s |
| hallway | arrived | 0.15 m | 12 s |
| charging_dock | arrived | 0.07 m | 10 s |

Zero `Transform data too old` or extrapolation errors across the whole run.

### A correction: the "localisation bug" was mine, not the workspace's

An earlier version of this file reported that navigation never moved the robot,
blamed a stale `map -> odom`, and tied it to the known-unexplained bug documented
in `navigation_mapbased.launch.py`. **That was wrong, and the retraction matters
more than the original claim.**

The cause was two complete sim stacks running at once. An earlier `pkill -f
"ros2 launch"` killed the launch processes but not their children, so a second
`ign gazebo` came up alongside the first. Two simulators both publishing `/clock`
means simulated time moves backwards continuously, which produced

```
[tf2_buffer]: Detected jump back in time. Clearing TF buffer.
```

thousands of times. No TF listener can hold a buffer through that, so AMCL and
`controller_server` starved exactly as if localisation were broken.

How it was caught: `ros2 topic info` showed **Publisher count: 2** on `/odom`,
`/imu`, `/clock` and the relay topics. On a clean single stack the time-jump
warnings drop to zero and navigation works. Two things I had also flagged as
symptoms were normal all along:

- **A stationary AMCL does not republish `/amcl_pose`.** Nav2 AMCL only runs a
  filter update once the robot has moved past `update_min_d`/`update_min_a`, so a
  frozen pose topic on a parked robot means nothing is wrong.
- **`tf2_echo odom base_link` timing out was a tooling artifact.** A raw
  subscriber showed `odom -> base_link` arriving at 50 Hz the whole time.

The EKF is fine. `ekf_filter_node` does log `Failed to meet update rate!`, but the
sim uses `ekf_mapbased.yaml` at **50 Hz**, not the 100 Hz `ekf.yaml` I first
pointed at, and a 0.025 s overrun on a 0.02 s budget under a heavy headless sim is
cosmetic -- it publishes `/odometry/filtered` at a steady 50 Hz and its TF
throughout. **Nothing in this workspace needed fixing.**

Lesson for the next person: verify `Publisher count: 1` on `/clock` before
believing any TF or localisation symptom in this sim.

### The bug this did expose: my own waypoints

Three of the five original coordinates were unnavigable, measured against
`explored_map.pgm`:

| Place | Old clearance | |
|---|---|---|
| bedroom | **0.00 m** | inside an obstacle |
| kitchen | 0.25 m | under footprint + inflation |
| living_room | 0.35 m | marginal |

The kitchen goal was **accepted and then stalled 0.45 m short with no error from
Nav2** -- it reported `driving` indefinitely and logged nothing. That silent
failure is what first looked like a controller bug. `PLACES` now holds measured
coordinates with 1.6-2.0 m clearance, and the genuinely open area of this map is
much smaller than a house layout suggests: roughly a corridor x in [-0.6, 0.6],
y in [-0.6, 2.3].

## Why this one is worth fine-tuning

66-69% base is exactly the range where Cactus's claimed +18–36 point lift would land
this at 87–100%. That was not true of the cobot cell at 37%.

Two things to do in the training data, neither of which `needle platform generate`
will do for you:

1. **Negative examples.** ~150 utterances whose correct answer is an empty call
   list: doors, stairs, cameras, mapping, picking things up, off-topic. Abstention
   is the blocker, and generated data will be almost entirely positive.
2. **Paraphrase coverage on `go_to_place`.** "head over to", "meet me in", "drive
   to", "let's go to". This is the one command that matters and it is where the
   near-misses cluster.

Caveats unchanged from the other spike: the platform route keeps `confidence`
calibrated but sends your tool definitions and data to Cactus GPUs; the local LoRA
route runs free on this machine but leaves `confidence` as `None`, which disables
the gate's `read` tier.

## Scope

`Nav2Backend` is written against the real `navigate_to_pose`, `/cmd_vel`,
`/amcl_pose`, `/scan` and `controller_server` parameters, **but was never run
against a live stack** — Gazebo was not brought up. Treat it as the shape of the
bridge. Everything above was measured through `MockBackend`, which is the right
surface: tool-selection accuracy does not depend on whether the simulator is up.

`turn` and `drive` are open-loop timed `/cmd_vel` bursts with no obstacle check.
Fine for a demo nudge, wrong for anything real — Nav2's `Spin` and `BackUp`
behaviours are the correct implementation.

The named places in `tb_tools.PLACES` are plausible coordinates inside
`explored_map.yaml`, not surveyed ones. Re-measure them on the real robot with
`ros2 topic echo /amcl_pose`.

## Reproducing

```sh
python3 -m venv nv && ./nv/bin/pip install cactus-needle
export NEEDLE_TELEMETRY=0 DO_NOT_TRACK=1     # engine telemetry is ON by default
./nv/bin/python run_eval.py --verbose
./nv/bin/python console.py                   # mock, no ROS needed
./nv/bin/python console.py --live            # real robot; act tools MOVE IT
```
