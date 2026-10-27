# Architecture: camera-assisted localisation, Phase 1 offline spike
Status: design approved 2026-10-27 (owner said "defaults" to D1 to D8; Q5, Q7 to Q14 defaults stand as proposals)

## Owner decisions on D1 to D8 (2026-10-27)

All eight recommended defaults accepted: D1 ORB-SLAM3 plus the floor-plane module; D2 JPEG through the existing
stream, `jpeg_quality` the only camera knob; D3 the Cartographer config T-001 recommends; D4 merge `capture-recorder`
(`ea199de`); D5 `analysis/camera_spike/` with `COLCON_IGNORE`; D6 sessions S0 to S5; D7 bags in a path excluded from
the Drive backup (to be checked before the first bag exists, not yet checked); D8 this design lives on
`feature/precision-5cm`. The "Where the requirements are" paragraph below describes the file as written on another
branch and is now out of date.
Implements: docs/requirements/camera-assisted-localisation.md, Phase 1 only (AC1 to AC15). Phase 2 is out of
scope and nothing below designs it.

**Where the requirements are.** The approved requirements are commit `980a52a` on `feature/precision-5cm`
("A-005: owner approved scope of camera-assisted localisation (Phase 1)"). This working tree is on
`feature/T-001-rigid-frame-slam`, and that branch has no `docs/requirements/`. The design was written against the
text of that document as last saved in this session. BUILD_LOG.md:90-94 says `980a52a` changed only the status and
approval lines. Section and AC numbers below are from that document.

Short names: REQ = the requirements document; FINDINGS = `docs/test-reports/run4-localisation-findings.md`;
PLAN = `docs/test-reports/run4-precision-5cm-plan.md`; T-001 = `docs/tasks/T-001-rigid-frame-slam.md`;
CLAUDE = `CLAUDE.md`.

## Context

The spike answers one question with measurements: can this camera, recorded on this robot, track its motion well
enough to (C) independently measure where the robot stopped, and (B) measure drift while driving? A is in scope only
if T-001 shows that the jumps persist (owner decision, REQ "Owner decisions"). The spike records on the Pi and
analyses on the box afterwards. Nothing it adds runs in the robot's control loop, and nothing it adds commands
motion.

What exists today and is reused:

- **Camera.** `camera_ros` `camera_node`, 640x480 RGB888, capped at 10 fps by `FrameDurationLimits` [100000,
  100000], publishing `/camera_node/image_raw` (`tortoisebot_bringup/launch/autobringup.launch.py:132-155`). The
  only consumer is `floor_scan.py`, which keeps the latest frame and processes a 320-wide copy at 5 Hz
  (`tortoisebot_navigation/scripts/floor_scan.py:86-90`). libcamera reports the full-sensor binned mode
  `640x480-SGBRG10` (BUILD_LOG.md:432-434). `camera_ros` is not installed on the box (MODULE.md:289-291), so its
  behaviour cannot be read here; see Risks R1.
- **Recorder.** `tortoisebot_firmware/scripts/record_capture.sh` (`ea199de`, branch `capture-recorder`, **not
  merged** into this branch or `feature/precision-5cm`) already records `/camera_node/image_raw/compressed`, `/tf`,
  `/tf_static`, `/scan`, `/imu` and `/odom` to `~/captures/<name>` as rosbag2 sqlite3. It refuses to start if the
  compressed topic is silent for 30 s, refuses to overwrite a directory, closes the bag on SIGINT and prints
  `ros2 bag info` (BUILD_LOG.md:261-273). It has had one stationary test and one supervised capture. The
  compressed topic therefore exists on the robot. Its CPU cost has not been measured.
- **IMU.** `imu_node.py` publishes `/imu` at 50 Hz in `base_link` (`tortoisebot_imu/scripts/imu_node.py:39,45,90`).
- **Lidar.** `/scan` at 10 Hz, frame `lidar` (`ydlidar_ros2_driver/params/ydlidar.yaml:4,22`).
- **Cartographer pose.** `map -> odom -> base_link` on `/tf`; `provide_odom_frame = true`, `use_odometry = false`,
  `pose_publish_period_sec = 5e-3` (`tortoisebot_slam/config/slam_real.lua:33-37,52`). The rigid variant is
  `slam_real_rigid.lua` (T-001), selected by `rigid_frame:=True`.
- **Camera geometry.** In the URDF, `camera_link` is about (0.050, 0, 0.134) m from `base_link` with rpy 0 0 0
  (`tortoisebot_description/models/urdf/tortoisebotreal.xacro:172-181`). The measured 4.5 deg down tilt is only in
  the `floor_scan` parameter (`autobringup.launch.py:171-179`), so `/tf_static` has the camera rotation wrong by
  about 4.5 deg. `floor_scan` assumes a 53.5 x 41.4 deg FOV and a 0.134 m height "above floor"
  (`floor_scan.py:63-67`). Whether `base_link` is at floor height or axle height was not established, so the true
  camera height above the floor is an unknown (see "Calibration and time sync").
- **Set P.** 10 taped crosses (PLAN section 2), placed by hand "good to a few cm" (FINDINGS Table 3 note).
  `nav_course.py --record --pose-source tf` records the SLAM pose per cross but no timestamp
  (`tortoisebot_navigation/scripts/nav_course.py:167-196`).
- **Guarded drive tool.** `tortoisebot_firmware/scripts/calibrate_drive.py` drives guarded out-and-back legs,
  with distance from the lidar and heading from the BNO055 (docstring lines 1-15). It publishes `cmd_vel`
  directly, so it must run without Nav2 (CLAUDE s4, "one publisher per topic").

Pi budget (owner decision Q6): **at most 0.25 extra cores** while recording, with no change to the existing
camera consumer. The Pi is already oversubscribed: Cartographer about 1.6 to 1.7 cores, load average 9 to 18,
82.7 C with no fan (CLAUDE s5; FINDINGS Table 2 note; T-001 Follow-ups). A cost that fits within 0.25 cores can
still slow Cartographer, so the design measures that effect too (AC13).

## Components and data flow (who owns each shared buffer; threads, queues, locks)

```
PI (recording, owner present)                                BOX (offline, one heavy job at a time)
-----------------------------                                --------------------------------------
camera_node --raw rgb8 10 fps--> floor_scan (unchanged)
    |  (compressed only while subscribed)                    copy (rsync, by hand, after the session)
    +--/image_raw/compressed--+                                         |
imu_node --/imu 50 Hz---------+                                         v
ydlidar  --/scan 10 Hz--------+--> ros2 bag record --> ~/captures/<session>/   --->  stage 1 export
cartographer --/tf------------+     (record_capture.sh)        bag + manifest.yaml        |  (ROS adapter only here)
rsp --/tf_static (latched)----+                                + marks.csv                v
camera_node --/camera_info----+                                + resources.csv     dataset/<session>/
                                                                                          |
mark_cross (no ROS) ----> marks.csv                                         stage 2 calibrate + clock audit
res_sampler (no ROS) ---> resources.csv                                           |
                                                                       stage 3 VO run(s)   <-- heavy, job ticket
                                                                                  |
                                                                       stage 4 align + metrics (pure)
                                                                                  |
                                                                       stage 5 report (docs/test-reports/)
```

### On the Pi

| Process | New? | Owns | Reads | Writes | Threads, queues |
|---|---|---|---|---|---|
| `camera_node` | no | libcamera request buffers, the raw frame | sensor | `/camera_node/image_raw`, `.../compressed` (only while a subscriber exists, assumed, R1), `/camera_node/camera_info` | camera_ros internal; **configuration unchanged** |
| `floor_scan` | no | `self.latest` (its own frame copy) | raw image | `/floor_scan` | unchanged; this design adds nothing to its process |
| `ros2 bag record` (via `record_capture.sh`) | reused | its subscription queues and the rosbag2 write cache (Humble default `--max-cache-size`) | topics in "Recording design" | sqlite3 `.db3` files plus `metadata.yaml` on close | rosbag2 internal; one writer thread; flushed and closed on SIGINT |
| `mark_cross.py` | new, **no ROS** | an open append-only file | stdin (owner presses Enter) | `marks.csv`, one flushed and fsynced line per event | single thread |
| `res_sampler.py` | new, **no ROS** | an open append-only file | `/proc/<pid>/stat`, `/proc/loadavg`, `/proc/meminfo`, `vcgencmd measure_temp`, `vcgencmd get_throttled`, `df` | `resources.csv`, every 2 s | single thread |

No buffer is shared between a new component and an existing one. The new components are processes outside ROS.
They talk to ROS only through rosbag2 subscriptions (one more subscriber on topics that already exist) and the
filesystem. No lock is shared. The only coupling to an existing process is that `camera_node` does extra work
(JPEG encoding) because the compressed topic now has a subscriber. That cost lands in `camera_node`'s own process,
not in `floor_scan`. The cost-measurement session checks that `floor_scan`'s input and output rates do not change.

The recorder, sampler and marker are started with `setsid nohup` (as in the run recipe) so a dropped `ssh` does
not kill them. The marker needs a terminal; if the `ssh` drops, marks are lost from that point and the stationary
windows (below) still recover the cross order. Stop order: SIGINT to the recorder first, wait for
`metadata.yaml`, then stop the sampler, then the stack, following the run recipe (motor node first). Kill by PID
(shared process-kill rule), never `pkill` by name.

`res_sampler` runs at `nice 10`. The recorder runs at normal priority by default. Niced, it may drop messages under
load, and a drop would be invisible (see Failure handling). Whether to nice it is decided from the cost session
data, not now.

### On the box

The stages are separate, sequential processes. Each one reads the previous stage's output directory and writes a
new one. Nothing is edited in place, and the bag is never modified (read-only copy, checksummed on arrival). No
stage shares memory with another. Only stage 1 imports ROS (`rosbag2_py`, which is installed on the box under
`/opt/ros/humble`). Stages 2 to 5 are plain Python with numpy/cv2.

Stage 3 (VO) is the only heavy stage. Each run is one process, launched only under a `job` ticket after the owner
says yes. CPU systems run one at a time; anything that uses the GPU goes through `gpuq` (Q14). The VO systems are
multi-threaded and not deterministic, so **each pass is run 3 times per system**, and the spread across repeats is
reported next to the spread across passes. One run of the algorithm is not a measurement either.

## Recording design (Pi)

### Topics and rates

| Topic | Rate (expected) | Why | Size per message (estimate, to be measured in S0) |
|---|---|---|---|
| `/camera_node/image_raw/compressed` | 10 Hz (same frames as raw) | VO input | JPEG at 640x480: about 40 to 120 kB, depending on `jpeg_quality` and the scene (unmeasured) |
| `/camera_node/camera_info` | 10 Hz | what the driver thinks the intrinsics are; a frame-count proxy | under 1 kB |
| `/imu` | 50 Hz | gyro for the time-offset fit; yaw cross-check | under 1 kB |
| `/scan` | 10 Hz | stationary-window detection; golden-scan option; Cartographer is rebuilt from nothing else | about 5 kB |
| `/tf` | Cartographer plus rsp, measured 66 Hz on the wire (MODULE.md:313) | Cartographer pose (`map->odom`, `odom->base_link`) | under 1 kB |
| `/tf_static` | latched | extrinsics as configured (known to be missing the tilt) | under 1 kB |
| `/floor_scan`, `/scan_fused` | 5 Hz, 10 Hz | **measurement only**: shows the existing consumer is unchanged (AC13) | under 3 kB |
| `/rosout` | low | Cartographer "differs by" lines and recorder warnings, time-aligned | small |

Not recorded: `/camera_node/image_raw`. At 921,600 bytes x 10 fps it is 9.2 MB/s, 33 GB per hour, and an extra
900 kB copy per frame through the recorder (the cost CLAUDE s5 warns about). Also not recorded: costmaps,
`/trajectory_node_list` and `/constraint_list` (large and growing). Exception: one short raw clip (30 s) in session
S0, so JPEG loss can be judged against raw for calibration (D6).

**The camera configuration does not change.** Format, size, frame rate and `FrameDurationLimits` stay as they are;
changing any of them changes `floor_scan`'s input. The only camera-side knob the design may touch is the
compressed stream's `jpeg_quality` parameter, and only if (a) the installed `camera_ros` version has it, (b) the S0
cost is over budget, and (c) the owner agrees. It affects only the compressed output. Its live value is read with
`ros2 param get` before and after (CLAUDE s1).

### Bag storage

- rosbag2, **sqlite3** storage (the Humble default, and what `record_capture.sh` uses). The mcap plugin is not
  installed on the box and was not checked on the Pi.
- **No bag compression.** JPEG does not compress further, and zstd would spend Pi CPU for nothing.
- `--max-bag-size 1000000000` (1 GB splits). A crash or full disk then loses at most the open split. A split bag
  is still one bag directory per pass, which satisfies AC3's "single bag per pass".
- Location: `~/captures/spike_<YYYYMMDD>_<session>`, local SD card (Q12 default). Before each session, `df -h ~`
  must show at least twice the estimate below free, or the session does not start.
- Size estimate (to be replaced by the S0 measurement): 0.5 to 1.2 MB/s. A Set P pass with 1 to 4 min of settling
  per cross is about 20 to 45 min, so 0.6 to 3.2 GB per pass. All sessions S0 to S5 together: about 5 to 15 GB.
- A `manifest.yaml` per session, written by the operator script (Data model). It records the robot checkout's
  commit, the launch arguments, the Cartographer config basename **as read from the Cartographer log** (the value
  landed, not the value asked for), the `camera_ros` version, `jpeg_quality` as read live, free disk before and
  after, and the clock-sync state (`timedatectl show`).

### Pi cost budget and how it is measured (AC1, AC2, AC13)

Cost is measured, not estimated. Session S1 (cost) runs the same stack as the Set P sessions (drivers plus compute,
from the same launch scripts), with the robot at rest on cross 10. It runs three arms, each 5 min, in the order
A0, A1, B, A0, A1, B. Interleaving keeps the temperature drift from favouring one arm. That gives n = 2 blocks
per arm and about 150 samples per block at 2 s.

| Arm | What runs | Measures |
|---|---|---|
| A0 | stack plus `res_sampler` only | baseline CPU: per-process cores, load average, temperature, throttled flags |
| A1 | A0 plus a light bag of `/floor_scan`, `/scan_fused`, `/camera_node/camera_info` | baseline rates of the existing consumers, with a measuring cost that is also present in B |
| B | A0 plus the full spike bag | cost of the recording, and the consumer rates under it |

Reported, each with n, mean and spread (CLAUDE s1): cores (sum of utime and stime deltas per process), whole-Pi
busy cores, load average, temperature, throttled bits, and the per-process split (`camera_node`, the recorder,
Cartographer, Nav2 servers).

- **Extra cost** = B minus A0, whole-Pi busy cores. Owner limit: 0.25. `camera_node` minus its A0 value is
  reported separately, because that is the JPEG cost, which no recorder setting can remove.
- **Unchanged consumers**: the `/floor_scan`, `/scan_fused` and `camera_info` rates in B against A1, from bag
  receive times (not `ros2 topic hz`, which costs CPU itself). "Unchanged" means within the A1 block-to-block
  spread.
- **Control-loop side effects** (AC13): Cartographer's `map -> base_link` rest spread and step count (same method
  as FINDINGS Table 1), the count of Cartographer work-queue lines in the compute log, and Nav2 "missed its
  desired rate" lines, in B against A0.

With two blocks per arm, a change smaller than the block-to-block spread cannot be told apart from noise, and the
report will say so if that happens.

If B is over 0.25 cores, these fallbacks are tried in order, each measured the same way: (1) lower
`jpeg_quality` (needs the owner's agreement); (2) record at `nice`, accepting drops, which are counted; (3) record
the rest session raw instead of compressed (no encoding cost, about 5.5 GB for 10 min) and drop the camera from the
drive sessions. Lowering the camera frame rate is **not** an option, because it changes `floor_scan`.

### Marking cross arrivals (AC4)

Two independent sources are combined, so neither has to be precise on its own.

1. **`mark_cross.py` (new, no ROS, on the Pi, same terminal the owner already uses).** It walks the Set P order
   p1 to p10. For each cross the owner presses Enter when the chassis is placed (`arrive`) and when they pick it
   up or slide it off (`leave`). Typing `n <text>` adds a free note (for example "touched", "re-placed"). Each line
   holds `wall_ns` (`CLOCK_REALTIME`), `mono_ns` (`CLOCK_MONOTONIC`), the label, the event and the note.
   `CLOCK_REALTIME` on the Pi is the clock rosbag2 uses for its receive timestamps, because the recorder runs on
   the same machine with `use_sim_time` false. Marks and bag are therefore on one clock, and the only matching
   error is the owner's key-press lag (expected under about 2 s; unmeasured).
2. **Stationary windows from `/scan` (offline).** A window is a run of at least 5 s in which consecutive scans
   differ by less than a noise threshold, using a median absolute range difference over valid beams. The threshold
   is fitted on the rest session, not picked. Each `arrive`/`leave` pair is associated with the stationary window
   that overlaps it most. The cross pose of every trajectory (visual and Cartographer) is the **median over that
   window**, and its in-window spread is reported.

How time was matched and its error (AC4): the report gives, per cross, the mark times, the window bounds, the
offset between them and the in-window pose spread. The robot sits for 1 to 4 min per cross (FINDINGS), so key-press
lag affects only which window is chosen, not the pose taken from it. A cross with no matching window, or with two
candidate windows, is reported as unmatched and not dropped.

`nav_course.py --record` may still be run alongside, as in run 4. It is not needed, because the Cartographer pose
is in `/tf` in the bag, and it is not changed.

### Sessions (all `job` tickets, owner present, no motion except S5)

| Session | Robot | Stack | Content | Est. duration |
|---|---|---|---|---|
| S0 preflight | at rest | drivers only | versions (`camera_ros`, rosbag2 storage plugins, OS), `df`, RAM, `timedatectl`, live `jpeg_quality`, compressed topic rate and sizes, 30 s raw clip, ruler measurement of camera lens height above the floor and its forward offset from the axle | 20 min |
| S1 cost | at rest on p10 | full Set P stack | arms A0/A1/B x 2 (above) | 30 min |
| S2 calibration | at rest | drivers only | checkerboard held and moved by the owner in front of the camera, about 2 min; then the board flat on the floor at a measured position, 30 s | 15 min |
| S3 rest | untouched on p10 | full Set P stack | 10 min recording (owner decision Q4) | 12 min |
| S4 Set P | hand-placed on p1 to p10 | full Set P stack | at least 3 passes (Q4), one bag each, with marks | 3 x 20-45 min |
| S5 drives | guarded 1 m legs | drivers plus Cartographer, **no Nav2** | 5 guarded 1 m drives (Q4), one bag, marks at start and stop of each | 20 min |

T-001 runs first (owner decision Q2), and its robot measurements are taken **without** the spike recorder running.
Otherwise the recorder's CPU would contaminate T-001's rest-drift and settling figures, which depend on Pi load
(FINDINGS "The jump"). The spike sessions use the Cartographer config that T-001 recommends (`rigid_frame` true or
false, D3). The manifest records which config loaded.

S4 is a hand-placed pass, as in run 4: the robot is slid or lifted between crosses. That is not the motion of a
driven robot. The camera will see lifting (out-of-plane motion) and the owner's legs and hands (moving objects).
Recommendation: slide rather than lift, with the owner standing behind the robot. The report keeps hand-moved
passes (S4, answering C) apart from driven legs (S5, answering B).

## Offline analysis (box)

### Candidate visual systems

| System | Kind | Scale | Box cost | For this data | Licence | Verdict |
|---|---|---|---|---|---|---|
| **ORB-SLAM3** monocular, pinned at tag v1.0 | feature SLAM, CPU | none (monocular) | build is a heavy job; known Ubuntu 22.04 patches (C++ standard, Pangolin) | most benchmarked open system, so a failure points at the data rather than the tool; needs parallax to start; may lose tracking on plain floor; its own loop closure can make jumps | GPL-3 (fine for an offline tool; matters only for Phase 2) | **recommended, primary** |
| **Ground-plane homography VO** (new pure module) | floor features tracked frame to frame; homography decomposed with known height and tilt | **metric** (from camera height) | trivial CPU | made for this rig: tilted down, sees floor from 0.24 m (`floor_scan.py:17`), wood grain is visible texture (`autobringup.launch.py:180-187`); measures planar motion directly, which is B; drift only, no absolute fix, so weaker for C | ours | **recommended, secondary**; the only candidate that could plausibly fit on a Pi later |
| COLMAP or GLOMAP SfM on 2-3 fps keyframes | offline batch reconstruction | none | CPU heavy, hours; not installed | best accuracy available offline; an upper bound, not representative of live tracking | BSD | optional, only if ORB-SLAM3 is inconclusive |
| DPVO / DROID-SLAM | learned VO/SLAM, GPU | none | CUDA extension builds (risky on this toolkit, see the CUDA memory note); 4 to 11 GB GPU; competes with ComfyUI | most robust on low texture and blur | MIT / BSD | only if CPU systems lose tracking **and** the owner allows GPU (Q14) |
| stella_vslam | feature SLAM, CPU | none | build job | close to ORB-SLAM3 | BSD-2 | not for the spike; noted as the licence-friendly alternative if Phase 2 happens |
| VIO: ORB-SLAM3 mono-inertial, OpenVINS, Basalt | camera plus IMU | in principle | build job | BNO055 at 50 Hz, stamped by a Python timer, with no camera-IMU calibration; planar constant-speed motion is a known weak case for scale | various | rejected for the spike: the IMU data is not good enough to test fairly |
| RTAB-Map | RGB-D, stereo or lidar SLAM | n/a | n/a | no depth sensor; monocular is not its strength | BSD | rejected |
| Fiducial tags | absolute markers | metric | n/a | out of scope until after the spike (Q11 default) | n/a | not considered here |

**Recommendation: two systems on every bag.** ORB-SLAM3 monocular answers "can a standard visual SLAM track this
robot". The ground-plane VO answers "how far did it move", in metres, which is B. COLMAP and DPVO/DROID are
escalations, each its own job ticket. If the pinned ORB-SLAM3 version has a switch to turn its loop closing off, it
is run both with and without it; otherwise the report records that its loop closures can move its trajectory, as
Cartographer's do. Each ORB-SLAM3 sub-map (Atlas) after a tracking loss is reported as a loss with its time and
duration (AC7). Only the largest map is aligned, and the others are listed, not dropped.

### Calibration and time sync (open questions 8 and 9): how the spike establishes them

| Unknown | How it is established | Accept / reject signal |
|---|---|---|
| Intrinsics and distortion | S2 checkerboard clip; `cv2.calibrateCamera` on about 30 to 60 well-spread frames; prior fx about 320 / tan(26.75 deg), about 635 px, from `floor_scan`'s 53.5 deg HFOV | RMS reprojection error and the spread of fx over 3 frame subsets (n, mean, spread); fx far from the prior is investigated, not accepted |
| JPEG effect on calibration | the same board poses in the S0 raw clip and in the compressed stream | the difference in the calibrated parameters |
| Camera to `base_link`: pitch and roll | floor plane fitted to ground-plane VO features and ORB-SLAM3 map points; checked against the S2 floor board | compared with the 4.5 deg from `fit_camera_tilt.py` |
| Camera to `base_link`: yaw | heading of the S5 straight legs in camera frame against the lidar/IMU heading | spread over 5 legs |
| Camera height above the floor | ruler in S0, then the plane fit | gives the metric scale for the ground-plane VO; a disagreement is reported, not averaged |
| Header stamp base and latency per topic | **clock audit**: distribution of (bag receive time minus `header.stamp`) for each topic | a constant offset of seconds to years means a different clock base (for example a monotonic sensor timestamp); the spread is the latency jitter |
| Camera to IMU offset | cross-correlation of gyro z with visual yaw rate (ORB-SLAM3 and ground-plane VO) over S5 and the turns in S4 | peak lag and the peak's width; if there is no clear peak, the result is "not established" |
| Camera to Cartographer offset | the same with Cartographer's yaw rate from `/tf` | as above |
| FOV and frame rate enough (Q9) | tracked-feature counts and tracking-state log per frame; motion blur from image sharpness per frame | reported per pass; there is no pass/fail without the owner's thresholds |

Things that cannot be established from this data: rolling-shutter parameters (the OV5647 is rolling shutter; its
effect shows only as worse error during turns), auto-exposure and white-balance changes (logged as frame
brightness), and the BNO055's internal latency.

### Scale and alignment (AC8): reported before and after

Every visual trajectory is reported under each of these alignments. Error is shown next to the unaligned figure, so
a fit cannot hide a poor result.

| Label | Rotation and translation | Scale | Uses truth? | Purpose |
|---|---|---|---|---|
| U0 raw | none | none | no | shape only; shown as a plot and a step-at-rest figure |
| U1 start-anchored | SE(2) from the first stationary window on p10 to (0, 0, 0) | camera height (independent of lidar and tape) | p10 only | **the honest number for C** |
| F-carto | Sim(2) Umeyama fit to Cartographer over the whole pass | fitted | Cartographer | agreement with the lidar; not independent of Cartographer |
| F-tape-all | Sim(2) fit to all matched crosses | fitted | all crosses | best case, labelled optimistic |
| F-tape-LOO | Sim(2) fit leaving each cross out in turn; that cross's error is taken from the fit without it | fitted | all but the cross scored | the fair version of F-tape-all |

The ground-plane VO is metric by construction, so its U1 needs no scale fit. ORB-SLAM3's U1 takes its scale from
the camera height above the fitted floor plane. If no floor plane fits (too few floor points), U1 is "not
available" and the report says so.

### Comparisons (AC6, AC7, AC10)

- **Per cross, against the tape.** The error is the 2D distance between the visual pose median over the window and
  the planned cross position. The same is done for the Cartographer pose. A per-pass table (cross, mark times,
  window, visual error under each alignment, Cartographer error) is followed by n, mean, spread and worst per pass,
  then the same across passes, then the same across the 3 algorithm repeats.
- **Against Cartographer over time.** Over the pass, the difference between the visual trajectory (F-carto) and
  `map -> base_link`, with `map -> odom` plotted separately. A jump in `map -> odom` with no matching visual motion
  is the A-type event. If none occurs in the recording, the report says so (AC10 A).
- **C.** The per-cross error at stop under U1 and F-tape-LOO, with the tape floor stated (Q5: "a few cm"). The
  report gives an explicit statement on whether the error is small enough to tell 5 cm from 10 cm given that floor.
- **B.** For the 5 S5 legs: the end-point error per metre against the tape, from ground-plane VO, ORB-SLAM3 (U1)
  and Cartographer, with n, mean and spread, compared with T-001 criterion 5's figure.
- **Rest (S3).** Maximum step and spread over 10 min for each visual system and for Cartographer.
- **Tracking loss.** Every interval with time, place (nearest cross) and duration, per system per repeat.
- **Thresholds.** Blank (owner decision Q3). The report prints the numbers and "not judged". The AC9 thresholds
  file exists, empty, so that adding numbers later turns judgement on without a code change.

The report ends as AC11 requires: result line, "Not verified", and one recommendation (tracks well; does not;
inconclusive, and what to measure next). It does not claim 5 cm.

## Data model

| Artifact | Where | Fields |
|---|---|---|
| `manifest.yaml` | Pi `~/captures/<session>/`, copied with the bag | session id and type (S0 to S5), pass number, start and end `wall_ns`, robot commit, launch args, Cartographer config basename from the log, `camera_ros` version, `jpeg_quality` live value, `df` before and after, `timedatectl` sync state, operator notes |
| `marks.csv` | same | `wall_ns, mono_ns, label, event{arrive,leave,start,stop,note}, text` |
| `resources.csv` | same | `wall_ns, mono_ns, arm, loadavg1, temp_c, throttled_hex, mem_avail_kb, disk_free_kb, then per-process utime and stime ticks for named PIDs` |
| bag | same | rosbag2 sqlite3, topics as above |
| `dataset/<session>/` | box | `images/<recv_ns>.jpg` (the recorded JPEG bytes, not re-encoded); `images.csv (recv_ns, header_ns, file)`; `imu.csv`; `scans.npz`; `carto.csv (t, map_base x y yaw, map_odom x y yaw)`; `tape.yaml` (Set P plan plus any ruler offsets); `clock_audit.csv` |
| `calib/` | box | `camera.yaml` (K, distortion, RMS, n frames, subset spread), `extrinsics.yaml` (pitch, roll, yaw, height, each with its source and spread) |
| `vo/<system>/<repeat>/` | box | TUM trajectory `t x y z qx qy qz qw`; `tracking.csv (t, state, n_features)`; exact command line and version |
| `results/<session>/` | box | per-cross CSV, alignment parameters per label, the figures; then the report in `docs/test-reports/` |

Bags and datasets are camera images of the building and of people in it. Data location: see Security.

## Interfaces and contracts (each hardware/network/API boundary + its fake)

Pure-logic modules import no ROS (CLAUDE s3). They take plain arrays and dataclasses, not messages. Only the
adapters named "real" touch ROS, `/proc`, subprocesses or the camera.

| Boundary | Interface (contract) | Real | Fake for desk tests |
|---|---|---|---|
| Bag reading | `BagSource`: `frames() -> iter[(recv_ns, header_ns, jpeg_bytes)]`, `imu() -> iter[ImuSample]`, `scans() -> iter[Scan]`, `transforms() -> iter[Tf]`, `static_transforms()`; time-ordered; never raises on a corrupt message, yields a counted `Bad` record instead | `rosbag2_py` adapter (the only module importing ROS; installed on the box) | `SyntheticBag`: built from a scripted ground-truth path, with injectable gaps, duplicates, NaN ranges, a header-stamp base offset and a camera time offset |
| Camera images | as above; decode via `cv2.imdecode` | recorded JPEG | rendered frames: a random texture on a floor plane warped through a known K, tilt and height (`cv2.warpPerspective`), the pattern `cam_ws/src/product_counter` uses for its 19 synthetic-frame tests |
| Visual odometry | `VisualOdometry.run(dataset_dir, calib, out_dir) -> (Trajectory, TrackingLog)`; trajectory times in the bag's receive clock; lost frames are absent, not interpolated | ORB-SLAM3 subprocess adapter (writes a TUM file); `ground_plane_vo` module | `ScriptedVO`: ground truth with scale s, rotation offset, random-walk drift, and lost intervals, so that alignment and metrics are tested end to end |
| Calibration | `calibrate(frames, board) -> CameraModel` | `cv2.findChessboardCorners` / `calibrateCamera` | boards rendered through a known K and distortion |
| Resource sampling (Pi) | `ProcSource.read() -> Snapshot` (text of `/proc/<pid>/stat`, `/proc/loadavg`, `vcgencmd` output); `cores_between(a, b, clk_tck)` is pure | file reads plus `vcgencmd` subprocess (missing gives an empty field, never 0) | canned `/proc` text, including a PID that exited between samples |
| Marks (Pi) | `next_event(state, line) -> (state, record)` is pure; the clock is injected | stdin, `time.time_ns`, `time.monotonic_ns` | `StringIO` input with a fixed clock |
| Recording | the CLI contract of `record_capture.sh`: topic list, out dir, duration; refuses if silent for 30 s or the dir exists; SIGINT closes the bag | `ros2 bag record` | none at the desk; a `--dry-run` that prints the exact command is the reviewable part (R7) |

Pure modules and the behaviour each must have (each tested on fakes):

- `spike_time`: clock audit (offset, base-mismatch flag, jitter, gaps over 2 periods), stationary windows from
  scans, mark-to-window association (unmatched and ambiguous are explicit results).
- `spike_align`: SE(2) and Sim(2) Umeyama, start anchoring, leave-one-out, floor-plane fit and height-derived scale,
  3D-to-planar projection; refuses with a typed error when fewer than 3 valid pairs exist.
- `spike_metrics`: per-cross error, n/mean/spread/worst, step at rest, tracking-loss intervals, drift per metre.
- `time_offset`: cross-correlation lag between two rate signals on a common grid, with the peak width as confidence.
- `ground_plane_vo`: feature track, then homography, then the planar motion decomposed with known K, tilt and
  height.
- `spike_report`: markdown tables; prints "not judged" when the thresholds are blank.

Where the code lives is a tech-lead decision (D5). Recommendation: a directory outside the ROS packages, for
example `analysis/camera_spike/` with a `COLCON_IGNORE` file, so the robot never builds or installs it. Its `test/`
is added to `run_tests.sh`; a module not in that list rots (CLAUDE s3). The two Pi scripts (`mark_cross.py`,
`res_sampler.py`) sit beside `record_capture.sh` in `tortoisebot_firmware/scripts/`, with their pure parts in a
module that imports no ROS.

## Failure handling (dropped data, timeouts, NaN, disconnect, shutdown)

| Failure | Detection | Handling |
|---|---|---|
| Camera frames dropped (driver or recorder) | gaps over 2 frame periods in receive times; count against rate x duration | listed per pass with time and length; VO sees a gap, not a fabricated frame |
| Recorder drops under load (Humble rosbag2 does not report drops, as far as I know; unverified) | gaps in `/imu` (regular 50 Hz) and `/scan` | counted and reported per arm in S1; a pass whose IMU gaps exceed the S1 baseline is flagged |
| Compressed topic silent at start | `record_capture.sh` 30 s guard | refuses to start; the operator fixes it and starts again; nothing is half-recorded |
| Compressed topic stops mid-run (camera disconnect) | gap over 2 s | the pass is flagged; it is kept and reported, not deleted |
| Disk full | `df` pre-check at twice the estimate; 1 GB splits | at most the open split is lost; the manifest records free space after |
| `ssh` drop | the operator sees it | the recorder and sampler survive under `setsid`; marks stop; stationary windows recover the cross order; the report states which marks are missing |
| Pi crash or power loss mid-bag | no `metadata.yaml` | try `ros2 bag reindex` (check it exists in the installed Humble rosbag2 first); else read the `.db3` directly; report what was recovered |
| Clock base mismatch or NTP step | clock audit; `wall_ns` against `mono_ns` in `marks.csv` and `resources.csv` | correct with a constant offset only if the audit shows a constant; otherwise those topics are excluded from timing claims |
| JPEG decode failure | `cv2.imdecode` returns None | counted, skipped, reported |
| NaN or inf in `/scan` | the driver sets invalid ranges to inf (`invalid_range_is_inf`, MODULE.md:349) | excluded from stillness and golden-scan statistics; NaN beams counted |
| NaN in IMU | `imu_node`'s sample checks reject them (`tortoisebot_imu/test/test_sample_checks.py`) | gaps as above |
| VO lost, new ORB-SLAM3 map, NaN pose | tracking log; non-finite rows | an interval marked lost, never interpolated; the largest map is aligned, the others are listed |
| Fewer than 3 matched crosses | `spike_align` typed error | that alignment is "not available" for the pass; the report says why |
| Stationary window not found or ambiguous | `spike_time` result | the cross is "unmatched", with its marks shown |
| Box VO job dies (oomd, ComfyUI contention) | non-zero exit; partial TUM file | the run is discarded and re-queued; never merged with a partial run |
| Shutdown on the Pi | n/a | SIGINT the recorder first and wait for `metadata.yaml`; then the sampler; then the stack per the run recipe (motor node first); kill by PID only |

## Security

- **Privacy.** The bags contain images of the building and of whoever is in it, including screens. They stay on
  the Pi and the box: no cloud tools, no upload. `/workspace` may be inside the Google Drive backup set (rclone,
  daily 03:00). Whether `robot_logs/` is included was not checked. Default: put spike bags in a path excluded from
  the backup, or have the owner accept that they are backed up (D7).
- No new network service, port, credential or listener. The recorder subscribes on the Pi's local DDS graph. The
  copy to the box is an `rsync` over the existing `ssh tb` access, started by hand.
- The bags are copied, not moved. Nothing on the Pi is deleted until the box copy is listed and its sizes match
  (PLAN section 3).

## Safety (AC12, AC13, AC14)

- **No `cmd_vel` code.** The spike adds a recorder wrapper, a marker and a sampler; none of them publishes anything
  on ROS. The analysis is offline.
- **The only motion** is S5's five 1 m drives, using the guarded tool that T-001 criterion 5 uses
  (`calibrate_drive.py` is the candidate). Whether it meets all four CLAUDE s2 requirements (clearance guard, a
  did-it-move check over several bearings, a stop on every exit, a leash sized to 1 m) is for T-001 to confirm; this
  design does not certify it. It publishes `cmd_vel` directly, so it runs with **Nav2 not launched** (CLAUDE s4). If
  it does not meet s2, S5 waits; the spike does not write a replacement.
- **Owner present** at the robot for every session (CLAUDE "Hardware boundary"). Every session is a `job` ticket.
- **Control loop unchanged** (AC13): same launch scripts and arguments; the Cartographer config is read from the log;
  S1 measures side effects on Cartographer and Nav2.
- **Box:** ORB-SLAM3's build and each VO run is a heavy job, asked for first, one at a time. Anything on the GPU goes
  through `gpuq`, and the owner decides about GPU use (Q14). COLMAP and DPVO/DROID are separate job tickets.
- **AC15** holds by construction (the recording is local and nothing crosses Wi-Fi). The report shows it by noting
  that the bag is copied only after the session ends, with the S1 rates measured before any copy.

## Test strategy (what runs without hardware; what is a `job`)

Runs at the desk, in `./run_tests.sh`, with no ROS, robot, GPU or network:

- `spike_align`: recovers a known Sim(2) from synthetic pairs to 1e-9. Leave-one-out on noisy data gives a larger
  error than the all-in fit. **Gate seen to fail:** let LOO include the held-out cross and the test must fail.
- `spike_time`: finds the injected stationary windows; associates marks; flags an injected header-stamp base offset
  of about 1.7e9 s; lists injected gaps.
- `time_offset`: recovers an injected 40 ms lag between synthetic gyro and visual yaw rate to within one sample.
- `spike_metrics`: n, mean, spread and worst on hand-computed cases; lost intervals are not interpolated
  (gate: interpolate them and the test fails).
- `ground_plane_vo`: on rendered floor frames, recovers a 1.00 m straight move and a 90 deg turn within a tolerance
  set by the implementer and recorded; fails if the height or tilt is wrong by a known amount.
- `calibrate`: recovers K from rendered boards to within 1 percent.
- `res_sampler` parsing: cores between two canned `/proc` samples; an exited PID gives an empty field.
- `mark_cross` state machine: Set P order, notes, and an extra Enter at the end that does not invent p11.
- `spike_report`: prints "not judged" with blank thresholds and pass/fail once thresholds are filled (gate seen
  both ways).
- An end-to-end desk run: `SyntheticBag` through `ScriptedVO`, alignment, metrics and report, with known answers.

Not covered by any desk test, and therefore measured in a `job`: the `camera_ros` behaviour, the Pi cost, real JPEG
quality, real VO tracking, whether the floor texture is enough, the BNO055 timing, and `record_capture.sh` itself
(beyond reading its `--dry-run`).

`job` tickets: S0 to S5 on the robot; the ORB-SLAM3 build; the VO runs; COLMAP or GPU systems only by escalation.

## Risks

| ID | Risk | Effect | Mitigation |
|---|---|---|---|
| R1 | `camera_ros` behaviour unknown at the desk (not installed on the box): whether it encodes JPEG only when subscribed, in which thread, at what quality, and whether `header.stamp` is a sensor timestamp or `now()` | the cost estimate and timing could be wrong | S0 reads the installed version's source and measures; S1 measures cost; the clock audit measures stamps |
| R2 | Pi already oversubscribed; the extra load slows Cartographer even within 0.25 cores | settling and jumps get worse during recording, which would bias the Cartographer comparison against Cartographer | S1 measures Cartographer side effects; T-001 measured without the recorder |
| R3 | Hand-moved passes are unlike driving: lifting, legs in view | VO loses track or errors more than it would when driving | slide, operator behind; S5 provides driven data; the two are reported apart |
| R4 | Little parallax and mostly floor in view; pure rotations in place | ORB-SLAM3 fails to start or loses track | ground-plane VO is designed for exactly this view; tracking loss is reported, not hidden |
| R5 | Rolling shutter and motion blur under indoor exposure | worse error during turns | sharpness logged per frame; error split by turning and straight segments |
| R6 | Tape reference good only to "a few cm" (Q5) | C may be undecidable at 5 cm | stated as the floor; a better reference is a Q5 decision |
| R7 | `record_capture.sh` is on an unmerged branch with little field use | the tool the plan depends on may differ from the BUILD_LOG description | merging it is a ticket (D4); `--dry-run` reviewed; S0 is its first deliberate test |
| R8 | ORB-SLAM3 build on 22.04, and CUDA builds for DPVO/DROID (box toolkit quirks) | days lost to builds | ORB-SLAM3 only by default; GPU systems only on escalation |
| R9 | The camera-height scale depends on knowing `base_link` height; URDF and `floor_scan` say 0.134 m, possibly relative to the axle | U1 scale wrong by up to about 25 percent | S0 ruler measurement; the plane fit cross-checks it; disagreement is reported |
| R10 | Large bags fill the SD card or the box backup | lost session, or images in the cloud backup | `df` gate, splits, D7 |
| R11 | Non-deterministic VO | a single lucky run is reported | 3 repeats per pass per system |

## Alternatives considered

- **Record raw images.** 9.2 MB/s, 33 GB/h, and a 900 kB copy per frame through the recorder on a Pi with no
  headroom (CLAUDE s5). Rejected except for the 30 s S0 clip and fallback (3).
- **Lower the camera frame rate or size for recording.** That changes `floor_scan`'s input, which the owner ruled
  out. Rejected.
- **A throttle or republish node on the Pi** (for example `topic_tools throttle`). It adds another full deserialise
  of raw 900 kB frames. Rejected.
- **Record on the box over Wi-Fi.** It needs Fast DDS unicast peers, adds cross-machine traffic to the graph the
  control loop uses, and the Wi-Fi split already made Cartographer run away on 2026-09-28 (owner's run-recipe
  note). AC15 wants a local recording. Rejected.
- **Record during T-001's session.** One trip, but the recorder's load would contaminate T-001's measurements, and
  the owner chose T-001 first. Rejected.
- **Add timestamps to `nav_course.py --record`.** It changes an existing tool on another branch, and records only
  one moment per cross. Rejected in favour of a separate script plus stationary windows.
- **Use Cartographer's pose as the only scale source.** Then the visual result is not independent of the lidar, and
  C needs independence. Kept only as the F-carto label.
- **VIO first.** The IMU is not good enough to test VIO fairly (rate, stamping, no camera-IMU calibration). Rejected
  for the spike.
- **Run VO on the Pi.** Out of budget and out of scope (Phase 2). Rejected.
- **Learned SLAM (DPVO/DROID) as primary.** GPU contention, build risk, and results harder to attribute. Kept as an
  escalation.
- **Fiducial tags.** Out of scope until after the spike (Q11 default).

## ADRs

The caller asked for this one file only, so no ADR files are written. Once the design is approved, these become
ADRs:

1. Compressed JPEG recording through the existing `camera_ros` stream, with no camera reconfiguration (Recording
   design).
2. Two VO systems on every bag, ORB-SLAM3 monocular plus a ground-plane homography VO, with COLMAP and learned
   systems as escalations.
3. Cross matching by marks plus lidar stationary windows, with the pose taken as a median over the window.
4. Scale and alignment reported under five labels, with start-anchored camera-height scale as the headline for C.
5. Spike analysis code outside the ROS packages, ROS confined to one bag adapter.

## Owner decisions already made (2026-10-27)

- Failures **C and B** for the spike; A only if T-001 shows that the jumps persist (Q1).
- **T-001 runs first** (Q2). This design also keeps the recorder out of T-001's session.
- **AC9 thresholds blank**: the report gives numbers and "not judged" (Q3).
- **At least 3 Set P passes, a 10-minute rest recording, 5 guarded 1 m drives** (Q4).
- **At most 0.25 extra Pi cores** while recording (Q6).
- Phase 1 offline spike only; Phase 2 out of scope (scope approval, `980a52a`).

## Open questions remaining (from REQ, with defaults)

- **Q5** Is the tape accurate enough to be the reference? Default: accept it for the spike, state "a few cm" as the
  floor on every error, and ask after the spike whether a better reference is wanted for C. Optional and cheap:
  ruler offsets of chassis to cross at each placement in S4 (PLAN section 5 method, about 0.5 cm), recorded in
  `tape.yaml`.
- **Q7** Source for the 2026-09-28 "about 90 ms late scans" figure. Default: owner-supplied; it lives in the run-recipe
  memory note, not in the repo; cite BUILD_LOG.md:155 for the 09-25 fault. It does not affect Phase 1.
- **Q8** Camera calibration. Default: unknown; established by S0 and S2 as above.
- **Q9** FOV and frame rate enough. Default: unknown; the per-frame feature and tracking logs answer it.
- **Q10** Second consumer of the camera. Default: record the existing compressed stream; measured in S1; no change to
  existing consumers.
- **Q11** Fiducial tags. Default: only after the spike.
- **Q12** Bag storage and copy. Default: local SD card under `~/captures/`, `df` gate at twice the estimate, copy by
  hand after each session with `rsync`, delete on the Pi only after the sizes match.
- **Q13** Same room and set as 2026-10-27. Default: yes.
- **Q14** GPU for offline SLAM. Default: no GPU unless the CPU systems are inconclusive; then ask, one job, via
  `gpuq`.

## Design decisions the owner must make to approve

- **D1** Recommended systems: ORB-SLAM3 monocular (v1.0) plus a ground-plane VO written as a tested pure module. Accept,
  or ORB-SLAM3 only (cheaper, but no metric answer for B), or another primary.
- **D2** Accept JPEG compression inside `camera_node` as the recording path, with fallbacks in the order given, and
  `jpeg_quality` as the only camera knob (changed only with agreement).
- **D3** Cartographer config for the spike sessions: whichever T-001 recommends (default), or fixed now.
- **D4** Merge `capture-recorder` (`ea199de`, `record_capture.sh`) into the spike branch, rather than writing a new
  recorder.
- **D5** Code location: `analysis/camera_spike/` with `COLCON_IGNORE`, added to `run_tests.sh`; Pi scripts in
  `tortoisebot_firmware/scripts/`.
- **D6** Session list S0 to S5 and the robot time (roughly 3 to 4 hours of owner presence in total, likely over two
  days), including the S1 cost arms and the S2 checkerboard (the owner holds a printed board).
- **D7** Privacy and backup of the bags: an excluded path (default), or accept backup.
- **D8** Which branch this design is committed on: it was written into the `feature/T-001-rigid-frame-slam` working
  tree, where the requirements file does not exist; suggested home is `feature/precision-5cm` beside `980a52a`, or a
  new design branch.
