# Build log

Dated entries of what was built, what broke, what was measured and what is
still open. Newest first. Numbers are as measured on the day; commit hashes
are on branch `frontier-exploration` unless noted.

---

## 2026-09-25 — second machine, frontier exploration, and a Pi that cannot keep up

### Machines

| Name | Hardware | Role | Address |
|---|---|---|---|
| `raspberry-pi` | Raspberry Pi 4 B, 8 GB, Ubuntu 22.04, kernel 5.15 raspi | on the robot: drivers | 192.168.1.6 (also .23), Wi-Fi |
| `nitin-linux2` | desktop, 12 cores, RTX 3060 | SLAM, Nav2, explorer, RViz | 192.168.1.100, wired |
| `linux-mac` | MacBook Air 2015 (MacBookAir7,1), i5-5250U, 3.7 GB, Ubuntu 22.04 | operator station | 192.168.1.18, Wi-Fi; Tailscale 100.99.197.70 |

All three run ROS 2 Humble with the defaults: domain 0, Fast DDS. Nodes
discover each other across machines with no extra configuration (checked
with talker/listener both ways, then `robot_description`, `tf_static` and
`joint_states` from the robot).

Clock offset between the Pi and the desktop is under 5 ms (IMU messages
arrive with a median age of −1 ms). Lidar messages arrive ~113 ms after their
stamp, with Wi-Fi spikes up to 0.7 s.

### Setup notes

- **`tb_ws` on the laptop is built there, not copied.** `--symlink-install`
  bakes absolute `/workspace/robotics/tb_ws/...` paths into `install/`, and
  the laptop has no `/workspace`. `src/` is rsynced and built in place
  (51 s, `--parallel-workers 2`). `ros-humble-robot-localization` had to be
  installed there.
- **The Pi's workspace is built without `--symlink-install`, and must stay
  that way.** Switching modes fails in `ament_cmake_python` ("existing path
  cannot be removed: Is a directory") and leaves the mode recorded in each
  package's `CMakeCache.txt`. Recovering means deleting that package's
  `build/` and `install/` directories. This machine hit the same thing
  for `tortoisebot_firmware`, which was first built in copy mode on 09-16.
- **Latched topics need matching QoS on the command line.**
  `robot_description` and `tf_static` are published once, transient-local:
  `ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable /tf_static`.
  Without those flags, before the topic name, the echo prints nothing and looks
  like a network fault.
- **RViz on the laptop** runs, but the map/costmap shader fails to link on Mesa
  ("active samplers with a different type refer to the same texture image
  unit"), so costmaps render as rainbow stripes. Software rendering does the
  same. RViz now runs on the desktop's RTX 3060, where it renders correctly.

### Faults found, and what was done

**Pi Wi-Fi dropped out.** `brcmfmac: CMD53 sg block write failed -84` at
11:48; the Pi kept running ROS with no network until power-cycled. Wi-Fi power
saving was on. It is now turned off at every boot by a unit installed on the Pi
only (not in this repo), `/etc/systemd/system/wifi-powersave-off.service`:

```ini
[Unit]
Description=Disable Wi-Fi power saving on wlan0 (brcmfmac drops out with it on)
After=sys-subsystem-net-devices-wlan0.device netplan-wpa-wlan0.service
BindsTo=sys-subsystem-net-devices-wlan0.device

[Service]
Type=oneshot
ExecStart=/usr/sbin/iw dev wlan0 set power_save off
RemainAfterExit=yes

[Install]
WantedBy=sys-subsystem-net-devices-wlan0.device
```

Verified across a reboot (`Power save: off`). No dropout since.

**IMU spikes moved the map 240 m** (`b615fab`). The BNO055 intermittently
returns ~318 m/s² (32 g, beyond its own 4 g range) on a stationary robot,
always about the same value, which points at the I2C bus or wiring rather
than noise. These samples passed the quaternion check, and Cartographer, with
no wheel odometry, integrated them into a 240 m jump in `odom → base_link`.
`imu_node.py` now drops samples with |accel| > 19.6 m/s² or |gyro| > 8.7 rad/s.
Afterwards, a stationary robot held within 3 cm for 3 minutes, and 15+ spikes
were rejected in that time.

**Motors kept the last command forever** (`f1c3f8b`). `differential.py` is
open-loop PWM and held the last duty cycle until a new `cmd_vel` arrived. A
Wi-Fi drop, a crashed controller or a closed teleop left the robot driving.
It now stops after 1 s without `cmd_vel` (`cmd_vel_timeout`, 0 disables).
Tested on the robot: PWM 15 on command, 0 at 1.2 s. On shutdown it zeroes the
duty cycle and calls `GPIO.cleanup()`, because the software-PWM thread could
otherwise leave an enable pin latched high.
**Keyboard teleop now needs the key held**; a single tap moves the robot for 1 s.

**The Pi 4 cannot run SLAM and Nav2 together.** With Cartographer, Nav2,
explorer and camera: load average ~13 on 4 cores, 67–70 °C, soft temperature
limit reached (`get_throttled 0x80000`), and 231 behaviour-tree overruns. Nav2
servers timed out acknowledging each other and every goal failed. Once the
robot moved, Cartographer's pose ran away (−138 → −431 → −559 m). Measures:

| Change | Commit | Effect |
|---|---|---|
| `camera:=False` launch argument | `dcf3dfa` | camera cost ~21% CPU; unused by exploration |
| Explorer: 0.5 Hz, greedy MRTSP, no preemption | `dcf3dfa` | was ~28% CPU |
| Cartographer: search window 0.1 → 0.3 m, motion filter 1° / 5 cm / 1 s, optimise every 90 nodes | `244f9f6` | hand-driven 6.5 min, ~40 m loop: no runaway, loop almost closed, 16 submaps instead of 35 |
| Nav2: DWB 10×20 samples (was 20×100), local costmap 5/2 Hz, BT loop 20 ms, trajectory debug and Groot off | `88c7917` | see tests B below |
| `stack:=robot` / `stack:=compute` | `dcf3dfa` | SLAM and Nav2 on the desktop instead |

Even with all of the above, the full stack on the Pi (test B) overloaded it
until SSH stopped answering within a minute. **Running SLAM and Nav2 on the
Pi 4 is not viable; the split setup is the working configuration.**

**Nav2 abandoned goals for lack of progress** (`88c7917`). With open-loop
motors the robot averaged 0.04 m/s while exploring, and turning in place
covers no distance, so the 0.5 m in 10 s progress check abandoned 11 of 12
goals. Now 0.25 m in 20 s.

**Glass is invisible to the lidar.** In test A the robot pushed against a
glass wall for about a minute ("Failed to make progress") until it was lifted
and moved by hand. The explorer also sees open space through glass, so the
frontier beyond it stays attractive.

### Added

- **Frontier exploration** (`dcf3dfa`): [mertgulerx/frontier_exploration_ros2](https://github.com/mertgulerx/frontier_exploration_ros2),
  cloned into `tb_ws/src/frontier_exploration_ros2` at `ec530d2` on each
  machine. **That clone is not tracked by this repo.** `autobringup` starts
  it 30 s in, with Nav2, when `exploration:=True`. It idles until started
  unless `auto_explore:=True`. Tuning is in
  `tortoisebot_navigation/config/explore_params_robot.yaml` (base frame
  `base_link`, 0.15 m/s, 0.5 m minimum goal distance, settle after each goal,
  suppression of repeatedly failing frontiers).
  Start or stop it with `ros2 run frontier_exploration_ros2 frontier_exploration_ctl start|stop`,
  or from the RViz panel `frontier_exploration_ros2_rviz/ExplorationControlPanel`.
  The panel has to be built separately with
  `--base-paths src/frontier_exploration_ros2/plugin/frontier_exploration_ros2_rviz`.
- **Pose watchdog** (`dcf3dfa`, `tortoisebot_navigation/scripts/pose_watchdog.py`).
  The wheels cap the robot at 0.20 m/s (65 mm, 60 rpm). If `odom → base_link`
  averages more than 0.5 m/s over 2 s, it stops the explorer, cancels all
  NavigateToPose goals and publishes zero `cmd_vel`. It stays tripped until
  `~/reset`. A 0.5 s window tripped on a one-off 0.33 m scan-matching
  correction, which is why it averages over 2 s. Checked offline against
  synthetic TF: a 0.33 m step does not trip it; a 1 m/s runaway trips it
  within ~1 s. In test A it tripped when the robot was carried by hand.
- **Launch arguments** on `autobringup.launch.py`: `camera`, `stack`
  (`all` | `robot` | `compute`), `rviz`, `auto_explore`.

### Exploration tests

| Run | Setup | Result |
|---|---|---|
| 1 | all on Pi, camera on | stopped: pose jumped to (142, −190) m before the explorer started (IMU spikes) |
| 2 | all on Pi, IMU filter | no goal reached, every one failed; Nav2 timeouts, load 13 |
| 3 | all on Pi, no camera, lighter explorer | robot drove a few metres, then pose ran away to −559 m; stopped by shutting down bringup |
| 4 | hand-driven, `slam_only`, tuned Cartographer | ~40 m in 6.5 min, no runaway, loop almost closed |
| 5 | split (drivers on Pi, rest on desktop) | 1 of 7 goals reached, 11 "Failed to make progress", no runaway; Pi load 1.4, 52 °C |
| A | split + lighter Nav2 + relaxed progress check | 1 goal reached, 3 no-progress, stuck on glass, watchdog tripped when carried |
| B | all on Pi + lighter Nav2 | not run: Pi stopped answering SSH within a minute of bringup |

### How to run it now

On the Pi:

```bash
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=False stack:=robot camera:=False
```

On the desktop:

```bash
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=False stack:=compute rviz:=False exploration:=True
```

Then run RViz on the desktop with a copy of `nav2.rviz` that adds the
Exploration panel and the `/explore/frontiers` markers (`~/tb_explore.rviz`
on the desktop, not in this repo).

To stop everything on the robot: `ssh nitin@192.168.1.6 bash ~/stop_bringup.sh`.
That script (Pi only) interrupts the launch and then any orphaned nodes.
`ros2 launch` sometimes exits on SIGINT and leaves children running.

### Open

- **Glass.** Put a strip of paper or matte tape on the glass at lidar height,
  or add a Nav2 keepout mask for mapped rooms.
- **IMU wiring.** The 318 m/s² spikes and non-unit quaternions (~50 in
  5 min) are filtered, but they point at the I2C connection.
- **Pi cooling.** It reaches 67–70 °C under load; add a heatsink or fan.
- **RViz dies when Cartographer restarts** (desktop and laptop, several
  times). Close RViz before restarting SLAM and reopen it afterwards.
- **`nav2.rviz` Image panel** subscribes to `/camera/image_raw`, but the camera
  publishes `/camera_node/image_raw`.
- **Nav2 changes are only validated in exploration so far.** The 10-goal
  course (MODULE.md §5) ran on 09-24, headless in simulation, before these
  changes.
  Rerun it after `88c7917`.
- **Pi's `.git`** was not yet synced to `frontier-exploration` (it was off
  charging); its working tree already matches.
