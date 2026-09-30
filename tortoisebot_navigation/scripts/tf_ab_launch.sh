#!/bin/bash
# One launch of the Cartographer TF-rate A/B, run ON the robot.
#
#     tf_ab_launch.sh <pose_publish_period_sec> <tag>
#     e.g.  tf_ab_launch.sh 5e-3 before1      # 200 Hz, as it was
#           tf_ab_launch.sh 2e-2 after1       #  50 Hz, as staged in 8d1999a
#
# Needs the robot drivers already up (scan, imu, scan_fused). Brings up
# Cartographer + Nav2, soaks, probes controller_server's TF buffer with two
# pairs of in-place turns while tf_watch.py records an independent buffer on
# the same clock, then tears everything down and verifies nothing survived.
#
# MOTION: four rotations in place, +90 / -90 / +90 / -90 degrees, returning
# to the starting heading. No translation. Each goal carries one_goal.py's
# guards (clearance in every direction, IMU-verified turn, cancel on exit,
# 21 s leash) and is additionally wrapped in `timeout`, which signals the
# script so its `finally` cancels the goal. Run this detached (setsid nohup):
# killing the ssh that started it does NOT stop a goal Nav2 already holds.
#
# Why turns and not drives: the freeze needs no translation -- isGoalReached()
# looks up map->odom on every control cycle of any goal -- and a drive walks
# the robot across the room, which makes n > 1 impossible from one spot.
#
# Why a long soak: in the only freeze observed (2026-09-29), map->odom stopped
# updating about 9 minutes after launch. A short window would report both
# configs clean and prove nothing, so each launch probes early AND late.

set -u
RATE=$1
TAG=$2
D=~/logs/ab
mkdir -p "$D"
SRC=~/tb_ws/src/tortoisebot/tortoisebot_slam/config/slam_real.lua
NAV=~/tb_ws/install/tortoisebot_navigation/lib/tortoisebot_navigation
META=$D/$TAG.meta
: > "$META"

# ROS's setup scripts read unset variables, so -u is off while they run.
set +u
source /opt/ros/humble/setup.bash
source ~/tb_ws/install/setup.bash
set -u

note() { echo "$(date +%s.%N) $*" | tee -a "$META"; }

# Patterns bracketed so pgrep never matches a shell that merely mentions them.
COMPUTE='[a]utobringup.*stack:=compute|/opt/ros/humble/lib/nav2_|[c]artographer|[f]rontier_explor|[p]ose_watchdog|[o]ne_goal\.py|[t]f_watch\.py'

# Cancel every goal on the server, from outside the client. An empty goal ID
# and zero stamp means "all goals" in action_msgs/CancelGoal. This does not
# depend on one_goal.py surviving: if `timeout` fires, rclpy's own SIGINT
# handler shuts the context down first and the script's finally may not get
# its cancel out.
cancel_all() {
  timeout 10 ros2 service call /navigate_to_pose/_action/cancel_goal \
    action_msgs/srv/CancelGoal '{}' >/dev/null 2>&1
}

teardown() {
  cancel_all
  pids=$(pgrep -f "$COMPUTE" | tr '\n' ' ')
  [ -n "$pids" ] && kill $pids 2>/dev/null
  sleep 5
  pids=$(pgrep -f "$COMPUTE" | tr '\n' ' ')
  [ -n "$pids" ] && { note "SIGKILL survivors: $pids"; kill -9 $pids 2>/dev/null; sleep 2; }
  left=$(pgrep -af "$COMPUTE" | wc -l)
  note "teardown survivors=$left"
  # differential.py holds its last duty cycle, so leave the wheels at zero
  # explicitly rather than trusting whoever was last to publish.
  timeout 4 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1
}
trap teardown EXIT

# --- 0. clean start ------------------------------------------------------
if pgrep -f "$COMPUTE" >/dev/null; then
  note "FAIL compute processes already running; refusing to start"
  pgrep -af "$COMPUTE" >> "$META"
  trap - EXIT
  exit 2
fi

# --- 1. set the rate and check it landed in the INSTALLED file ------------
sed -i "s/^  pose_publish_period_sec = .*,/  pose_publish_period_sec = $RATE,/" "$SRC"
INST=$(find ~/tb_ws/install/tortoisebot_slam -name slam_real.lua)
got=$(grep -h '^  pose_publish_period_sec' "$INST" | sed 's/.*= //;s/,//')
note "rate requested=$RATE installed=$got"
[ "$got" = "$RATE" ] || { note "FAIL installed rate mismatch"; exit 3; }

# --- 2. bring up ---------------------------------------------------------
ros2 daemon stop >/dev/null 2>&1
setsid nohup ~/logs/run_compute.sh > "$D/$TAG.compute.log" 2>&1 < /dev/null &
note "launch started"
for i in $(seq 1 60); do
  grep -q "Managed nodes are active" "$D/$TAG.compute.log" && break
  grep -q "Aborting bringup" "$D/$TAG.compute.log" && { note "FAIL nav2 aborted bringup"; exit 4; }
  sleep 3
done
grep -q "Managed nodes are active" "$D/$TAG.compute.log" || { note "FAIL nav2 not active after 180 s"; exit 4; }
note "nav2 active; sources: $(grep -c 'Subscribed to Topics: scan_fused' "$D/$TAG.compute.log") x scan_fused"

sleep 45
note "settled load=$(cut -d' ' -f1-3 /proc/loadavg) $(vcgencmd get_throttled)"

# --- 3. soak with an independent buffer recording throughout --------------
python3 "$NAV/tf_watch.py" --seconds 450 --csv "$D/$TAG.tfwatch.csv" > "$D/$TAG.tfwatch.log" 2>&1 &
TFW=$!

probe() {   # $1 = label, $2 = degrees
  note "probe $1 start load=$(cut -d' ' -f1 /proc/loadavg)"
  timeout -s INT 50 python3 "$NAV/one_goal.py" --turn "$2" > "$D/$TAG.$1.log" 2>&1
  rc=$?
  cancel_all
  note "probe $1 end rc=$rc $(grep -h '^RESULT' "$D/$TAG.$1.log")"
}

sleep 20
probe early_a 90;  sleep 8; probe early_b -90
sleep 330
probe late_a 90;   sleep 8; probe late_b -90

wait $TFW
note "tf_watch done: $(grep -h 'raw /tf\|longest buffer freeze' "$D/$TAG.tfwatch.log" | tr '\n' ' ')"
note "end load=$(cut -d' ' -f1-3 /proc/loadavg) $(vcgencmd get_throttled)"
# teardown runs from the EXIT trap
