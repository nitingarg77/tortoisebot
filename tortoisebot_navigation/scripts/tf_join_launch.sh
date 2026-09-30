#!/bin/bash
# One launch of the join/leave test, run ON the robot. Commands NO motion.
#
#     tf_join_launch.sh <tag> [quiet_s] [stim_s]
#
# Needs the robot drivers already up. Brings up Cartographer + Nav2 at the
# committed TF rate, caps velocity_smoother's output at zero and verifies it,
# then runs tf_join_probe.py: probes controller_server's TF buffer every 10 s
# through its own FollowPath action, first with no stimulus, then with
# processes joining and leaving the graph every 40 s. Stops at the first
# freeze, snapshots controller_server's threads, and tears everything down.
#
# Why nothing can move: FollowPath bypasses the behaviour tree, so no recovery
# behaviour can be triggered (behavior_server publishes /cmd_vel directly),
# and the controller's only path to the wheels is velocity_smoother, which
# clamps every command to max/min_velocity -- both set to zero and read back
# before the first probe. If either check fails, no probe is sent.

set -u
TAG=$1
QUIET=${2:-240}
STIM=${3:-480}
D=~/logs/join
mkdir -p "$D"
NAV=~/tb_ws/src/tortoisebot/tortoisebot_navigation/scripts
META=$D/$TAG.meta
: > "$META"

# ROS's setup scripts read unset variables, so -u is off while they run.
set +u
source /opt/ros/humble/setup.bash
source ~/tb_ws/install/setup.bash
set -u

note() { echo "$(date +%s.%N) $*" | tee -a "$META"; }

COMPUTE='[a]utobringup.*stack:=compute|/opt/ros/humble/lib/nav2_|[c]artographer|[f]rontier_explor|[p]ose_watchdog|[t]f_join_probe\.py'

cancel_all() {
  timeout 10 ros2 service call /follow_path/_action/cancel_goal \
    action_msgs/srv/CancelGoal '{}' >/dev/null 2>&1
}

teardown() {
  cancel_all
  pids=$(pgrep -f "$COMPUTE" | tr '\n' ' ')
  [ -n "$pids" ] && kill $pids 2>/dev/null
  sleep 5
  pids=$(pgrep -f "$COMPUTE" | tr '\n' ' ')
  [ -n "$pids" ] && { note "SIGKILL survivors: $pids"; kill -9 $pids 2>/dev/null; sleep 2; }
  note "teardown survivors=$(pgrep -af "$COMPUTE" | wc -l)"
  timeout 4 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1
}

if pgrep -f "$COMPUTE" >/dev/null; then
  note "FAIL compute processes already running; refusing to start"
  exit 2
fi
trap teardown EXIT

INST=$(find ~/tb_ws/install/tortoisebot_slam -name slam_real.lua)
note "tf rate installed: $(grep -h '^  pose_publish_period_sec' "$INST" | sed 's/.*= //;s/,//')"

ros2 daemon stop >/dev/null 2>&1
setsid nohup ~/logs/run_compute.sh > "$D/$TAG.compute.log" 2>&1 < /dev/null &
note "launch started"
for i in $(seq 1 60); do
  grep -q "Managed nodes are active" "$D/$TAG.compute.log" && break
  grep -q "Aborting bringup" "$D/$TAG.compute.log" && { note "FAIL nav2 aborted bringup"; exit 4; }
  sleep 3
done
grep -q "Managed nodes are active" "$D/$TAG.compute.log" || { note "FAIL nav2 not active after 180 s"; exit 4; }
note "nav2 active"

# --- the motion lock: cap the smoother, then prove it ----------------------
# Every ros2 CLI call is wrapped in `timeout`: join3 sat in one `ros2 param get`
# for 294 s with no output until it was killed by hand.
# Set and read back, retried: with the CLI daemon stopped every call rediscovers
# the graph from scratch, and on a loaded Pi the first one can miss the node
# (join1: max_velocity silently unchanged while min_velocity landed).
cap() {   # $1 = parameter name
  for try in 1 2 3; do
    out=$(timeout 30 ros2 param set /velocity_smoother "$1" "[0.0, 0.0, 0.0]" 2>&1 | tr -d '\n')
    got=$(timeout 30 ros2 param get /velocity_smoother "$1" 2>&1 | tr -d '\n')
    note "cap $1 try $try: set -> '$out'"
    echo "$got" | grep -q "\[0.0, 0.0, 0.0\]" && break
    sleep 3
  done
}
cap max_velocity
cap min_velocity
maxv=$(timeout 30 ros2 param get /velocity_smoother max_velocity 2>&1 | tr -d '\n')
minv=$(timeout 30 ros2 param get /velocity_smoother min_velocity 2>&1 | tr -d '\n')
note "smoother max_velocity: $maxv"
note "smoother min_velocity: $minv"
nonzero() { echo "$1" | grep -oE '\-?[0-9]+\.[0-9]+' | grep -qvE '^-?0\.0+$'; }
if ! echo "$maxv" | grep -q "0.0, 0.0, 0.0" || ! echo "$minv" | grep -q "0.0, 0.0, 0.0" \
   || nonzero "$maxv" || nonzero "$minv"; then
  note "FAIL velocity cap did not land; no probes sent"
  exit 5
fi
subs=$(timeout 30 ros2 topic info -v /cmd_vel_nav 2>/dev/null | awk '/^Node name:/{n=$3} /^Endpoint type: SUBSCRIPTION/{print n}' | tr '\n' ' ')
note "/cmd_vel_nav subscribers: $subs"
[ "$(echo $subs)" = "velocity_smoother" ] || { note "FAIL something other than velocity_smoother reads /cmd_vel_nav"; exit 5; }

sleep 30
note "settled load=$(cut -d' ' -f1-3 /proc/loadavg) $(vcgencmd get_throttled)"

python3 "$NAV/tf_join_probe.py" --log "$D/$TAG.compute.log" --out "$D/$TAG.events" \
  --quiet "$QUIET" --stim "$STIM" > "$D/$TAG.probe.out" 2>&1
note "probe exited rc=$? $(tail -1 "$D/$TAG.events")"
note "end load=$(cut -d' ' -f1-3 /proc/loadavg) $(vcgencmd get_throttled)"
