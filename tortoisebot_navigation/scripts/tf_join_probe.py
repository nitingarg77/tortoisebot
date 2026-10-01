#!/usr/bin/env python3
"""Does a participant joining or leaving the graph freeze controller_server's TF?

    tf_join_probe.py --log <compute.log> --out <events file> [--quiet 240] [--stim 480]

Run by tf_join_launch.sh, which brings Nav2 up and caps velocity_smoother's
output at zero first. Commands NO motion: see "Why nothing can move" below.

The hypothesis (MODULE.md, "The TF-rate A/B"): every freeze of
controller_server's map->odom buffer so far followed, within ~1-8 s, a /tf
subscriber joining or leaving the graph. Four of four, but most such events
trigger nothing. This makes those events on purpose, on a schedule, and
watches for the freeze.

Phases
------
quiet     probes only, no stimulus. The spontaneous rate.
stimulus  every 40 s a short-lived process joins the graph for 15 s and leaves,
          alternating two kinds:
            T  a node with a tf2 TransformListener  (joins /tf and /tf_static)
            P  a plain node with no subscriptions    (joins the graph only)
          If freezes follow T and not P, it is /tf matching; if both, it is
          participant discovery in general.

The probe
---------
A frozen controller_server logs nothing while idle -- it only reports stale TF
from isGoalReached(), which runs during a goal. So every 10 s this node sends
the controller's own FollowPath action a one-pose path in `map`, stamped now,
and cancels it within 2.5 s. Stamped now, a frozen map->odom makes the
controller log "Transform data too old" with the frozen Transform time. That
line is the detector, read back from the compute log.

This node is ONE long-lived process on purpose. Starting a fresh process per
probe would itself join and leave the graph -- the very stimulus under test.

Why nothing can move
--------------------
- FollowPath goes straight to controller_server: no behaviour tree, so no
  recovery behaviours, and behavior_server (which publishes to /cmd_vel
  directly) is never asked to spin or back up.
- The controller's output only reaches the wheels through velocity_smoother,
  whose max_velocity and min_velocity are set to zero and verified by the
  launch script before this starts; the smoother clamps every command to them.
"""

import argparse
import os
import re
import subprocess
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath

PAIR = re.compile(r'Data time: (\d+)s (\d+)ns, Transform time: (\d+)s (\d+)ns')

STIM_T = r'''
import rclpy, time
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = Node("stim_tf"); b = Buffer(); l = TransformListener(b, n)
print("READY", time.time(), flush=True)
end = time.time() + {life}
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.2)
n.destroy_node(); rclpy.shutdown(); print("LEFT", time.time(), flush=True)
'''
STIM_P = r'''
import rclpy, time
from rclpy.node import Node
rclpy.init(); n = Node("stim_plain")
print("READY", time.time(), flush=True)
end = time.time() + {life}
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.2)
n.destroy_node(); rclpy.shutdown(); print("LEFT", time.time(), flush=True)
'''


class Probe(Node):
    def __init__(self):
        super().__init__('tf_join_probe')
        self.ac = ActionClient(self, FollowPath, 'follow_path')

    def once(self, wait=2.5):
        """Send a one-pose path stamped now; cancel it; return a status word."""
        now = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = now
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = now
        p.pose.orientation.w = 1.0
        path.poses = [p]
        fut = self.ac.send_goal_async(FollowPath.Goal(path=path))
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 3.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        h = fut.result() if fut.done() else None
        if h is None or not h.accepted:
            return 'rejected'
        res = h.get_result_async()
        while not res.done() and time.time() - t0 < wait:
            rclpy.spin_once(self, timeout_sec=0.05)
        if res.done():
            return 'result=%d' % res.result().status
        c = h.cancel_goal_async()
        t1 = time.time()
        while not c.done() and time.time() - t1 < 2.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        return 'cancelled'


class LogTail:
    """New controller_server stale-TF lines since the last read."""
    def __init__(self, path):
        self.path, self.pos = path, 0

    def new_pairs(self):
        out = []
        try:
            with open(self.path, errors='replace') as f:
                f.seek(self.pos)
                for line in f:
                    if 'controller_server' in line:
                        m = PAIR.search(line)
                        if m:
                            out.append((int(m.group(1)) + int(m.group(2)) * 1e-9,
                                        int(m.group(3)) + int(m.group(4)) * 1e-9))
                self.pos = f.tell()
        except FileNotFoundError:
            pass
        return out


def threads_snapshot(pid, path):
    """Per-thread state, wait channel, syscall and CPU of controller_server.

    Read-only, through /proc; wchan and syscall need sudo because ptrace_scope
    is 1. A thread that was busy while healthy and sits in the same wait from
    the moment of the freeze is the listener; its wait channel says whether it
    is waiting on a lock (futex) or for data (epoll/poll).
    """
    rows = []
    try:
        tids = sorted(os.listdir('/proc/%d/task' % pid), key=int)
    except OSError as e:
        return 'no /proc for %d: %s' % (pid, e)
    for tid in tids:
        base = '/proc/%d/task/%s/' % (pid, tid)
        try:
            stat = open(base + 'stat').read()
            rest = stat[stat.rindex(')') + 2:].split()
            state, ut, st = rest[0], int(rest[11]), int(rest[12])
        except (OSError, ValueError, IndexError):
            continue
        def sudo_cat(name):
            try:
                return subprocess.run(['sudo', '-n', 'cat', base + name],
                                      capture_output=True, text=True,
                                      timeout=3).stdout.strip().split('\n')[0]
            except Exception:
                return '?'
        rows.append('%s %s cpu=%d wchan=%s syscall=%s' % (
            tid, state, ut + st, sudo_cat('wchan') or '-',
            (sudo_cat('syscall') or '-').split(' ')[0]))
    with open(path, 'w') as f:
        f.write('\n'.join(rows) + '\n')
    return '%d threads -> %s' % (len(rows), os.path.basename(path))


def backtrace(pid, path):
    """All-thread backtrace of controller_server, taken ONLY once frozen.

    Attaching stops the process for the duration, which is harmless once the
    freeze is confirmed but would itself make a healthy listener fall behind,
    so this is never done before. debuginfod is off so gdb does not stall on
    network symbol lookups; the ROS libraries export enough symbols for
    function names without debug packages.
    """
    cmd = ['sudo', '-n', 'timeout', '180', 'gdb', '-nx', '-batch',
           '-iex', 'set debuginfod enabled off',
           '-iex', 'set pagination off',
           '-p', str(pid),
           '-ex', 'info threads',
           '-ex', 'thread apply all bt 30']
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=200)
        out = r.stdout + r.stderr
    except Exception as e:
        out = 'gdb failed: %s' % e
    with open(path, 'w') as f:
        f.write(out)
    return '%d lines -> %s' % (out.count('\n'), os.path.basename(path))


def libtf2_mapped(pid):
    """Which libtf2.so the process actually loaded: apt's or the overlay.

    tf2_overlay.sh puts a fixed tf2 ahead of /opt/ros on the library path. The
    memory map is the only proof a given run used it.
    """
    try:
        with open('/proc/%d/maps' % pid) as f:
            libs = {ln.split()[-1] for ln in f if 'libtf2.so' in ln}
        return ' '.join(sorted(libs)) or 'none'
    except OSError as e:
        return 'unreadable: %s' % e


def controller_pid():
    r = subprocess.run(['pgrep', '-f', '/opt/ros/humble/lib/nav2_controller/controller_server'],
                       capture_output=True, text=True)
    pids = [int(x) for x in r.stdout.split()]
    return pids[0] if pids else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--quiet', type=float, default=240.0)
    ap.add_argument('--stim', type=float, default=480.0)
    ap.add_argument('--probe-every', type=float, default=10.0)
    ap.add_argument('--stim-every', type=float, default=40.0)
    ap.add_argument('--stim-life', type=float, default=15.0)
    a = ap.parse_args()

    ev = open(a.out, 'a')

    def note(msg):
        line = '%.3f %s' % (time.time(), msg)
        ev.write(line + '\n')
        ev.flush()
        print(line, flush=True)

    rclpy.init()
    n = Probe()
    tail = LogTail(a.log)
    tail.new_pairs()                      # skip anything before we started
    if not n.ac.wait_for_server(timeout_sec=20.0):
        note('FAIL follow_path server not available')
        return 2
    pid = controller_pid()
    stem = os.path.splitext(a.out)[0]
    note('start controller_pid=%s quiet=%.0f stim=%.0f' % (pid, a.quiet, a.stim))
    if pid:
        note('libtf2 mapped: ' + libtf2_mapped(pid))
        note('threads healthy: ' + threads_snapshot(pid, stem + '.threads_healthy'))

    seen = []                              # (data_time, transform_time)
    stim, stim_kind, stim_n, stim_kind_live = None, "T", 0, ""
    t_start = time.time()
    next_probe = t_start
    next_stim = t_start + a.quiet
    frozen = False
    try:
        while time.time() - t_start < a.quiet + a.stim:
            now = time.time()
            phase = 'quiet' if now - t_start < a.quiet else 'stimulus'

            # --- stimulus lifecycle ---
            if stim is not None and stim.poll() is not None:
                for line in stim.stdout.read().splitlines():
                    if line.startswith(('READY', 'LEFT')):
                        k, ts = line.split()
                        note('stim %s%d %s at %.3f' % (stim_kind_live, stim_n,
                                                       k.lower(), float(ts)))
                stim = None
            if phase == 'stimulus' and stim is None and now >= next_stim:
                stim_n += 1
                stim_kind_live = stim_kind
                src = (STIM_T if stim_kind == 'T' else STIM_P).format(
                    life=a.stim_life)
                stim = subprocess.Popen([sys.executable, '-c', src],
                                        stdout=subprocess.PIPE,
                                        stderr=subprocess.DEVNULL, text=True)
                note('stim %s%d spawned' % (stim_kind, stim_n))
                stim_kind = 'P' if stim_kind == 'T' else 'T'
                next_stim = now + a.stim_every

            # --- probe ---
            if now >= next_probe:
                outcome = n.once()
                time.sleep(0.3)            # let the log line land
                pairs = tail.new_pairs()
                seen.extend(pairs)
                if pairs:
                    d, x = pairs[-1]
                    note('probe %s %s stale age=%.1f transform_time=%.3f (%d lines)'
                         % (phase, outcome, d - x, x, len(pairs)))
                else:
                    note('probe %s %s healthy' % (phase, outcome))
                next_probe = now + a.probe_every

                # Frozen: the same Transform time on two probes >= 3 s apart.
                xs = {}
                for d, x in seen:
                    lo, hi = xs.get(x, (d, d))
                    xs[x] = (min(lo, d), max(hi, d))
                held = [(x, hi - lo) for x, (lo, hi) in xs.items() if hi - lo >= 3.0]
                if held and not frozen:
                    frozen = True
                    x, span = max(held, key=lambda v: v[1])
                    note('FROZEN at transform_time=%.3f (held %.1f s so far) phase=%s'
                         % (x, span, phase))
                    if pid:
                        note('threads frozen: ' + threads_snapshot(pid, stem + '.threads_frozen1'))
                        time.sleep(5)
                        note('threads frozen: ' + threads_snapshot(pid, stem + '.threads_frozen2'))
                        note('backtrace 1: ' + backtrace(pid, stem + '.bt1'))
                        time.sleep(5)
                        note('backtrace 2: ' + backtrace(pid, stem + '.bt2'))
                    break
            rclpy.spin_once(n, timeout_sec=0.1)
    finally:
        if stim is not None and stim.poll() is None:
            stim.terminate()
        note('end frozen=%s probes_with_stale_lines=%d' % (frozen, len(seen)))
        try:
            n.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
