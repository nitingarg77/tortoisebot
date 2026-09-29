"""Decide whether the pose estimate has run away. No ROS, so it tests at a desk.

This is the only thing that stops a robot whose localisation has diverged, and
it was the one piece of safety logic in this repository with no test at all.
Split out of pose_watchdog.py, which needs rclpy and a live TF tree to reach.

The judgement it makes is a balance between two real events:

- Cartographer snapped the pose 0.33 m while the robot turned in place
  (2026-09-25). Read over 0.5 s that is 0.63 m/s and looks like a runaway; it
  is not, and tripping on it would halt a healthy robot.
- The shortest genuine runaway seen moved about 2 m in 2 s.

Averaging over a 2 s window separates them: the correction averages 0.17 m/s,
the runaway about 1 m/s. That is why speed is taken across the window rather
than between consecutive samples, and it is the whole reason this class exists
rather than a subtraction.

Once tripped it latches. A robot that has lost track of itself does not
recover by being left alone, and the caller must reset deliberately.
"""

import math


class RunawayDetector:
    """Sliding-window speed check over (time, x, y) samples.

    Not thread-safe by itself: the caller owns the lock. In pose_watchdog it
    is driven from a single timer callback, so there is nothing to contend.
    """

    def __init__(self, max_speed=0.5, window=2.0):
        # 2.5x the ~0.20 m/s the wheels can manage. The glitch that recovered
        # on its own peaked at 0.94 m/s and spent 2 s above 0.5 m/s, so this
        # would have caught it.
        self.max_speed = float(max_speed)
        self.window = float(window)
        self.samples = []            # (t, x, y), oldest first
        self.tripped = False
        self.trip_speed = None

    # --- queries ---------------------------------------------------------

    @property
    def warm(self):
        """True once there is enough history to judge a speed.

        Below half a window the denominator is small enough that ordinary
        jitter reads as a large speed, so no verdict is given at all.
        """
        if len(self.samples) < 2:
            return False
        return (self.samples[-1][0] - self.samples[0][0]) >= 0.5 * self.window

    def speed(self):
        """Mean speed across the window, or None while still warming up."""
        if not self.warm:
            return None
        t0, x0, y0 = self.samples[0]
        t1, x1, y1 = self.samples[-1]
        dt = t1 - t0
        if dt <= 0:
            return None
        return math.hypot(x1 - x0, y1 - y0) / dt

    # --- driving ---------------------------------------------------------

    def update(self, t, x, y):
        """Feed one pose. Returns True if this sample trips the detector.

        Samples that are not newer than the last are dropped: TF hands out the
        same transform repeatedly between updates, and counting it again would
        shrink the window for no reason.
        """
        if self.tripped:
            return False
        if self.samples and t <= self.samples[-1][0]:
            return False
        self.samples.append((float(t), float(x), float(y)))
        while self.samples and (t - self.samples[0][0]) > self.window:
            self.samples.pop(0)

        speed = self.speed()
        if speed is not None and speed > self.max_speed:
            self.tripped = True
            self.trip_speed = speed
            return True
        return False

    def reset(self):
        """Clear the latch. Deliberate, because a runaway does not self-heal."""
        self.samples.clear()
        self.tripped = False
        self.trip_speed = None
