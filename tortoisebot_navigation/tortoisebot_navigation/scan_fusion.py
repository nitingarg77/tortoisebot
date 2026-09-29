"""Fuse the camera's floor-boundary scan into the lidar scan. No ROS, so it
tests at a desk.

Why this node exists at all: the YDLidar cannot see the white fluted panels in
this building (nothing returned in 55-80% of scans head-on at 1.2 m) and does
not see a wooden door frame until it is inside its 0.10 m minimum range. The
robot drove into both. `floor_scan.py` sees them with the camera; the question
is how that reaches the costmaps.

It was wired in as a *second* observation source on each costmap, and that is
what hangs `controller_server`: a second `tf2_ros::MessageFilter` per layer,
whose transform listener stops being served while `/tf` on the wire stays at
66 Hz. Fusing first means one source per layer again, which is the whole
reason for this module.

The rule
--------
Per lidar beam, the **nearest valid** return of the two. Written carefully,
because the obvious `min(lidar, camera)` is wrong here:

    invalid_range_is_inf: false   (ydlidar_ros2_driver/params/ydlidar.yaml)

so this driver encodes a no-return as **0.0**, not `inf`. A plain `min()` would
return 0.0 at exactly the bearings where the camera is the only thing that can
see -- the panels -- and 0.0 is below `range_min`, so `laser_geometry` drops it
and nothing is marked at all. Validity is checked before the comparison, and a
beam neither sensor saw keeps the lidar's own encoding untouched, so the
costmap behaves there exactly as it does today.

Taking the nearer of the two is also what makes it safe to let this topic
*clear*, which `/floor_scan` as a separate marking-only source could not. The
camera can only ever pull a reading nearer, never push it farther, so it cannot
declare space free that the lidar sees as occupied -- while a camera false
positive now heals itself, because the lidar raytraces through it on the next
scan. The old wiring marked a phantom permanently.

Frames
------
Camera ranges are about `base_link`; lidar beams are about `lidar`, which
`tortoisebotreal.xacro` puts at (-0.03275, 0, 0.167) with no rotation. A planar
33 mm offset, which is not negligible at these ranges: at 0.45 m and 26.75 deg
it moves the bearing 1.8 deg and the range 3 cm. Projected per beam.

Timing
------
The lidar runs at ~11.7 Hz and the camera at 5 Hz, so camera data up to 200 ms
old gets stamped as current. At this robot's speeds translation costs 2.6 cm,
under the 5 cm costmap resolution, and is left uncorrected. Rotation costs up
to 13.8 deg at 1.2 rad/s and is not ignorable -- but rotating a planar fan is
just a bearing shift, so it is corrected exactly from the IMU's fused yaw. If
the yaw for the interval is unknown, the camera is used only when it is fresh
enough that the smear cannot matter; see `fuse`.
"""

import math

INF = float('inf')

# Lidar origin in base_link, from the lidar_joint in tortoisebotreal.xacro.
LIDAR_DX = -0.03275
LIDAR_DY = 0.0

# A camera scan older than this is dropped. Two camera frames at 5 Hz.
DEFAULT_MAX_AGE = 0.30
# Without a yaw for the interval the fan cannot be de-rotated, so the camera is
# used only inside this window. At 1.2 rad/s that bounds the smear at 3.4 deg,
# under the lidar's own 1.385 deg resolution plus a beam.
DEFAULT_UNKNOWN_YAW_MAX_AGE = 0.05
# A real floor boundary is not a wall at arm's length across the whole fan.
# Rejecting that pattern catches a camera looking at a hand, a lens cover or a
# blown exposure, any of which would otherwise stop the robot dead.
DEFAULT_CLUTTER_FRACTION = 0.8
DEFAULT_CLUTTER_RANGE = 0.6


def wrap(angle):
    """To (-pi, pi]. A cone about 180 deg falls apart at the seam without it."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


class Fan(object):
    """A planar scan reduced to what fusion needs, with no ROS types.

    Deliberately not a `sensor_msgs/LaserScan`: this module must import on a
    desk. `from_msg` reads the message's attributes and never imports it.
    """

    __slots__ = ('ranges', 'angle_min', 'angle_increment', 'range_min',
                 'range_max', 'stamp', 'frame_id')

    def __init__(self, ranges, angle_min, angle_increment,
                 range_min=0.0, range_max=INF, stamp=0.0, frame_id=''):
        self.ranges = list(ranges)
        self.angle_min = float(angle_min)
        self.angle_increment = float(angle_increment)
        self.range_min = float(range_min)
        self.range_max = float(range_max)
        self.stamp = float(stamp)
        self.frame_id = frame_id

    @classmethod
    def from_msg(cls, msg):
        """Duck-typed on LaserScan. Kept here so the node stays a thin shell."""
        h = msg.header
        return cls(msg.ranges, msg.angle_min, msg.angle_increment,
                   msg.range_min, msg.range_max,
                   h.stamp.sec + h.stamp.nanosec * 1e-9, h.frame_id)

    @property
    def count(self):
        return len(self.ranges)

    @property
    def wraps(self):
        """True for a full-circle scan, whose indices are modular."""
        return self.count * abs(self.angle_increment) >= 2 * math.pi - 1e-6

    def bearing(self, i):
        return self.angle_min + i * self.angle_increment

    def valid(self, r):
        """A reading that means something.

        `r == r` rejects NaN, which fails every comparison including against
        itself. The bounds reject both `inf` and this driver's 0.0 no-return in
        one test, which is the point: neither is a distance.
        """
        return r == r and self.range_min <= r <= self.range_max

    def index_for(self, bearing):
        """Nearest beam index to a bearing, or None if outside the scan."""
        if self.angle_increment == 0.0:
            return None
        i = int(round((wrap(bearing - self.angle_min)) / self.angle_increment))
        if self.wraps:
            return i % self.count
        return i if 0 <= i < self.count else None


def project(r, bearing, dx=LIDAR_DX, dy=LIDAR_DY):
    """A polar reading about base_link, re-expressed about the lidar.

    `dx, dy` is where the lidar sits in base_link, so the reading's coordinates
    in the lidar's frame are the base_link ones minus that offset.
    """
    x = r * math.cos(bearing) - dx
    y = r * math.sin(bearing) - dy
    return math.hypot(x, y), math.atan2(y, x)


def camera_onto_lidar(camera, lidar, yaw_delta=0.0,
                      dx=LIDAR_DX, dy=LIDAR_DY):
    """{lidar beam index: nearest projected camera range}.

    The camera fan is denser than the lidar (0.89 vs 1.385 deg), so several
    camera beams can land on one lidar beam; the nearest wins, because for an
    obstacle the nearest thing in the beam is what the robot hits.

    `yaw_delta` is how far the robot turned between the camera's stamp and the
    lidar's. The fan is planar, so de-rotating it is a bearing shift.
    """
    out = {}
    for i, r in enumerate(camera.ranges):
        if not camera.valid(r):
            continue
        rr, bearing = project(r, camera.bearing(i) - yaw_delta, dx, dy)
        if rr > lidar.range_max:
            continue
        # Below the lidar's minimum the projector would drop it, so clamp up
        # rather than lose it: a reading too near to represent is the one case
        # where discarding it is the dangerous choice.
        rr = max(rr, lidar.range_min)
        j = lidar.index_for(bearing)
        if j is None:
            continue
        if j not in out or rr < out[j]:
            out[j] = rr
    return out


def is_cluttered(camera, fraction=DEFAULT_CLUTTER_FRACTION,
                 near=DEFAULT_CLUTTER_RANGE):
    """Whether the whole fan reads as a wall at arm's length.

    Measured against the total beam count, not the valid ones: a genuine narrow
    obstacle lights a few beams and must not be thrown away, while a lens cover
    or a blown exposure lights nearly all of them.
    """
    if camera.count == 0:
        return True
    close = sum(1 for r in camera.ranges if camera.valid(r) and r < near)
    return close > fraction * camera.count


def fuse(lidar, camera, yaw_delta=0.0, max_age=DEFAULT_MAX_AGE,
         unknown_yaw_max_age=DEFAULT_UNKNOWN_YAW_MAX_AGE,
         dx=LIDAR_DX, dy=LIDAR_DY,
         clutter_fraction=DEFAULT_CLUTTER_FRACTION,
         clutter_range=DEFAULT_CLUTTER_RANGE):
    """Lidar ranges with the camera's nearer readings folded in.

    Returns `(ranges, note)`. `note` is None when the camera was used, and
    otherwise says why it was not; the caller logs it. It is never an error:
    dropping to lidar-only is the designed degradation, and this node must keep
    publishing whatever happens to the camera, because `/scan_fused` is the
    costmaps' only observation source and silence blinds Nav2 completely.

    `yaw_delta=None` means the robot's rotation over the interval is unknown,
    which is not the same as zero. The fan is then used only while it is fresh
    enough that any plausible rotation is under the lidar's own resolution.
    """
    ranges = list(lidar.ranges)
    if camera is None:
        return ranges, 'no camera scan yet'

    age = lidar.stamp - camera.stamp
    if age > max_age:
        return ranges, 'camera scan stale by %.2f s' % age
    if age < -max_age:
        # The camera ahead of the lidar means the clocks disagree, not that the
        # future arrived. Using it would put obstacles at the wrong bearings.
        return ranges, 'camera scan %.2f s ahead of the lidar' % (-age)
    if yaw_delta is None:
        if abs(age) > unknown_yaw_max_age:
            return ranges, 'no yaw for a %.2f s gap' % abs(age)
        yaw_delta = 0.0
    if is_cluttered(camera, clutter_fraction, clutter_range):
        return ranges, 'camera fan reads as a wall at arm\'s length'

    for j, r in camera_onto_lidar(camera, lidar, yaw_delta, dx, dy).items():
        # The nearest *valid* return, not min(): a beam the lidar did not see
        # holds 0.0 on this driver, which would win every comparison and then
        # be dropped downstream for being under range_min.
        if not lidar.valid(ranges[j]) or r < ranges[j]:
            ranges[j] = r
    return ranges, None


class YawHistory(object):
    """Recent absolute yaw, for de-rotating a camera fan to the lidar's stamp.

    The BNO055 publishes a fused absolute yaw at 49 Hz, so both stamps in a
    fusion cycle normally fall inside the buffer and the answer is exact.
    `delta` returns None rather than 0.0 when they do not, because "the robot
    did not turn" and "nobody knows whether it turned" have to reach `fuse` as
    different answers.

    Not thread-safe by itself: the caller owns the lock.
    """

    def __init__(self, window=2.0):
        self.window = float(window)
        self.samples = []             # (t, yaw), oldest first

    def add(self, t, yaw):
        """Feed one yaw. Out-of-order and repeated stamps are dropped."""
        t = float(t)
        if self.samples and t <= self.samples[-1][0]:
            return
        self.samples.append((t, float(yaw)))
        while self.samples and (t - self.samples[0][0]) > self.window:
            self.samples.pop(0)

    def yaw_at(self, t):
        """Yaw at `t` by linear interpolation, or None outside the buffer.

        Not extrapolated. Beyond the ends the buffer knows nothing, and a
        confident wrong bearing is worse here than no correction at all.
        """
        if len(self.samples) < 2 or not (self.samples[0][0] <= t
                                         <= self.samples[-1][0]):
            return None
        for (t0, y0), (t1, y1) in zip(self.samples, self.samples[1:]):
            if t0 <= t <= t1:
                if t1 == t0:
                    return y0
                # Interpolate along the shorter way round, so a sample pair
                # straddling +/-pi does not sweep the long way through zero.
                return y0 + wrap(y1 - y0) * (t - t0) / (t1 - t0)
        return None

    def delta(self, t_from, t_to):
        """How far the robot turned between two stamps, or None if unknown."""
        a, b = self.yaw_at(t_from), self.yaw_at(t_to)
        if a is None or b is None:
            return None
        return wrap(b - a)
