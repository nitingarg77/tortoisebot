"""Reading a LaserScan by bearing. No ROS types, so it tests at a desk.

`cone()` is here because getting it wrong has already cost something. Used as
an odometer while the robot was turning, it swept onto other surfaces, reported
*negative* progress, and the tool built on it concluded the motors were dead
while the robot drove 6.8 m down a corridor on a request for 3. The lesson is
in `cone`'s docstring and in `is_usable_as_odometer`.

Scans are passed as plain (ranges, angle_min, angle_increment) rather than as
sensor_msgs/LaserScan so that nothing here needs ROS installed. The callers
unpack the message.
"""

import math

INF = float('inf')


def _valid(r, range_min=0.0, range_max=50.0):
    """A range that means something. NaN fails every comparison, including
    against itself, which is the cheapest way to catch it."""
    return r == r and range_min < r < range_max


def bearings(count, angle_min, angle_increment):
    """Bearing of every beam, in radians."""
    return [angle_min + i * angle_increment for i in range(count)]


def index_for(bearing, angle_min, angle_increment, count):
    """Nearest beam index to a bearing, or None if outside the scan."""
    i = int(round((bearing - angle_min) / angle_increment))
    return i if 0 <= i < count else None


def cone(ranges, angle_min, angle_increment, centre_deg=0.0, half_deg=8.0,
         reducer='median'):
    """Range within a cone about `centre_deg`, or None if nothing returned.

    `reducer` is 'median' or 'min'. Use **'min' for safety decisions** — the
    nearest thing in the cone is what you hit. Use 'median' when treating the
    reading as a measurement, because one stray beam then cannot move it.

    Do not use this as an odometer while the robot is turning. The cone is
    fixed in the robot's frame, so a turn sweeps it across different surfaces
    and the change in range has nothing to do with distance travelled. See
    `is_usable_as_odometer`.
    """
    hits = []
    half = math.radians(half_deg)
    centre = math.radians(centre_deg)
    for i, r in enumerate(ranges):
        if not _valid(r):
            continue
        a = angle_min + i * angle_increment
        # Wrap to (-pi, pi] so a cone about 180 degrees does not fall apart at
        # the seam of a 360 degree scan.
        delta = (a - centre + math.pi) % (2 * math.pi) - math.pi
        if abs(delta) <= half:
            hits.append(r)
    if not hits:
        return None
    if reducer == 'min':
        return min(hits)
    return sorted(hits)[len(hits) // 2]


def nearest(ranges, range_min=0.0, range_max=50.0):
    """Closest valid return in any direction, or inf if the scan is empty.

    inf rather than None so a caller comparing against a guard distance cannot
    accidentally treat 'saw nothing' as 'something is touching me'.
    """
    hits = [r for r in ranges if _valid(r, range_min, range_max)]
    return min(hits) if hits else INF


def is_usable_as_odometer(yaw_change_rad, tolerance_deg=3.0):
    """Whether a distance read off a fixed cone can be believed.

    A cone is fixed in the robot's frame. Turn the robot and it points at
    something else, so the range changes for a reason that is not travel. This
    is the check that was missing when a 0.5 m guard did not fire until 0.22 m,
    on a run that curved 55 degrees.
    """
    return abs(math.degrees(yaw_change_rad)) <= tolerance_deg


def travelled(start_range, current_range):
    """Distance covered, from a shrinking range to a fixed surface ahead.

    Negative means the range grew, which straight-line travel toward a surface
    cannot do. The caller must treat a negative result as a broken reference,
    not as reversing; reporting it as progress is what hid a 6.8 m overshoot.
    """
    return start_range - current_range
