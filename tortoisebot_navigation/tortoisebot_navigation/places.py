"""Named map positions, read from a file. No ROS, so it tests at a desk.

Split out of nav_course.py. The validation matters more than it looks: the
built-in places are simulator coordinates and mean nothing on a real robot, so
a real course is always loaded from a file a human typed. A file with a typo in
it becomes a goal somewhere unintended, which on this robot means driving into
something the lidar cannot see.
"""

import math

import yaml


class BadPlaces(Exception):
    """A places file that cannot be trusted to drive a robot from."""


def parse_places(text, source='<string>'):
    """Read {name: [x, y, yaw]} from YAML, keeping the file's order.

    Order is kept because it is the order the course drives, so sorting the
    names would silently rearrange the route.
    """
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as e:
        raise BadPlaces('%s: not valid YAML: %s' % (source, e))
    if data is None:
        raise BadPlaces('%s: no places in it' % source)
    if not isinstance(data, dict):
        raise BadPlaces('%s: expected a mapping of name to [x, y, yaw], got %s'
                        % (source, type(data).__name__))

    places = {}
    for name, value in data.items():
        if not (isinstance(value, (list, tuple)) and len(value) == 3):
            raise BadPlaces('%s: %s should be [x, y, yaw], got %r'
                            % (source, name, value))
        try:
            coords = tuple(float(v) for v in value)
        except (TypeError, ValueError):
            raise BadPlaces('%s: %s has a non-numeric coordinate: %r'
                            % (source, name, value))
        if not all(math.isfinite(c) for c in coords):
            raise BadPlaces('%s: %s has a non-finite coordinate: %r'
                            % (source, name, value))
        places[name] = coords
    if not places:
        raise BadPlaces('%s: no places in it' % source)
    return places, list(places)


def quaternion_from_yaw(yaw):
    """(z, w) of a yaw-only rotation. x and y are zero on a flat floor."""
    return math.sin(yaw / 2), math.cos(yaw / 2)


def yaw_from_quaternion(x, y, z, w):
    """Yaw from a quaternion, in radians."""
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y ** 2 + z ** 2))


def drive_verdict(start, end, goal, xy_tol, slack=0.05, lidar_progress=None):
    """Judge a SUCCEEDED drive goal.

    Returns 'ok', 'false_success', 'untestable' or 'pose_disagrees'.

    A frozen TF buffer makes Nav2 report success with the robot still near its
    start (MODULE.md). The test is how far the robot ENDED from the goal, not
    how far it travelled: a success inside the goal checker's `xy_tol` can
    legitimately stop up to `xy_tol` short. On 2026-10-01 a 0.30 m goal with
    0.15 m tolerance stopped after 0.149 m, correctly, and a fixed
    "travelled < 0.15" rule called it a false success.

    'untestable' when the goal is itself within tolerance (plus slack) of the
    start: there, a robot that never moved also counts as arrived.

    The poses come from Cartographer, which is not an independent witness:
    with no encoders its map pose is itself an estimate. `lidar_progress` is
    the drop in range to whatever is straight ahead (a median over a few
    beams, straight drives only). When the pose says short but the lidar says
    the robot covered the distance, the verdict is 'pose_disagrees', not a
    false success. On 2026-10-01 a 0.50 m goal ended 0.231 m short by map pose
    (0.271 m travelled, 5.9 deg turned against the IMU's 1.0 deg), while the
    lidar ahead dropped 0.34 m, enough to be within tolerance.
    """
    to_goal = math.hypot(goal[0] - start[0], goal[1] - start[1])
    if to_goal <= xy_tol + slack:
        return 'untestable'
    left = math.hypot(goal[0] - end[0], goal[1] - end[1])
    if left <= xy_tol + slack:
        return 'ok'
    if lidar_progress is not None and lidar_progress >= to_goal - xy_tol - slack:
        return 'pose_disagrees'
    return 'false_success'
