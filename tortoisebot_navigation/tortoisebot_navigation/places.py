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
