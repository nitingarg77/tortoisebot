"""Which BNO055 samples are fit to publish. No ROS, no I2C, so it tests here.

These gates decide what reaches Cartographer, and on this robot they fire
constantly -- the launch log is full of "incomplete read", "non-unit
quaternion" and "implausible reading". That makes them worth testing in both
directions: too loose and a garbage sample poisons the SLAM gravity alignment,
too tight and the filter is starved of the samples it needs.

Split out of imu_node.py, which imports `board` and cannot be reached without
the sensor wired up.
"""

import math

# 2g. The chassis cannot pull anything like it, so a larger reading is the
# sensor talking rather than the robot moving.
DEFAULT_MAX_ACCEL_NORM = 19.6      # m/s^2
# ~500 deg/s, several times what the wheels can produce even spinning in place.
DEFAULT_MAX_GYRO_NORM = 8.7        # rad/s
# The BNO055 returns an all-zero quaternion while its fusion is converging, and
# occasionally a partly-written one. Either is an invalid rotation.
QUATERNION_TOLERANCE = 0.1


class Rejected(Exception):
    """A sample that must not be published, carrying why for the log."""


def is_complete(values, expected):
    """True if the driver handed back a full tuple of real numbers.

    The Adafruit driver returns None, or a tuple containing None, when an I2C
    read is short. Publishing that raises deep inside message conversion, far
    from the cause.
    """
    if values is None or len(values) != expected:
        return False
    return all(v is not None and v == v for v in values)   # v == v rejects NaN


def norm(values):
    return math.sqrt(sum(float(c) ** 2 for c in values))


def check_sample(quat, gyro, accel,
                 max_accel_norm=DEFAULT_MAX_ACCEL_NORM,
                 max_gyro_norm=DEFAULT_MAX_GYRO_NORM,
                 quat_tolerance=QUATERNION_TOLERANCE):
    """Raise Rejected if this sample should be dropped, else return the norms.

    Checked in the order the failures actually happen: a short read first,
    because the other two cannot be computed on a partial tuple.
    """
    if not (is_complete(quat, 4) and is_complete(gyro, 3)
            and is_complete(accel, 3)):
        raise Rejected('incomplete BNO055 read')

    q = norm(quat)
    if abs(q - 1.0) > quat_tolerance:
        raise Rejected('non-unit quaternion (norm %.3f)' % q)

    a, g = norm(accel), norm(gyro)
    if a > max_accel_norm or g > max_gyro_norm:
        raise Rejected('implausible reading (|accel| %.1f m/s^2, |gyro| '
                       '%.2f rad/s)' % (a, g))
    return a, g
