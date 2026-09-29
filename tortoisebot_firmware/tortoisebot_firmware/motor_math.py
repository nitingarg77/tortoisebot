"""Twist to wheel duty cycles. No ROS, no GPIO, so it can be tested at a desk.

This lives apart from differential.py deliberately. That module imports rclpy
and RPi.GPIO at import time, so nothing inside it can be reached without a
robot -- which meant the duty-cycle change in 3356eb2 was "verified" against a
copy of the logic pasted into a throwaway script rather than against the code
that actually runs. A copy proves the algorithm and nothing about the shipped
function. Everything here is importable with nothing but the standard library.

Geometry and limits come from the vendor's differential.py and are properties
of this chassis (CLAUDE.md section 7).
"""

from math import pi

MOTOR_RPM = 60.0            # at full voltage
WHEEL_DIAMETER = 0.065      # m
WHEEL_SEPARATION = 0.17     # m
MAX_PWM = 100.0             # Raspberry Pi software PWM is a percentage
DEFAULT_MIN_PWM = 15.0

MAX_SPEED = (pi * WHEEL_DIAMETER * MOTOR_RPM) / 60.0   # m/s, ~0.204


def wheel_speeds(linear, angular, separation=WHEEL_SEPARATION):
    """(left, right) wheel ground speeds for a body twist, in m/s."""
    half_diff = angular * separation / 2.0
    return linear - half_diff, linear + half_diff


def duty_for(speed, trim=1.0, min_pwm=DEFAULT_MIN_PWM, max_pwm=MAX_PWM,
             max_speed=MAX_SPEED):
    """Duty cycle for one wheel, as a percentage.

    A wheel asked for nothing gets nothing. The inherited code clamped every
    wheel up to min_pwm, so a wheel commanded to 0.0 still ran at 15% and the
    robot could neither hold a wheel still nor drive a gentle arc; the floor
    only means anything once the wheel is meant to turn at all.

    `trim` corrects the two motors not matching. On this robot the left motor
    measured 0.116 and 0.107 m/s against the right's 0.140 and 0.122, so
    left_trim is 1.28. See wheel_balance.py.
    """
    if speed == 0.0:
        return 0.0
    want = (abs(speed) / max_speed) * max_pwm * trim
    return max(min(want, max_pwm), min_pwm)


def drift_to_trim(current_trim, deg_per_m, separation=WHEEL_SEPARATION):
    """Suggest a new trim from a measured straight-line drift.

    Curving left is a positive drift and means the left wheel is slow, so the
    trim goes up. This is a starting point for the next measurement, not an
    answer: PWM is not linear in speed near stiction, and on this robot 1.28
    turned out to beat both 1.25 and 1.31, so the relationship is not even
    monotonic at that scale. Measure, adjust, measure again.
    """
    from math import radians
    # drift (rad/m) = (v_right - v_left) / (separation * v)
    ratio_error = radians(deg_per_m) * separation
    return current_trim * (1.0 + ratio_error)
