"""Tests for the duty-cycle maths. Run with pytest; no robot, no ROS.

    python3 -m pytest tortoisebot_firmware/test/test_motor_math.py -q
"""

import math

import pytest

from tortoisebot_firmware.motor_math import (
    DEFAULT_MIN_PWM, MAX_PWM, MAX_SPEED, WHEEL_SEPARATION,
    drift_to_trim, duty_for, wheel_speeds,
)


def old_duty(speed):
    """The formula this replaced, kept so the change stays pinned.

    From the vendor differential.py before 3356eb2. Every non-zero speed must
    still come out identical; the single intended difference is at 0.0.
    """
    return max(min((abs(speed) / MAX_SPEED) * MAX_PWM, MAX_PWM), DEFAULT_MIN_PWM)


class TestDutyFor:

    @pytest.mark.parametrize('speed', [0.001, 0.01, 0.05, 0.075, 0.1, 0.15,
                                       0.2, 0.204, 0.3, 1.0,
                                       -0.01, -0.15, -0.3])
    def test_matches_the_formula_it_replaced(self, speed):
        assert duty_for(speed) == pytest.approx(old_duty(speed))

    def test_a_wheel_asked_for_nothing_gets_nothing(self):
        # The one intended behaviour change: the old code returned 15 here, so
        # a stopped wheel still crept and the robot could not hold a line.
        assert duty_for(0.0) == 0.0
        assert old_duty(0.0) == DEFAULT_MIN_PWM

    def test_negative_speed_uses_magnitude(self):
        # Direction is set by the GPIO pins, not the duty cycle.
        assert duty_for(-0.15) == duty_for(0.15)

    def test_never_exceeds_max_even_when_trimmed(self):
        assert duty_for(10.0) == MAX_PWM
        assert duty_for(0.2, trim=5.0) == MAX_PWM

    def test_floor_applies_only_to_a_moving_wheel(self):
        assert duty_for(1e-9) == DEFAULT_MIN_PWM
        assert duty_for(0.0) == 0.0

    def test_trim_scales_a_moving_wheel(self):
        base = duty_for(0.15)
        assert duty_for(0.15, trim=1.28) == pytest.approx(base * 1.28)

    def test_trim_cannot_wake_a_stopped_wheel(self):
        assert duty_for(0.0, trim=1.28) == 0.0

    def test_the_measured_stiction_window(self):
        # Measured with the IMU: 0.6 rad/s of commanded yaw produced no
        # rotation at all, 1.0 rad/s gave 1.19 actual. Those are 25% and 42%
        # duty, which is the evidence that min_pwm of 15 never reached
        # stiction. If this maths changes, that reasoning needs redoing.
        for omega, expected in ((0.6, 25.0), (1.0, 42.0)):
            _, right = wheel_speeds(0.0, omega)
            assert duty_for(right) == pytest.approx(expected, abs=1.0)


class TestWheelSpeeds:

    @pytest.mark.parametrize('linear,angular', [
        (0.0, 0.0), (0.15, 0.0), (0.0, 1.0), (0.1, -0.5), (-0.15, 0.0),
        (0.075, 0.882), (-0.08, 1.2), (0.204, 2.0),
    ])
    def test_matches_the_formula_it_replaced(self, linear, angular):
        """differential.py used to compute these inline; d755d0c moved them
        here. Same numbers, or the robot steers differently."""
        vr_plus_vl = 2 * linear
        vr_minus_vl = angular * WHEEL_SEPARATION
        right = (vr_plus_vl + vr_minus_vl) / 2
        left = vr_plus_vl - right
        assert wheel_speeds(linear, angular) == (pytest.approx(left),
                                                 pytest.approx(right))

    def test_straight_drives_both_wheels_equally(self):
        left, right = wheel_speeds(0.15, 0.0)
        assert left == pytest.approx(right) == pytest.approx(0.15)

    def test_spin_in_place_is_equal_and_opposite(self):
        left, right = wheel_speeds(0.0, 1.0)
        assert left == pytest.approx(-right)
        assert right == pytest.approx(WHEEL_SEPARATION / 2)

    def test_one_wheel_can_be_held_at_exactly_zero(self):
        # wheel_balance.py depends on this: it pivots on one wheel to measure
        # the other, which only isolates a wheel if the held one is truly 0.
        speed = 0.15
        left, right = wheel_speeds(speed / 2, speed / WHEEL_SEPARATION)
        assert left == pytest.approx(0.0, abs=1e-12)
        assert right == pytest.approx(speed)
        assert duty_for(left) == 0.0

    def test_round_trip(self):
        for linear, angular in ((0.15, 0.0), (0.0, 1.0), (0.1, -0.5)):
            left, right = wheel_speeds(linear, angular)
            assert (left + right) / 2 == pytest.approx(linear)
            assert (right - left) / WHEEL_SEPARATION == pytest.approx(angular)


class TestDriftToTrim:

    def test_curving_left_raises_the_trim(self):
        assert drift_to_trim(1.0, +7.0) > 1.0

    def test_curving_right_lowers_it(self):
        assert drift_to_trim(1.28, -3.0) < 1.28

    def test_no_drift_leaves_it_alone(self):
        assert drift_to_trim(1.28, 0.0) == pytest.approx(1.28)

    def test_lands_near_the_value_measured_on_the_robot(self):
        # Untrimmed, the first clean forward run read +7.0 deg/m, and the trim
        # that measurement eventually settled on was 1.28. A first suggestion
        # in that neighbourhood is all this is for.
        assert drift_to_trim(1.0, 7.0) == pytest.approx(1.02, abs=0.02)

    def test_is_only_a_starting_point(self):
        # Documenting the limit rather than pretending it is exact: on the
        # robot 1.28 beat both 1.25 and 1.31, so a single step of this will
        # not land on the answer and the caller must re-measure.
        suggested = drift_to_trim(1.25, -2.9)
        assert not math.isclose(suggested, 1.28, abs_tol=0.005)
