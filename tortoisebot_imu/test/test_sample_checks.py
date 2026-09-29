"""Tests for the BNO055 plausibility gates. No sensor, no ROS.

These fire constantly on this robot -- the launch log is full of them -- so
they are tested in both directions. Too loose and a garbage sample poisons
Cartographer's gravity alignment; too tight and the filter is starved.
"""

import math

import pytest

from tortoisebot_imu.sample_checks import (
    DEFAULT_MAX_ACCEL_NORM, DEFAULT_MAX_GYRO_NORM, Rejected, check_sample,
    is_complete, norm,
)

LEVEL = (0.0, 0.0, 0.0, 1.0)          # identity rotation
RESTING = (0.0, 0.0, 9.81)            # gravity only, as sensor_msgs expects
STILL = (0.0, 0.0, 0.0)


class TestIsComplete:

    def test_a_full_tuple_passes(self):
        assert is_complete(LEVEL, 4)

    def test_none_is_a_short_read(self):
        # The Adafruit driver returns None outright when I2C fails.
        assert not is_complete(None, 4)

    def test_a_tuple_containing_none_is_a_short_read(self):
        assert not is_complete((0.0, None, 0.0, 1.0), 4)

    def test_nan_is_not_a_number(self):
        assert not is_complete((0.0, float('nan'), 0.0, 1.0), 4)

    def test_wrong_length_fails(self):
        assert not is_complete((0.0, 0.0, 1.0), 4)


class TestCheckSample:

    def test_a_healthy_resting_sample_passes(self):
        accel_norm, gyro_norm = check_sample(LEVEL, STILL, RESTING)
        assert accel_norm == pytest.approx(9.81)
        assert gyro_norm == pytest.approx(0.0)

    def test_a_short_read_is_rejected_first(self):
        # Before the norms, which cannot be computed on a partial tuple.
        with pytest.raises(Rejected, match='incomplete'):
            check_sample(None, STILL, RESTING)

    def test_the_all_zero_quaternion_is_rejected(self):
        # What the BNO055 returns while its fusion is still converging.
        # Publishing it hands every consumer an invalid rotation.
        with pytest.raises(Rejected, match='non-unit'):
            check_sample((0.0, 0.0, 0.0, 0.0), STILL, RESTING)

    def test_the_implausible_acceleration_seen_in_the_logs(self):
        # 318 m/s^2 appears repeatedly in the launch log. The chassis cannot
        # pull 32 g, so that is the sensor talking.
        with pytest.raises(Rejected, match='implausible'):
            check_sample(LEVEL, STILL, (318.0, 0.0, 0.0))

    def test_an_implausible_gyro_is_rejected(self):
        with pytest.raises(Rejected, match='implausible'):
            check_sample(LEVEL, (0.0, 0.0, 20.0), RESTING)

    def test_a_hard_but_real_manoeuvre_still_passes(self):
        # The robot genuinely spins at ~1.2 rad/s and the limit is 8.7, so
        # real motion must not be filtered out.
        check_sample(LEVEL, (0.0, 0.0, 1.2), (0.5, 0.5, 9.81))

    def test_gravity_alone_is_well_inside_the_limit(self):
        # If this ever failed, every resting sample would be dropped and
        # Cartographer would have no gravity direction at all.
        assert norm(RESTING) < DEFAULT_MAX_ACCEL_NORM

    def test_the_limits_sit_above_what_the_robot_can_produce(self):
        assert DEFAULT_MAX_ACCEL_NORM > 9.81 * 1.5
        assert DEFAULT_MAX_GYRO_NORM > 1.2 * 4

    def test_a_slightly_off_unit_quaternion_is_tolerated(self):
        # Fixed-point output is not exactly unit; rejecting that would drop
        # every sample.
        q = (0.0, 0.0, 0.05, math.sqrt(1 - 0.05 ** 2) * 1.02)
        check_sample(q, STILL, RESTING)

    def test_tolerances_are_adjustable(self):
        with pytest.raises(Rejected, match='implausible'):
            check_sample(LEVEL, STILL, RESTING, max_accel_norm=5.0)
