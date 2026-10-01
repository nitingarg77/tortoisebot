"""Tests for the places file and yaw conversion. No robot, no ROS."""

import math

import pytest

from tortoisebot_navigation.places import (
    BadPlaces, drive_verdict, parse_places, quaternion_from_yaw,
    yaw_from_quaternion,
)


class TestParsePlaces:

    def test_reads_a_good_file(self):
        places, order = parse_places('kitchen: [0.5, 2.3, 0.0]\ndock: [0, 0, 0]')
        assert places['kitchen'] == (0.5, 2.3, 0.0)
        assert order == ['kitchen', 'dock']

    def test_keeps_file_order_not_alphabetical(self):
        # Order is the order the course drives; sorting would reroute it.
        _, order = parse_places('zulu: [0,0,0]\nalpha: [1,1,0]')
        assert order == ['zulu', 'alpha']

    def test_integers_become_floats(self):
        places, _ = parse_places('dock: [0, 0, 0]')
        assert all(isinstance(c, float) for c in places['dock'])

    @pytest.mark.parametrize('text,why', [
        ('', 'no places'),
        ('kitchen: [0.5, 2.3]', 'should be'),
        ('kitchen: [0.5, 2.3, 0.0, 1.0]', 'should be'),
        ('kitchen: 0.5', 'should be'),
        ('kitchen: [a, b, c]', 'non-numeric'),
        ('- just\n- a\n- list', 'expected a mapping'),
        ('kitchen: [0.5, 2.3, 0.0]\n  bad indent', 'not valid YAML'),
    ])
    def test_rejects_what_it_cannot_drive_from(self, text, why):
        with pytest.raises(BadPlaces, match=why):
            parse_places(text)

    def test_rejects_non_finite_coordinates(self):
        # .inf parses as a float, so it survives the type check and would
        # become a goal at infinity.
        with pytest.raises(BadPlaces, match='non-finite'):
            parse_places('kitchen: [.inf, 0, 0]')

    def test_rejects_nan(self):
        with pytest.raises(BadPlaces, match='non-finite'):
            parse_places('kitchen: [.nan, 0, 0]')

    def test_the_error_names_the_file_and_the_place(self):
        with pytest.raises(BadPlaces, match='my_room.yaml: kitchen'):
            parse_places('kitchen: [0.5]', source='my_room.yaml')


class TestYaw:

    @pytest.mark.parametrize('yaw', [0.0, 0.5, 1.57, 3.0, -1.57, -3.0])
    def test_round_trips(self, yaw):
        z, w = quaternion_from_yaw(yaw)
        assert yaw_from_quaternion(0.0, 0.0, z, w) == pytest.approx(yaw)

    def test_zero_yaw_is_the_identity(self):
        assert quaternion_from_yaw(0.0) == (pytest.approx(0.0),
                                            pytest.approx(1.0))

    def test_a_quarter_turn(self):
        z, w = quaternion_from_yaw(math.pi / 2)
        assert z == pytest.approx(math.sqrt(0.5))
        assert w == pytest.approx(math.sqrt(0.5))

    def test_matches_the_reading_taken_off_the_robot(self):
        # tf2_echo reported (0, 0, -0.035, 0.999) and the robot was very
        # nearly facing +x.
        assert math.degrees(yaw_from_quaternion(0, 0, -0.035, 0.999)) == \
            pytest.approx(-4.0, abs=0.5)


class TestDriveVerdict:
    # Nav2's general_goal_checker in nav2_params_robot.yaml.
    TOL = 0.15

    def test_the_2026_10_01_run_is_a_real_success(self):
        # 0.307 m goal, stopped at 0.155: 0.152 short, inside tolerance + slack.
        v = drive_verdict((0.007, -0.010), (0.155, -0.022), (0.307, -0.011), self.TOL)
        assert v == 'ok'

    def test_robot_that_never_moved_is_a_false_success(self):
        assert drive_verdict((0, 0), (0.01, 0), (0.5, 0), self.TOL) == 'false_success'

    def test_full_arrival_is_ok(self):
        assert drive_verdict((0, 0), (0.49, 0.01), (0.5, 0), self.TOL) == 'ok'

    def test_stopped_just_outside_tolerance_plus_slack(self):
        assert drive_verdict((0, 0), (0.29, 0), (0.5, 0), self.TOL) == 'false_success'

    def test_goal_inside_tolerance_cannot_be_judged(self):
        assert drive_verdict((0, 0), (0, 0), (0.18, 0), self.TOL) == 'untestable'

    def test_uses_end_pose_not_distance_travelled(self):
        # Drove 0.4 m but sideways: travelled a lot, still far from the goal.
        assert drive_verdict((0, 0), (0, 0.4), (0.5, 0), self.TOL) == 'false_success'

    def test_the_second_2026_10_01_run_pose_short_lidar_says_arrived(self):
        # Map pose: 0.231 m short of a 0.50 m goal. Lidar ahead: 1.32 -> 0.98.
        v = drive_verdict((-0.015, 0.008), (0.255, -0.018), (0.485, 0.005),
                          self.TOL, lidar_progress=0.34)
        assert v == 'pose_disagrees'

    def test_pose_and_lidar_both_short_is_a_false_success(self):
        v = drive_verdict((0, 0), (0.01, 0), (0.5, 0), self.TOL,
                          lidar_progress=0.02)
        assert v == 'false_success'

    def test_lidar_just_short_of_tolerance_does_not_excuse_the_pose(self):
        # Needs 0.5 - 0.15 - 0.05 = 0.30 of progress.
        v = drive_verdict((0, 0), (0.1, 0), (0.5, 0), self.TOL,
                          lidar_progress=0.29)
        assert v == 'false_success'
