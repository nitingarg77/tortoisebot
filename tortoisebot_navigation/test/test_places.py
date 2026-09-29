"""Tests for the places file and yaw conversion. No robot, no ROS."""

import math

import pytest

from tortoisebot_navigation.places import (
    BadPlaces, parse_places, quaternion_from_yaw, yaw_from_quaternion,
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
