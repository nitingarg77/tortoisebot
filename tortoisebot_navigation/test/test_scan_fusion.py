"""Tests for folding the camera's floor scan into the lidar's. No robot, no ROS.

The cases that matter are the ones this robot has already paid for: a lidar
that cannot see a white panel, a no-return encoded as 0.0 rather than inf, and
a camera fan 200 ms out of date while the robot turns.
"""

import math

import pytest

from tortoisebot_navigation.scan_fusion import (
    INF, LIDAR_DX, Fan, YawHistory, camera_onto_lidar, fuse, is_cluttered,
    project, wrap,
)

# The YDLidar on this robot, from ydlidar_ros2_driver/params/ydlidar.yaml:
# 260 beams over the full circle, 0.1-12 m, and a no-return published as 0.0
# because invalid_range_is_inf is false.
COUNT = 260
INCREMENT = 2 * math.pi / COUNT
NO_RETURN = 0.0

# floor_scan.py's defaults: 60 beams over 53.5 deg, 0.30-3.0 m, inf elsewhere.
CAM_BEAMS = 60
CAM_HFOV = math.radians(53.5)


def lidar(default=5.0, stamp=10.0, **at_deg):
    ranges = [default] * COUNT
    f = Fan(ranges, -math.pi, INCREMENT, 0.1, 12.0, stamp, 'lidar')
    for key, value in at_deg.items():
        deg = float(key.replace('n', '-').replace('d', ''))
        f.ranges[f.index_for(math.radians(deg))] = value
    return f


def camera(default=INF, stamp=10.0, **at_deg):
    ranges = [default] * CAM_BEAMS
    increment = CAM_HFOV / (CAM_BEAMS - 1)
    f = Fan(ranges, -CAM_HFOV / 2, increment, 0.30, 3.0, stamp, 'base_link')
    for key, value in at_deg.items():
        deg = float(key.replace('n', '-').replace('d', ''))
        f.ranges[f.index_for(math.radians(deg))] = value
    return f


def at(ranges, deg):
    """The fused range at a bearing, read back off a full-circle scan."""
    i = int(round(wrap(math.radians(deg) + math.pi) / INCREMENT))
    return ranges[i % COUNT]


class TestValidity:
    """What counts as a reading. Every later test rests on this."""

    def test_a_zero_is_a_no_return_not_a_touch(self):
        # The whole trap: this driver publishes 0.0 for "nothing came back".
        # Read as a distance it means an obstacle against the robot.
        assert not lidar().valid(NO_RETURN)

    def test_nan_is_not_a_reading(self):
        assert not lidar().valid(float('nan'))

    def test_inf_is_not_a_reading(self):
        assert not lidar().valid(INF)
        assert not camera().valid(INF)

    def test_a_real_range_is(self):
        assert lidar().valid(1.2)
        assert camera().valid(1.2)

    def test_the_camera_rejects_what_is_beyond_its_own_limits(self):
        # 5 m is a fine lidar range and a meaningless camera one.
        assert lidar().valid(5.0)
        assert not camera().valid(5.0)


class TestIndexing:

    def test_a_bearing_round_trips(self):
        f = lidar()
        for deg in (-179, -90, -1, 0, 1, 90, 179):
            i = f.index_for(math.radians(deg))
            assert math.degrees(wrap(f.bearing(i))) == \
                pytest.approx(deg, abs=math.degrees(INCREMENT))

    def test_a_full_circle_scan_wraps_at_the_seam(self):
        # 181 degrees is 179 the other way, not off the end of the array.
        f = lidar()
        assert f.wraps
        assert f.index_for(math.radians(181)) == f.index_for(math.radians(-179))

    def test_the_camera_fan_does_not_wrap(self):
        f = camera()
        assert not f.wraps
        assert f.index_for(math.radians(90)) is None


class TestProjection:
    """33 mm of lidar offset, which is small and not negligible."""

    def test_the_offset_is_not_rounded_away(self):
        # The design claim, at the camera's near edge: 1.8 deg of bearing and
        # 3 cm of range. Either is more than a costmap cell at this resolution.
        r, bearing = project(0.45, math.radians(26.75))
        assert r - 0.45 == pytest.approx(0.030, abs=0.002)
        assert math.degrees(bearing) - 26.75 == pytest.approx(-1.8, abs=0.1)

    def test_straight_ahead_is_a_pure_range_shift(self):
        r, bearing = project(1.0, 0.0)
        assert r == pytest.approx(1.0 - LIDAR_DX)
        assert bearing == pytest.approx(0.0)

    def test_the_shift_shrinks_with_distance(self):
        near, _ = project(0.45, math.radians(26.75))
        far, _ = project(3.0, math.radians(26.75))
        assert (near - 0.45) > (far - 3.0)


class TestFusingInTheCamera:

    def test_the_white_panel_case(self):
        # The reason this node exists. The lidar returns nothing ahead -- 0.0,
        # not inf -- and the camera sees the panel at 1.2 m.
        fused, note = fuse(lidar(5.0, d0=NO_RETURN), camera(d0=1.2))
        assert note is None
        assert at(fused, 0) == pytest.approx(1.2, abs=0.05)

    def test_min_would_have_got_this_exactly_backwards(self):
        # min(0.0, 1.2) is 0.0, which is under range_min, so laser_geometry
        # drops the beam and nothing is marked at all -- at the one bearing
        # where the camera is the only sensor that can see.
        fused, _ = fuse(lidar(5.0, d0=NO_RETURN), camera(d0=1.2))
        assert at(fused, 0) != NO_RETURN

    def test_the_camera_wins_when_it_is_nearer(self):
        fused, _ = fuse(lidar(5.0), camera(d0=1.2))
        assert at(fused, 0) == pytest.approx(1.2, abs=0.05)

    def test_the_camera_cannot_push_an_obstacle_further_away(self):
        # The structural safety property that makes it safe to let this topic
        # clear: a camera that misses something cannot un-see the lidar's hit.
        fused, _ = fuse(lidar(5.0, d0=0.8), camera(d0=2.5))
        assert at(fused, 0) == pytest.approx(0.8)

    def test_a_beam_neither_sensor_saw_keeps_the_lidar_encoding(self):
        # Untouched, so the costmap behaves there exactly as it does today.
        fused, _ = fuse(lidar(5.0, d0=NO_RETURN), camera())
        assert at(fused, 0) == NO_RETURN

    def test_bearings_outside_the_fan_are_untouched(self):
        fused, _ = fuse(lidar(5.0), camera(d0=1.2))
        assert at(fused, 180) == pytest.approx(5.0)
        assert at(fused, 90) == pytest.approx(5.0)

    def test_the_output_is_the_same_length_as_the_lidar_scan(self):
        fused, _ = fuse(lidar(), camera(d0=1.2))
        assert len(fused) == COUNT

    def test_the_input_scan_is_not_modified(self):
        raw = lidar(5.0)
        before = list(raw.ranges)
        fuse(raw, camera(d0=1.2))
        assert raw.ranges == before

    def test_an_off_centre_obstacle_lands_off_centre(self):
        fused, _ = fuse(lidar(5.0), camera(d20=1.0))
        assert at(fused, 20) < 2.0
        assert at(fused, -20) == pytest.approx(5.0)

    def test_the_nearest_of_several_camera_beams_wins_a_lidar_beam(self):
        # The fan is denser than the lidar, so beams collide. For an obstacle
        # the nearest thing in the beam is what the robot hits.
        cam = camera()
        for i in range(CAM_BEAMS):
            cam.ranges[i] = 2.0
        cam.ranges[CAM_BEAMS // 2] = 0.9
        mapped = camera_onto_lidar(cam, lidar())
        assert min(mapped.values()) == pytest.approx(0.9, abs=0.05)

    def test_a_reading_nearer_than_the_lidar_minimum_is_kept_not_dropped(self):
        # Clamped up to range_min rather than discarded: a reading too near to
        # represent is the one case where throwing it away is the dangerous
        # choice. The door frame that wedged the robot sat at 0.10 m.
        cam = camera(d0=0.30)
        mapped = camera_onto_lidar(cam, lidar())
        assert all(r >= 0.1 for r in mapped.values())


class TestDegradation:
    """Every one of these must still publish. /scan_fused is the costmaps'
    only source, and silence blinds Nav2 completely."""

    def test_no_camera_at_all_is_lidar_only(self):
        fused, note = fuse(lidar(5.0), None)
        assert fused == lidar(5.0).ranges
        assert 'no camera' in note

    def test_a_stale_camera_scan_is_ignored(self):
        fused, note = fuse(lidar(5.0, stamp=10.0),
                           camera(stamp=9.5, d0=1.2))
        assert at(fused, 0) == pytest.approx(5.0)
        assert 'stale' in note

    def test_a_fresh_one_is_not(self):
        fused, note = fuse(lidar(5.0, stamp=10.0),
                           camera(stamp=9.85, d0=1.2))
        assert note is None
        assert at(fused, 0) < 2.0

    def test_a_camera_scan_from_the_future_is_ignored(self):
        # Clocks disagreeing, not the future arriving. Using it would put the
        # obstacle at the wrong bearing.
        fused, note = fuse(lidar(5.0, stamp=10.0),
                           camera(stamp=11.0, d0=1.2))
        assert at(fused, 0) == pytest.approx(5.0)
        assert 'ahead' in note

    def test_a_fan_that_reads_as_a_wall_is_rejected(self):
        # A lens cover, a hand, a blown exposure. Trusting it stops the robot
        # dead in open space.
        blind = camera()
        blind.ranges = [0.35] * CAM_BEAMS
        fused, note = fuse(lidar(5.0), blind)
        assert at(fused, 0) == pytest.approx(5.0)
        assert 'wall' in note

    def test_a_genuine_narrow_obstacle_is_not_mistaken_for_that(self):
        # The counterweight to the test above: a door frame lights a few beams
        # and must survive. The check is against the whole fan, not the valid
        # beams, precisely so this passes.
        cam = camera()
        for i in range(CAM_BEAMS // 2 - 3, CAM_BEAMS // 2 + 4):
            cam.ranges[i] = 0.5
        assert not is_cluttered(cam)
        fused, note = fuse(lidar(5.0), cam)
        assert note is None
        assert at(fused, 0) < 1.0

    def test_an_empty_fan_counts_as_cluttered_not_as_clear(self):
        assert is_cluttered(Fan([], 0.0, 0.0))

    def test_nan_in_either_input_is_survived(self):
        lid = lidar(5.0)
        lid.ranges[0] = float('nan')
        cam = camera(d0=1.2)
        cam.ranges[0] = float('nan')
        fused, note = fuse(lid, cam)
        assert note is None
        assert all(r == r for r in fused[1:])


class TestRotationCorrection:
    """A 200 ms old fan smears 13.8 deg at 1.2 rad/s. Translation over the same
    interval costs 2.6 cm, under one costmap cell, and is left alone."""

    def test_a_turn_moves_the_obstacle_to_where_it_now_is(self):
        # The camera saw it dead ahead; by the lidar's stamp the robot has
        # turned 20 deg left, so it now lies 20 deg to the right.
        fused, note = fuse(lidar(5.0, stamp=10.0), camera(stamp=9.8, d0=1.2),
                           yaw_delta=math.radians(20))
        assert note is None
        assert at(fused, -20) < 2.0
        assert at(fused, 0) == pytest.approx(5.0)

    def test_turning_the_other_way_moves_it_the_other_way(self):
        fused, _ = fuse(lidar(5.0, stamp=10.0), camera(stamp=9.8, d0=1.2),
                        yaw_delta=math.radians(-20))
        assert at(fused, 20) < 2.0

    def test_no_turn_leaves_it_where_it_was(self):
        fused, _ = fuse(lidar(5.0, stamp=10.0), camera(stamp=9.8, d0=1.2),
                        yaw_delta=0.0)
        assert at(fused, 0) < 2.0

    def test_unknown_yaw_is_not_the_same_as_no_turn(self):
        # None must not silently become 0.0. Over 200 ms the robot could have
        # turned 13.8 deg, which puts the obstacle in the wrong place.
        fused, note = fuse(lidar(5.0, stamp=10.0), camera(stamp=9.8, d0=1.2),
                           yaw_delta=None)
        assert at(fused, 0) == pytest.approx(5.0)
        assert 'yaw' in note

    def test_unknown_yaw_is_tolerated_while_the_fan_is_very_fresh(self):
        # Inside 50 ms even 1.2 rad/s smears less than the lidar's own 1.385
        # deg resolution, so refusing the fan would lose more than it saves.
        fused, note = fuse(lidar(5.0, stamp=10.0), camera(stamp=9.98, d0=1.2),
                           yaw_delta=None)
        assert note is None
        assert at(fused, 0) < 2.0


class TestYawHistory:

    def test_interpolates_between_samples(self):
        h = YawHistory()
        h.add(1.0, 0.0)
        h.add(2.0, 1.0)
        assert h.yaw_at(1.5) == pytest.approx(0.5)
        assert h.delta(1.0, 1.5) == pytest.approx(0.5)

    def test_will_not_extrapolate(self):
        # A confident wrong bearing is worse than no correction, so outside the
        # buffer it says it does not know.
        h = YawHistory()
        h.add(1.0, 0.0)
        h.add(2.0, 1.0)
        assert h.yaw_at(2.5) is None
        assert h.delta(1.0, 2.5) is None

    def test_one_sample_is_not_enough(self):
        h = YawHistory()
        h.add(1.0, 0.0)
        assert h.yaw_at(1.0) is None

    def test_takes_the_short_way_round_the_seam(self):
        # 179 deg to -179 deg is a 2 deg turn, not a 358 deg one. Getting this
        # wrong would swing the whole fan on any pass through the seam.
        h = YawHistory()
        h.add(1.0, math.radians(179))
        h.add(2.0, math.radians(-179))
        assert math.degrees(h.delta(1.0, 2.0)) == pytest.approx(2.0, abs=0.01)

    def test_repeated_and_out_of_order_stamps_are_dropped(self):
        # TF and sensor callbacks both re-deliver stamps.
        h = YawHistory()
        h.add(2.0, 0.0)
        h.add(2.0, 9.0)
        h.add(1.0, 9.0)
        assert len(h.samples) == 1

    def test_old_samples_leave_the_window(self):
        h = YawHistory(window=2.0)
        for i in range(100):
            h.add(i / 10.0, 0.0)
        assert h.samples[-1][0] - h.samples[0][0] <= 2.0 + 1e-9

    def test_the_real_rates_line_up(self):
        # The IMU runs at 49 Hz and a fusion cycle spans the lidar's 11.7 Hz
        # against the camera's 5 Hz, so both stamps land inside the buffer and
        # the correction is exact rather than a fallback.
        h = YawHistory()
        for i in range(100):
            h.add(i / 49.0, i * math.radians(1.0))
        cam_stamp, lidar_stamp = 1.0, 1.2
        assert h.delta(cam_stamp, lidar_stamp) is not None


class TestFromMsg:
    """The one place this module touches a ROS shape. Duck-typed, so a stub
    proves the unpacking without sensor_msgs installed."""

    def test_unpacks_a_laserscan_shaped_object(self):
        stamp = type('S', (), {'sec': 12, 'nanosec': 500000000})()
        header = type('H', (), {'stamp': stamp, 'frame_id': 'lidar'})()
        msg = type('M', (), {'header': header, 'ranges': [1.0, 2.0],
                             'angle_min': -1.0, 'angle_increment': 0.5,
                             'range_min': 0.1, 'range_max': 12.0})()
        f = Fan.from_msg(msg)
        assert f.stamp == pytest.approx(12.5)
        assert f.frame_id == 'lidar'
        assert f.ranges == [1.0, 2.0]
