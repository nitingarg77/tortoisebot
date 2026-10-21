"""Pin the A-005 precision-goal parameters. No robot, no ROS.

These are config tests: they cannot show the robot reaches 5 cm, only that the
numbers that must agree with each other still do. Each assertion guards a
mismatch that was found in nav2_params_robot.yaml (docs/test-reports/
auto-mapping-run3.md, cause A3) or a trap spelled out in a comment there.
"""

import glob
import math
import os
import re

import pytest
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(PKG)
CONFIG = os.path.join(PKG, 'config')

# Worst pose error seen on the robot: the map pose moved 0.12 m with no motion
# commanded (BUILD_LOG.md:314-318).
WORST_POSE_ERROR_M = 0.12
GOAL_TOLERANCE_M = 0.05


def load(path):
    with open(path) as f:
        return yaml.safe_load(f)


@pytest.fixture(scope='module')
def nav():
    return load(os.path.join(CONFIG, 'nav2_params_robot.yaml'))


@pytest.fixture(scope='module')
def explore():
    return load(os.path.join(CONFIG, 'explore_params_robot.yaml'))


def ctrl(nav):
    return nav['controller_server']['ros__parameters']


def occupancy_value(cost):
    """Nav2 Costmap2DPublisher's cost -> OccupancyGrid translation."""
    if cost == 0:
        return 0
    if cost == 253:
        return 99
    if cost == 254:
        return 100
    return 1 + (97 * (cost - 1)) // 251


def clearance_for_threshold(threshold, radius, scaling):
    """Smallest distance from a lethal cell at which the grid value stays below
    `threshold`, i.e. at which the explorer would still accept a goal.
    Inflation cost is 252 * exp(-scaling * (d - radius)), truncated to int."""
    d = radius
    while d < 2.0:
        cost = int(252 * math.exp(-scaling * (d - radius)))
        if occupancy_value(cost) < threshold:
            return d
        d += 0.005
    raise AssertionError('threshold never reached')


class TestEveryChangedFileParses:

    @pytest.mark.parametrize('path', sorted(glob.glob(
        os.path.join(CONFIG, '*.yaml'))))
    def test_yaml_parses(self, path):
        assert isinstance(load(path), dict)

    def test_there_are_configs_to_check(self):
        assert len(glob.glob(os.path.join(CONFIG, '*.yaml'))) >= 3


class TestGoalChecking:

    def test_goal_checker_is_5cm(self, nav):
        gc = ctrl(nav)['general_goal_checker']
        assert gc['xy_goal_tolerance'] <= GOAL_TOLERANCE_M
        assert gc['plugin'] == 'nav2_controller::SimpleGoalChecker'

    def test_dwb_tolerance_equals_goal_checker(self, nav):
        # RotateToGoal spins the robot in place once inside this radius, so a
        # wider value stops translation early; a narrower one never slows.
        c = ctrl(nav)
        assert (c['FollowPath']['xy_goal_tolerance']
                == c['general_goal_checker']['xy_goal_tolerance'])

    def test_stopped_velocity_is_reachable_and_small(self, nav):
        # Was 0.25 against max_vel_x 0.15: always "stopped", penalty dead.
        c = ctrl(nav)['FollowPath']
        assert 0.0 < c['trans_stopped_velocity'] <= 0.03
        assert c['trans_stopped_velocity'] < c['max_vel_x']

    def test_planner_cannot_stop_far_short_of_goal(self, nav):
        tol = nav['planner_server']['ros__parameters']['GridBased']['tolerance']
        assert tol <= 0.1


class TestSlowApproach:

    def test_speed_cap_is_slowed_and_mirrored(self, nav, explore):
        c = ctrl(nav)['FollowPath']
        vs = nav['velocity_smoother']['ros__parameters']
        vmax = explore['frontier_explorer']['ros__parameters'][
            'max_linear_speed_vmax']
        assert c['max_vel_x'] <= 0.10
        assert c['max_speed_xy'] <= c['max_vel_x']
        assert vs['max_velocity'][0] == c['max_vel_x']
        assert vs['min_velocity'][0] == -c['max_vel_x']
        assert vmax == c['max_vel_x']


class TestProgressChecker:

    def test_turning_counts_as_progress(self, nav):
        pc = ctrl(nav)['progress_checker']
        assert pc['plugin'] == 'nav2_controller::PoseProgressChecker'
        assert 0.0 < pc['required_movement_angle'] <= 1.0

    def test_slow_creep_satisfies_it(self, nav):
        # At the measured 0.04 m/s average, the required distance must be
        # covered well inside the allowance or a slow approach is aborted.
        pc = ctrl(nav)['progress_checker']
        seconds = pc['required_movement_radius'] / 0.04
        assert seconds <= pc['movement_time_allowance'] / 2

    def test_progress_radius_is_not_the_old_one(self, nav):
        assert ctrl(nav)['progress_checker']['required_movement_radius'] <= 0.10


class TestFrontierGoalsStayClearOfWalls:

    def costmap(self, nav, which):
        return nav[which][which]['ros__parameters']

    @pytest.mark.parametrize('which', ['global_costmap', 'local_costmap'])
    def test_goal_clearance_covers_tolerance_and_pose_error(
            self, nav, explore, which):
        cm = self.costmap(nav, which)
        radius = cm['robot_radius']
        scaling = cm['inflation_layer']['cost_scaling_factor']
        threshold = explore['frontier_explorer']['ros__parameters'][
            'occ_threshold']
        needed = radius + GOAL_TOLERANCE_M + WORST_POSE_ERROR_M
        assert clearance_for_threshold(threshold, radius, scaling) >= needed

    def test_clearance_model_matches_the_default(self):
        # Sanity of the model itself: the package default 65 on the robot's
        # costmap params gives the ~0.24 m quoted in explore_params_robot.yaml.
        d = clearance_for_threshold(65, 0.1, 3.0)
        assert 0.22 <= d <= 0.26


class TestMapbasedOdometerWindow:

    def window(self, name):
        with open(os.path.join(REPO, 'tortoisebot_slam', 'config', name)) as f:
            m = re.search(
                r'^TRAJECTORY_BUILDER_2D\.real_time_correlative_scan_matcher'
                r'\.linear_search_window\s*=\s*([0-9.]+)', f.read(), re.M)
        assert m, name
        return float(m.group(1))

    def test_mapbased_window_not_narrower_than_slam_real(self):
        # 0.1 m lost the match on a late scan and ran odom away (2026-09-25).
        assert self.window('slam_real_mapbased.lua') >= self.window(
            'slam_real.lua')
