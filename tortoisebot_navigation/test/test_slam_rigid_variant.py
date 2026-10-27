"""Pin the T-001 rigid-frame Cartographer variant. No robot, no ROS.

These cannot show the map frame stops jumping (that is a measurement on the
robot, docs/tasks/T-001-rigid-frame-slam.md). They guard two things a desk can:
the variant differs from slam_real.lua in exactly the one setting, and the
launch file's config selection still picks the files it should, with the
default unchanged.
"""

import importlib.util
import os
import re
import sys
from unittest import mock

import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(PKG)
SLAM = os.path.join(REPO, 'tortoisebot_slam')
CONFIG = os.path.join(SLAM, 'config')
LAUNCH = os.path.join(SLAM, 'launch', 'cartographer.launch.py')

# The ROS libraries the launch file imports at module scope. The functions
# under test use none of them, so stand-ins are enough.
STUBBED = ['launch', 'launch.actions', 'launch.conditions',
           'launch.substitutions', 'launch_ros', 'launch_ros.actions',
           'ament_index_python', 'ament_index_python.packages']


def code_lines(path):
    """Non-blank, non-comment Lua lines."""
    with open(path) as f:
        return [ln.rstrip() for ln in f
                if ln.strip() and not ln.strip().startswith('--')]


@pytest.fixture(scope='module')
def launch_module():
    stubs = {name: mock.MagicMock() for name in STUBBED}
    with mock.patch.dict(sys.modules, stubs):
        spec = importlib.util.spec_from_file_location('cartographer_launch',
                                                      LAUNCH)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    return mod


def test_variant_differs_from_slam_real_only_in_optimize_every_n_nodes():
    base = code_lines(os.path.join(CONFIG, 'slam_real.lua'))
    rigid = code_lines(os.path.join(CONFIG, 'slam_real_rigid.lua'))
    assert len(base) == len(rigid)
    diff = [(a, b) for a, b in zip(base, rigid) if a != b]
    assert len(diff) == 1, diff
    old, new = diff[0]
    assert re.fullmatch(r'POSE_GRAPH\.optimize_every_n_nodes = 90', old), old
    assert re.fullmatch(r'POSE_GRAPH\.optimize_every_n_nodes = 0', new), new


def test_default_launch_is_unchanged(launch_module):
    # Real robot SLAM, the rigid flag not set: the file it has always used.
    assert launch_module.lua_for(False, False, False) == 'slam_real.lua'


def test_rigid_flag_selects_the_variant_on_the_real_robot(launch_module):
    assert launch_module.lua_for(False, False, True) == 'slam_real_rigid.lua'


@pytest.mark.parametrize('rigid', [False, True])
def test_odom_only_and_sim_ignore_the_rigid_flag(launch_module, rigid):
    assert launch_module.lua_for(True, False, rigid) == 'slam_real_mapbased.lua'
    assert launch_module.lua_for(True, True, rigid) == 'slam_real_mapbased.lua'
    assert launch_module.lua_for(False, True, rigid) == 'slam_sim.lua'


@pytest.mark.parametrize('args', [(a, b, c) for a in (False, True)
                                  for b in (False, True)
                                  for c in (False, True)])
def test_every_selectable_config_exists(launch_module, args):
    name = launch_module.lua_for(*args)
    assert os.path.isfile(os.path.join(CONFIG, name)), name
