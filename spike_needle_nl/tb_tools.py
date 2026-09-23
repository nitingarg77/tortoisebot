"""TortoiseBot's Nav2 surface as Needle tools, with a read/tune/act risk tier.

WHY THIS ROBOT AND NOT THE COBOT CELL
=====================================
The same spike was run first against the cobot cell (a separate repo:
cobot_ws/spike_needle_nl) and the base model scored 37-40% there, refusing only
2 of 7 out-of-scope requests. Two things were wrong there, and both differ here:

  1. Hardware. The cobot cell runs on a workstation with a 12 GB GPU, where an
     8-29 MB 2-bit model buys nothing. TortoiseBot is a Raspberry Pi. Needle's
     entire reason to exist applies.
  2. Vocabulary. "suspicion threshold", "audit fraction", "reject lane" are
     industrial jargon far outside what Needle 3 was trained on. "go to the
     kitchen", "stop", "turn left" are mobile-robot commands, which is close to
     the DroidCall / mobile-actions distribution it was actually built for.

So this is the fair test of Needle, and the cobot spike was arguably the unfair
one. Whether that matters is what the eval in this directory measures.

PLACES
======
TortoiseBot ships no named locations -- tortoisebot_navigation/scripts/waypoints.py
is a hardcoded 3-pose demo. The places below are coordinates inside the real
explored_map.yaml (174x175 cells at 0.05 m, origin -4.38 -4.38, so roughly -4.3 to
+4.3 in both axes). They are a plausible room layout, not a surveyed one: whoever
runs this on the real robot should re-measure them with `ros2 topic echo /amcl_pose`
while driving it around.

RISK
====
  read -- answers a question, changes nothing.
  tune -- writes a live Nav2 parameter.
  act  -- MOVES THE ROBOT. Never auto-executed, whatever the confidence says.

`stop` is deliberately classed `act` even though it is the safe direction: it
cancels an active goal, which an operator should still see confirmed rather than
have happen silently.
"""

from typing import Literal

import needle
from needle import Field

_backend = None

PLACE = Literal["kitchen", "living_room", "bedroom", "hallway", "charging_dock"]

# Map-frame coordinates, metres: x, y, yaw.
#
# MEASURED against explored_map.yaml, not invented. The first version of this
# table was a plausible-looking house layout and three of its five entries were
# unnavigable: bedroom sat inside an obstacle (0.00 m clearance), kitchen had
# 0.25 m and living_room 0.35 m, which is less than the robot's footprint plus
# costmap inflation. The kitchen goal was accepted and then stalled 0.45 m short
# with no error from Nav2 -- a silent failure that looked like a controller bug
# and was really a bad waypoint.
#
# The figures below are the clearance to the nearest non-free cell, computed from
# the .pgm. Anything under ~0.45 m is not reliably reachable. Note how small the
# genuinely open area of this map is: it is a corridor roughly x in [-0.6, 0.6],
# y in [-0.6, 2.3], so these are five distinct spots in a small space rather than
# rooms of a house. Re-measure on the real robot with `ros2 topic echo /amcl_pose`.
PLACES = {                            # clearance
    "kitchen": (0.52, 2.32, 0.0),     # 1.60 m
    "living_room": (-0.58, 2.32, 1.57),   # 1.60 m
    "bedroom": (-0.58, -0.58, 3.14),  # 1.60 m
    "hallway": (0.15, -0.60, 0.0),    # 2.00 m
    "charging_dock": (0.00, 0.00, 0.0),   # 2.00 m
}

READ, TUNE, ACT = "read", "tune", "act"

RISK = {
    "get_robot_pose": READ,
    "get_navigation_status": READ,
    "list_places": READ,
    "get_nearest_obstacle": READ,
    "set_max_speed": TUNE,
    "go_to_place": ACT,
    "stop": ACT,
    "go_home": ACT,
    "turn": ACT,
    "drive": ACT,
}


def set_backend(backend):
    global _backend
    _backend = backend


def _b():
    if _backend is None:
        raise RuntimeError("tb_tools.set_backend() was never called")
    return _backend


# ---------------------------------------------------------------------- reads
@needle.tool
def get_robot_pose():
    "Where the robot is now on the map."
    return _b().robot_pose()


@needle.tool
def get_navigation_status():
    "Whether the robot is driving to a goal, and which one."
    return _b().navigation_status()


@needle.tool
def list_places():
    "The named places the robot can drive to."
    return _b().list_places()


@needle.tool
def get_nearest_obstacle():
    "Distance and direction to the closest obstacle from the laser scan."
    return _b().nearest_obstacle()


# ----------------------------------------------------------------------- tune
@needle.tool
def set_max_speed(speed: float = Field(ge=0.05, le=0.5)):
    "Set the robot's maximum driving speed in metres per second."
    return _b().set_max_speed(float(speed))


# ------------------------------------------------------------------- actuation
@needle.tool
def go_to_place(place: PLACE):
    "Drive the robot to a named place on the map."
    return _b().go_to_place(place)


@needle.tool
def stop():
    "Stop the robot and cancel whatever goal it is driving to."
    return _b().stop()


@needle.tool
def go_home():
    "Send the robot back to its charging dock."
    return _b().go_to_place("charging_dock")


@needle.tool
def turn(direction: Literal["left", "right"],
         degrees: int = Field(90, ge=1, le=360)):
    "Turn the robot in place by some number of degrees."
    return _b().turn(direction, degrees)


@needle.tool
def drive(direction: Literal["forward", "backward"],
          distance: float = Field(0.5, ge=0.05, le=5.0)):
    "Drive the robot straight by a distance in metres."
    return _b().drive(direction, distance)


ALL = [get_robot_pose, get_navigation_status, list_places, get_nearest_obstacle,
       set_max_speed, go_to_place, stop, go_home, turn, drive]

SYSTEM = ("Voice console for a small two-wheeled robot that drives around a "
          "mapped house. You report where it is and send it to named places. "
          "You cannot open doors, pick things up, or see the camera.")


def _functions_by_name():
    return {schema_name(f): f for f in ALL}


def schema_name(fn):
    spec = getattr(fn, "_needle_tool", None)
    return spec["name"] if spec else fn.__name__


def agent(backend, **kwargs):
    set_backend(backend)
    return needle.Needle(tools=ALL, system=SYSTEM, **kwargs)
