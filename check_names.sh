#!/bin/bash
# Catch undefined names in the node scripts, which nothing else can.
#
#     ./check_names.sh
#
# The nodes import `rclpy`, `board`, `RPi.GPIO` and `cv2` at module scope, so
# they cannot be imported off-robot and the pytest suites never touch them.
# That left a whole class of bug reaching the robot: splitting the BNO055 gates
# into sample_checks.py left imu_node.py referring to a local `norm` that no
# longer existed, every sample died on it, and /imu published nothing for a
# day. Cartographer lost its gravity alignment and scan_fusion lost its yaw
# correction, and the only sign was one throttled line in a launch log.
#
# pyflakes parses rather than imports, so it sees that without a robot.
#
# Deliberately ONLY undefined names. Unused imports are real but cosmetic, and
# the vendored launch files are full of them; a check that shouts about those
# gets ignored, and then it catches nothing.
set -uo pipefail
cd "$(dirname "$0")"

files=$(find tortoisebot_* -name '*.py' \
  -not -path '*/build/*' -not -path '*/install/*' -not -path '*__pycache__*' \
  -not -name 'teleop_twist_keyboard.py' | sort)

# teleop_twist_keyboard.py is vendored and guards a Windows-only `msvcrt`
# behind a platform check that pyflakes cannot follow.

hits=$(python3 -m pyflakes $files 2>/dev/null | grep 'undefined name' || true)
if [ -n "$hits" ]; then
  echo "FAILED  undefined names (these break the node at runtime):"
  echo "$hits" | sed 's/^/  /'
  exit 1
fi
echo "ok  no undefined names in $(echo "$files" | wc -l) files"
