#!/bin/bash
# Every desk-testable suite in this repository. No robot, no ROS, no hardware.
#
#     ./run_tests.sh
#
# Runs from each package directory because the ROS package and the Python
# package inside it share a name; the conftest.py in each test/ handles the
# rest. CI runs exactly this set.
set -uo pipefail
cd "$(dirname "$0")"
failed=0
for pkg in tortoisebot_firmware tortoisebot_navigation tortoisebot_imu; do
  printf '%-26s ' "$pkg"
  out=$( (cd "$pkg" && python3 -m pytest test -q 2>&1) )
  if [ $? -eq 0 ]; then
    echo "$out" | tail -1
  else
    failed=1
    echo "FAILED"
    echo "$out" | tail -20
  fi
done
exit $failed
