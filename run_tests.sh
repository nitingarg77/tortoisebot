#!/bin/bash
# Every desk-testable check in this repository. No robot, no ROS, no hardware.
#
#     ./run_tests.sh
#
# Two kinds: the pytest suites over the pure modules, and check_names.sh over
# the node scripts, which cannot be imported off-robot and so are covered by
# nothing else.
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
printf '%-26s ' "undefined names"
if out=$(./check_names.sh 2>&1); then
  echo "${out##*$'\n'}"
else
  failed=1
  echo "FAILED"
  echo "$out"
fi
exit $failed
