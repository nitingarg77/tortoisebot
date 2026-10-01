#!/bin/bash
# Overlay tf2 0.25.24 on a ROS 2 Humble install that has 0.25.23.
#
#     ./tf2_overlay.sh install [WS]    # default WS: ~/tb_ws
#     ./tf2_overlay.sh status  [WS]
#     ./tf2_overlay.sh remove  [WS]
#
# Why: tf2 0.25.23 has an ABBA deadlock between Buffer::waitForTransform and
# BufferCore::testTransformableRequests. It froze controller_server's map->odom
# buffer in 6 of 7 Nav2 launches on the robot (MODULE.md, "The cause"). Fixed
# upstream in geometry2 9997e969 and released as 0.25.24; apt still had
# 0.25.23 on 2026-09-30.
#
# What it does: copies ONLY the tf2 package at the 0.25.24 tag into WS/src/tf2
# and builds it, so every process started from WS/install/setup.bash loads the
# fixed libtf2.so ahead of /opt/ros. Nothing else is rebuilt. That is safe
# because the one commit between the two releases changes tf2/src/buffer_core.cpp
# and a test, and no header: the ABI Nav2 was built against is unchanged.
#
# Remove it once apt ships 0.25.24 -- `status` says when. The only proof it is
# in use is a running process's memory map; `status` checks controller_server's.
set -euo pipefail

CMD=${1:-status}
WS=${2:-$HOME/tb_ws}
TAG=0.25.24
FIX=9997e969556f5fe5d32ad498d02b3b3473da70b6
URL=https://github.com/ros2/geometry2.git
LIB=$WS/install/tf2/lib/libtf2.so

apt_version() { dpkg-query -W -f='${Version}' ros-humble-tf2 2>/dev/null | cut -d- -f1; }

status() {
  echo "apt tf2:      $(apt_version)"
  if dpkg --compare-versions "$(apt_version)" ge "$TAG"; then
    echo "  apt now has >= $TAG: the overlay is no longer needed ($0 remove)"
  fi
  if [ -f "$WS/src/tf2/OVERLAY" ]; then
    echo "overlay src:  $(cat "$WS/src/tf2/OVERLAY")"
  else
    echo "overlay src:  none"
  fi
  [ -f "$LIB" ] && echo "overlay lib:  $LIB" || echo "overlay lib:  not built"
  local pid
  pid=$(pgrep -f '/opt/ros/humble/lib/nav2_controller/controller_server' | head -1 || true)
  if [ -n "$pid" ]; then
    echo "controller_server $pid maps: $(grep -o '/[^ ]*libtf2\.so[^ ]*' /proc/$pid/maps | sort -u | tr '\n' ' ')"
  else
    echo "controller_server: not running"
  fi
}

case $CMD in
  install)
    if dpkg --compare-versions "$(apt_version)" ge "$TAG"; then
      echo "apt already has tf2 $(apt_version); no overlay needed"; exit 0
    fi
    tmp=$(mktemp -d)
    trap 'rm -rf "$tmp"' EXIT
    git clone -q --depth 1 -b "$TAG" "$URL" "$tmp/geometry2"
    # The tag must contain the fix; refuse anything else.
    got=$(grep -m1 -o '<version>[^<]*' "$tmp/geometry2/tf2/package.xml" | cut -d'>' -f2)
    [ "$got" = "$TAG" ] || { echo "tag $TAG has tf2 version $got"; exit 1; }
    grep -q 'ABBA deadlock' "$tmp/geometry2/tf2/CHANGELOG.rst" \
      || { echo "tag $TAG changelog does not mention the ABBA fix"; exit 1; }
    rm -rf "$WS/src/tf2"
    cp -r "$tmp/geometry2/tf2" "$WS/src/tf2"
    echo "geometry2 $TAG (contains $FIX), tf2 only, installed $(date -I)" > "$WS/src/tf2/OVERLAY"
    set +u; source /opt/ros/humble/setup.bash; set -u
    # --allow-overriding: deliberate, and ABI-safe for the reason above.
    (cd "$WS" && colcon build --packages-select tf2 --allow-overriding tf2 \
        --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release)
    status
    ;;
  remove)
    rm -rf "$WS/src/tf2" "$WS/build/tf2" "$WS/install/tf2"
    echo "removed; restart any ROS processes started with the overlay"
    status
    ;;
  status) status ;;
  *) sed -n '2,6p' "$0"; exit 2 ;;
esac
