#!/usr/bin/env bash
# ROS 2 Jazzy environment loader (must be sourced)

# Guard: if not sourced, warn and exit cleanly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "[ROS ERROR] Run this with: source ${BASH_SOURCE[0]}"
  exit 1
fi

echo "[ROS] Loading ROS 2 Jazzy environment..."

# 1) Source global ROS install
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
else
  echo "[ROS ERROR] /opt/ros/jazzy/setup.bash not found!"
  return 1
fi

# 2) Overlay workspace (adjust if you use different path)
WS="$HOME/ros2_ws"
if [ -f "$WS/install/setup.bash" ]; then
  echo "[ROS] Overlaying workspace: $WS"
  source "$WS/install/setup.bash"
  cd "$WS" || return 1
else
  echo "[ROS] Workspace overlay not found at: $WS/install/setup.bash"
  echo "[ROS] (That’s ok if you haven’t built it yet.)"
fi

# 3) Defaults
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset ROS_AUTOMATIC_DISCOVERY_RANGE
unset CYCLONEDDS_URI
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=192.168.186.3:11811
export ROS_SUPER_CLIENT=True

# Optional: visual cue
export PS1="(ROS2) $PS1"

echo "[ROS] Ready ✔"
