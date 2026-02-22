# docker/profile.sh  (shim only)

# Source repo-mounted profile if available (preferred)
if [[ -f /ws/scripts/container/reflex_profile.sh ]]; then
  # shellcheck disable=SC1091
  source /ws/scripts/container/reflex_profile.sh
  return 0 2>/dev/null || true
fi

# Fallback: source ROS only (minimal)
if [[ -f "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
fi

# Fallback overlay if built
if [[ -f "/ws/ros_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "/ws/ros_ws/install/setup.bash"
fi