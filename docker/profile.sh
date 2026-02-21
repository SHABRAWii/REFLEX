# docker/profile.sh

# ---- ROS ----
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
  source "/opt/ros/jazzy/setup.bash"
fi

# ---- Workspace (if built) ----
if [ -f "/ws/ros_ws/install/setup.bash" ]; then
  source "/ws/ros_ws/install/setup.bash"
fi

# ---- Container aliases (optional) ----
if [ -f "/ws/scripts/container/aliases.sh" ]; then
  source "/ws/scripts/container/aliases.sh"
fi

# ---- Pretty prompt (always inside container) ----
if [[ $- == *i* ]]; then
  export TERM="${TERM:-xterm-256color}"
  # green user@host, blue working dir
  export PS1='\[\e[1;32m\]\u@\h\[\e[0m\]:\[\e[1;34m\]\w\[\e[0m\]\$ '
fi

export REFLEX_NAME="REFLEX"
export REFLEX_WS="/ws"
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/${ROS_DISTRO}/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}
export LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib:${LD_LIBRARY_PATH}

# ---- REFLEX helper functions (container) ----
reflex_cd() {
  cd "${REFLEX_WS}" || return
}

reflex_ws() {
  # convenience: go to ROS workspace root (where src/, install/, build/ live)
  cd "${REFLEX_WS}/ros_ws" 2>/dev/null || cd "${REFLEX_WS}" || return
}

reflex_src() {
  # Re-source environment (useful after builds)
  if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source "/opt/ros/jazzy/setup.bash"
  fi
  if [ -f "${REFLEX_WS}/ros_ws/install/setup.bash" ]; then
    source "${REFLEX_WS}/ros_ws/install/setup.bash"
  fi
}

reflex_help() {
  cat <<EOF
REFLEX (container) helpers:
  reflex_cd   -> cd to \$REFLEX_WS (${REFLEX_WS})
  reflex_ws   -> cd to ROS workspace (\$REFLEX_WS/ros_ws)
  reflex_src  -> source ROS + workspace overlays
EOF
}

