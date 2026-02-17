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
