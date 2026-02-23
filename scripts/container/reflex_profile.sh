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

# ============================================================
# REFLEX Container Profile
# - ROS + workspace sourcing
# - Convenience navigation/build helpers
# - PID tuner staging + apply (pause -> set params -> reset_pid -> apply_pose -> resume)
# - Tuning presets + inspection
#
# Notes:
#   - Disable colors: export NO_COLOR=1
#   - History persistence is configured in reflex_bashrc (recommended)
# ============================================================

# ----------------------------
# [0] Defaults / Environment
# ----------------------------
export REFLEX_NAME="${REFLEX_NAME:-REFLEX}"
export REFLEX_WS="${REFLEX_WS:-/ws}"
export REFLEX_TUNER_NODE="${REFLEX_TUNER_NODE:-/reflex_pid_tuner}"

# Gazebo / ROS library paths (safe if empty)
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/${ROS_DISTRO:-jazzy}/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="/opt/ros/${ROS_DISTRO:-jazzy}/lib:${LD_LIBRARY_PATH:-}"
export GZ_SIM_RESOURCE_PATH=/ws/ros_ws/src/REFLEX/reflex_gazebo/models:$GZ_SIM_RESOURCE_PATH

# ----------------------------
# [1] Color + Logging Helpers
# ----------------------------
# IMPORTANT:
#   Don't use "-t 1" inside $(...) command substitution because stdout becomes a pipe.
#   So we compute a stable flag once in the *parent* shell.

# 1) Decide once
__REFLEX_COLOR=0
if [[ -z "${NO_COLOR:-}" && -t 1 && "${TERM:-}" != "dumb" ]]; then
  __REFLEX_COLOR=1
fi

# 2) Color wrapper
__reflex_c() {
  # usage: __reflex_c <ansi_code> <text...>
  local code="$1"; shift
  if [[ "${__REFLEX_COLOR}" -eq 1 ]]; then
    printf "\033[%sm%s\033[0m" "$code" "$*"
  else
    printf "%s" "$*"
  fi
}

# 3) Tag + log helpers
_reflex_tag()  { __reflex_c "36" "[REFLEX]"; }                 # cyan
_reflex_info() { echo "$(_reflex_tag) $(__reflex_c "0"  "$*")"; }
_reflex_ok()   { echo "$(_reflex_tag) $(__reflex_c "32" "$*")"; }  # green
_reflex_warn() { echo "$(_reflex_tag) $(__reflex_c "33" "$*")"; }  # yellow
_reflex_err()  { echo "$(_reflex_tag) $(__reflex_c "31" "$*")" >&2; } # red
_reflex_dbg()  { [[ -n "${REFLEX_DEBUG:-}" ]] && echo "$(_reflex_tag) $(__reflex_c "35" "$*")"; } # magenta

# ----------------------------
# [2] ROS + Workspace Sourcing
# ----------------------------
reflex_src() {
  local ros_setup="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
  local ws_setup="${REFLEX_WS}/ros_ws/install/setup.bash"

  if [[ -f "$ros_setup" ]]; then
    # shellcheck disable=SC1090
    source "$ros_setup"
    _reflex_ok "Sourced ROS: $ros_setup"
  else
    _reflex_warn "ROS setup not found: $ros_setup"
  fi

  if [[ -f "$ws_setup" ]]; then
    # shellcheck disable=SC1090
    source "$ws_setup"
    _reflex_ok "Sourced overlay: $ws_setup"
  else
    _reflex_warn "Overlay not built yet: $ws_setup"
    _reflex_info "Build it with: reflex_build"
  fi
}

# Source automatically if interactive (optional)
if [[ $- == *i* ]]; then
  reflex_src >/dev/null 2>&1 || true
fi

# ----------------------------
# [3] Navigation + Build Helpers
# ----------------------------
reflex_cd() {
  if cd "${REFLEX_WS}" 2>/dev/null; then
    _reflex_ok "cd -> ${REFLEX_WS}"
  else
    _reflex_err "Failed: REFLEX_WS=${REFLEX_WS} (does it exist?)"
    return 1
  fi
}

reflex_ws() {
  if cd "${REFLEX_WS}/ros_ws" 2>/dev/null; then
    _reflex_ok "cd -> ${REFLEX_WS}/ros_ws"
  elif cd "${REFLEX_WS}" 2>/dev/null; then
    _reflex_warn "ros_ws not found, cd -> ${REFLEX_WS}"
  else
    _reflex_err "Failed to cd (REFLEX_WS=${REFLEX_WS})"
    return 1
  fi
}

reflex_build() {
  if ! cd "${REFLEX_WS}/ros_ws" 2>/dev/null; then
    _reflex_err "Workspace not found: ${REFLEX_WS}/ros_ws"
    return 1
  fi

  _reflex_info "Building (colcon build --symlink-install) ..."
  if colcon build --symlink-install; then
    _reflex_ok "Build done."
    reflex_src
  else
    _reflex_err "Build failed."
    return 1
  fi
}

# ----------------------------
# [4] PID Tuner: Staging Store
# ----------------------------
declare -gA __REFLEX_STAGED_PARAMS=()

reflex_set() {
  # Stage a single param: reflex_set t_kp -90.0
  local key="${1:-}"
  local val="${2:-}"
  if [[ -z "$key" || -z "$val" ]]; then
    _reflex_err "Usage: reflex_set <param_name> <value>"
    return 2
  fi
  __REFLEX_STAGED_PARAMS["$key"]="$val"
  _reflex_warn "Staged: ${REFLEX_TUNER_NODE}  ${key}=${val}"
}

reflex_show() {
  # Show staged params (sorted)
  if [[ ${#__REFLEX_STAGED_PARAMS[@]} -eq 0 ]]; then
    _reflex_info "No staged params."
    return 0
  fi

  _reflex_info "Staged params for ${REFLEX_TUNER_NODE}:"
  local k
  for k in "${!__REFLEX_STAGED_PARAMS[@]}"; do
    printf "  %s = %s\n" \
      "$(__reflex_c "35" "$k")" \
      "$(__reflex_c "33" "${__REFLEX_STAGED_PARAMS[$k]}")"
  done | sort
}

reflex_clear() {
  __REFLEX_STAGED_PARAMS=()
  _reflex_ok "Cleared staged params."
}

# ----------------------------
# [5] Pose Helpers (6 params)
# ----------------------------
reflex_pose() {
  # Stage pose params.
  # Usage:
  #   reflex_pose                      -> stage defaults
  #   reflex_pose x y z r p y          -> stage provided values
  #
  # Defaults can be overridden via env:
  #   REFLEX_POSE_X, REFLEX_POSE_Y, REFLEX_POSE_Z,
  #   REFLEX_POSE_ROLL, REFLEX_POSE_PITCH, REFLEX_POSE_YAW

  local x y z r p yaw

  if [[ $# -eq 0 ]]; then
    x="${REFLEX_POSE_X:-0.0}"
    y="${REFLEX_POSE_Y:-0.0}"
    z="${REFLEX_POSE_Z:-0.5}"
    r="${REFLEX_POSE_ROLL:-0.0}"
    p="${REFLEX_POSE_PITCH:-0.0}"
    yaw="${REFLEX_POSE_YAW:-0.0}"
    _reflex_info "Using default pose: x=$x y=$y z=$z roll=$r pitch=$p yaw=$yaw"
  elif [[ $# -eq 6 ]]; then
    x="$1"; y="$2"; z="$3"; r="$4"; p="$5"; yaw="$6"
  else
    _reflex_err "Usage: reflex_pose [<x> <y> <z> <roll_deg> <pitch_deg> <yaw_deg>]"
    _reflex_err "  (no args -> defaults: x=${REFLEX_POSE_X:-0.0} y=${REFLEX_POSE_Y:-0.0} z=${REFLEX_POSE_Z:-0.5} roll=${REFLEX_POSE_ROLL:-0.0} pitch=${REFLEX_POSE_PITCH:-0.0} yaw=${REFLEX_POSE_YAW:-0.0})"
    return 2
  fi

  reflex_set x "$x"
  reflex_set y "$y"
  reflex_set z "$z"
  reflex_set roll_deg "$r"
  reflex_set pitch_deg "$p"
  reflex_set yaw_deg "$yaw"
  _reflex_warn "Pose staged (6 params). Run: reflex_apply"
}

# Individual setters (stage only)
reflex_x()     { reflex_set x "$1"; }
reflex_y()     { reflex_set y "$1"; }
reflex_z()     { reflex_set z "$1"; }
reflex_roll()  { reflex_set roll_deg "$1"; }
reflex_pitch() { reflex_set pitch_deg "$1"; }
reflex_yaw()   { reflex_set yaw_deg "$1"; }

# ----------------------------
# [6] Services: reset_pid / apply_pose
# ----------------------------
reflex_reset_pid() {
  if ros2 service list 2>/dev/null | grep -qx "/reset_pid"; then
    ros2 service call /reset_pid std_srvs/srv/Trigger "{}" >/dev/null \
      && _reflex_ok "PID reset: /reset_pid" \
      || { _reflex_err "PID reset FAILED: /reset_pid"; return 1; }
  else
    _reflex_err "Service not found: /reset_pid"
    return 1
  fi
}

reflex_apply_pose_srv() {
  if ros2 service list 2>/dev/null | grep -qx "/apply_pose"; then
    ros2 service call /apply_pose std_srvs/srv/Trigger "{}" >/dev/null \
      && _reflex_ok "Pose applied: /apply_pose" \
      || { _reflex_err "Pose apply FAILED: /apply_pose"; return 1; }
  else
    _reflex_err "Service not found: /apply_pose"
    return 1
  fi
}
reflex_apply_gains_srv() {
  if ros2 service list 2>/dev/null | grep -qx "/apply_gains"; then
    ros2 service call /apply_gains std_srvs/srv/Trigger "{}" >/dev/null \
      && _reflex_ok "Gains applied: /apply_gains" \
      || { _reflex_err "Gains apply FAILED: /apply_gains"; return 1; }
  else
    _reflex_err "Service not found: /apply_gains"
    return 1
  fi
}

# ----------------------------
# [7] Gazebo Harmonic pause/resume
# ----------------------------
reflex_pause() {
  gz service -s /world/reflex_world/control \
    --reqtype gz.msgs.WorldControl \
    --reptype gz.msgs.Boolean \
    --req 'pause: true' >/dev/null 2>&1 \
    && _reflex_ok "Simulation paused." \
    || _reflex_warn "Could not pause simulation."
}

reflex_resume() {
  gz service -s /world/reflex_world/control \
    --reqtype gz.msgs.WorldControl \
    --reptype gz.msgs.Boolean \
    --req 'pause: false' >/dev/null 2>&1 \
    && _reflex_ok "Simulation resumed." \
    || _reflex_warn "Could not resume simulation."
}

# ----------------------------
# [7.5] Stop diff_drive cmd_vel (safety)
# ----------------------------
reflex_stop() {
  # Publish zero cmd_vel a few times to ensure controller receives it.
  # If another node publishes non-zero, it can override immediately.
  local topic="${REFLEX_CMD_VEL_TOPIC:-/diff_drive_controller/cmd_vel}"
  local rate="${REFLEX_STOP_RATE_HZ:-30}"
  local seconds="${REFLEX_STOP_SECONDS:-0.35}"

  # Only if topic exists (avoid errors when controller not running)
  if ! ros2 topic list 2>/dev/null | grep -qx "${topic}"; then
    _reflex_warn "cmd_vel topic not found: ${topic} (skip stop)"
    return 0
  fi

  _reflex_info "STOP: publishing zero Twist -> ${topic} (${seconds}s @ ${rate}Hz)"
  ros2 topic pub -r "${rate}" "${topic}" geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    >/dev/null 2>&1 &

  local pub_pid=$!
  sleep "${seconds}" 2>/dev/null || true
  kill "${pub_pid}" >/dev/null 2>&1 || true
  wait "${pub_pid}" >/dev/null 2>&1 || true
  _reflex_ok "STOP: zero cmd_vel sent."
}

# ----------------------------
# [7.8] Global reset (PID integrators cleared) - std_srvs/Trigger on /reset
# ----------------------------
reflex_reset_global() {
  local srv="${REFLEX_GLOBAL_RESET_SRV:-/reset}"
  _reflex_info "Calling global reset: ${srv} (std_srvs/srv/Trigger)"
  ros2 service call "${srv}" std_srvs/srv/Trigger "{}" >/dev/null 2>&1 \
    && _reflex_ok "Global reset done." \
    || { _reflex_err "Global reset FAILED (${srv})"; return 1; }
}

# ----------------------------
# [8] Apply Staged Params (pause -> set -> reset -> apply_gains -> apply_pose -> stop -> resume)
# - If called with NO staged params: still do (pause -> reset -> apply_gains -> apply_pose -> stop -> resume)
# ----------------------------
reflex_apply() {
  local had_params=1
  if [[ ${#__REFLEX_STAGED_PARAMS[@]} -eq 0 ]]; then
    had_params=0
    _reflex_info "No staged params. Will still: pause -> global_reset -> apply_gains -> apply_pose -> stop -> resume"
  fi

  reflex_pause || { _reflex_err "Failed to pause"; return 1; }

  # Apply staged params if present
  if [[ $had_params -eq 1 ]]; then
    _reflex_info "Applying staged params to $(__reflex_c "35" "${REFLEX_TUNER_NODE}") ..."
    local k
    for k in "${!__REFLEX_STAGED_PARAMS[@]}"; do
      if ros2 param set "${REFLEX_TUNER_NODE}" "$k" "${__REFLEX_STAGED_PARAMS[$k]}" >/dev/null; then
        _reflex_ok "Applied: $k=${__REFLEX_STAGED_PARAMS[$k]}"
      else
        _reflex_err "FAILED applying: $k"
        reflex_stop   >/dev/null 2>&1 || true
        reflex_resume >/dev/null 2>&1 || true
        return 1
      fi
    done

    __REFLEX_STAGED_PARAMS=()
    _reflex_ok "All staged params applied."
  fi

  reflex_apply_gains_srv || { reflex_resume; return 1; }
  reflex_reset_pid       || { reflex_resume; return 1; }
  reflex_apply_pose_srv  || { reflex_resume; return 1; }

  # Safety: stop before resuming to avoid a “jump” on unpause
  reflex_stop >/dev/null 2>&1 || true

  reflex_resume || { _reflex_err "Failed to resume"; return 1; }
}

# ----------------------------
# [9] Batch tuning + presets + inspect
# ----------------------------
reflex_tune() {
  # Stage many params as pairs: reflex_tune t_kp -70 t_kd -1.5 v_kp -2
  if (( $# % 2 != 0 )); then
    _reflex_err "Usage: reflex_tune <p1> <v1> <p2> <v2> ..."
    return 2
  fi
  while (( $# )); do
    reflex_set "$1" "$2"
    shift 2
  done
  _reflex_warn "Tuning staged. Run: reflex_apply"
}

reflex_get() {
  # Print current key params from node (live)
  local p
  for p in \
    left_joint right_joint wheel_radius wheel_separation balance_axis theta_max w_max inner_rate_hz outer_rate_hz \
    v_kp v_ki v_kd v_i_max v_out_max \
    t_kp t_ki t_kd t_i_max t_out_max \
    x y z roll_deg pitch_deg yaw_deg
  do
    ros2 param get "${REFLEX_TUNER_NODE}" "$p" 2>/dev/null | sed "s/^/  /"
  done
}

reflex_preset_safe() {
  reflex_tune \
    wheel_radius 0.06 wheel_separation 0.30 balance_axis roll \
    theta_max 0.20 w_max 20.0 \
    inner_rate_hz 250.0 outer_rate_hz 50.0 \
    v_kp -2.0 v_ki -0.5 v_kd 0.0 v_i_max 0.15 v_out_max 0.12 \
    t_kp -50.0 t_ki 0.0 t_kd -1.0 t_i_max 2.0 t_out_max 15.0
}

reflex_preset_aggr() {
  reflex_tune \
    theta_max 0.25 w_max 40.0 \
    v_kp -5.0 v_ki -3.0 v_kd 0.0 v_i_max 0.30 v_out_max 0.25 \
    t_kp -90.0 t_ki 0.0 t_kd -2.0 t_i_max 5.0 t_out_max 40.0
}

# ----------------------------
# [10] Help
# ----------------------------
reflex_help() {
  cat <<EOF
$(__reflex_c '36' "REFLEX (container) helpers:")

  $(__reflex_c '35' "Navigation")
    reflex_cd                -> cd to \$REFLEX_WS (${REFLEX_WS})
    reflex_ws                -> cd to ROS workspace (\$REFLEX_WS/ros_ws)

  $(__reflex_c '35' "Environment")
    reflex_src               -> source ROS + workspace overlays

  $(__reflex_c '35' "Build")
    reflex_build             -> build workspace then re-source overlay

  $(__reflex_c '35' "Staging (params)")
    reflex_set <p> <v>       -> stage any single param (e.g., t_kp -70.0)
    reflex_tune <p v...>     -> stage many params as pairs
    reflex_show              -> show staged params
    reflex_clear             -> clear staged params

  $(__reflex_c '35' "Pose (6 params)")
    reflex_pose x y z r p y  -> stage pose all at once (deg for angles)
    reflex_x|y|z <v>         -> stage individual position
    reflex_roll|pitch|yaw <d>-> stage individual angles (deg)

  $(__reflex_c '35' "Apply / Services")
    reflex_apply             -> pause -> set params -> reset_pid -> apply_pose -> resume
    reflex_reset_pid         -> call /reset_pid
    reflex_apply_pose_srv    -> call /apply_pose
    reflex_pause             -> pause Gazebo (world/default)
    reflex_resume            -> resume Gazebo (world/default)

  $(__reflex_c '35' "Presets / Inspect")
    reflex_get               -> print current key params (live)
    reflex_preset_safe       -> stage conservative defaults
    reflex_preset_aggr       -> stage aggressive defaults

  $(__reflex_c '35' "Tips")
    export NO_COLOR=1        -> disable colors
    export REFLEX_DEBUG=1    -> enable debug prints (limited)

EOF
}

# ----------------------------
# [11] Slow down simulation
# ----------------------------

reflex_slow() {
  local factor="${1:-0.3}"
  gz service -s /world/reflex_world/set_physics \
    --reqtype gz.msgs.Physics \
    --reptype gz.msgs.Boolean \
    --req "real_time_factor: ${factor}" >/dev/null 2>&1 \
    && _reflex_ok "Simulation slowed (RTF=${factor})" \
    || _reflex_warn "Failed to set real_time_factor."
}