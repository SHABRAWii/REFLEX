# -----------------------------
# REFLEX Host CLI
# -----------------------------

# -------- Configuration --------
: "${REFLEX_CONTAINER:=reflex_dev}"
: "${REFLEX_BASE_IMAGE:=reflex:jazzy-harmonic}"
: "${REFLEX_WRAPPER_IMAGE:=reflex:rocker}"
: "${REFLEX_REPO_ROOT:=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
: "${REFLEX_DAEMON_SCRIPT:=${REFLEX_REPO_ROOT}/docker/rocker_daemon.sh}"
: "${REFLEX_VERSION_FILE:=${REFLEX_REPO_ROOT}/VERSION}"

# -------- Colors --------
RESET="\033[0m"
BOLD="\033[1m"
GREEN="\033[1;32m"
RED="\033[1;31m"
YELLOW="\033[1;33m"
BLUE="\033[1;34m"
CYAN="\033[1;36m"
GRAY="\033[90m"

_ok()     { echo -e "${GREEN}✔${RESET} $1"; }
_warn()   { echo -e "${YELLOW}▶${RESET} $1"; }
_err()    { echo -e "${RED}✖${RESET} $1"; }
_title()  { echo -e "${BOLD}${CYAN}$1${RESET}"; }

# -------- Internal --------
_reflex_is_running() {
  docker ps --format '{{.Names}}' | grep -qx "${REFLEX_CONTAINER}"
}

_reflex_exists() {
  docker ps -a --format '{{.Names}}' | grep -qx "${REFLEX_CONTAINER}"
}

_reflex_version() { 
  [[ -f "$REFLEX_VERSION_FILE" ]] && cat "$REFLEX_VERSION_FILE" || echo "dev"; 
}

# -------- Commands --------

reflex_status() {
  _title "REFLEX Status"
  if _reflex_is_running; then
    _ok "Container is running (${REFLEX_CONTAINER})"
    docker ps --filter "name=^/${REFLEX_CONTAINER}$" \
      --format "  {{.Names}}  {{.Image}}  {{.Status}}"
  else
    _warn "Container is not running"
  fi
  echo
}

reflex_up() {
  if _reflex_is_running; then
    _warn "REFLEX already running"
    return 0
  fi

  _title "Starting REFLEX..."
  # Delegate startup policy (GPU flags, mounts, wrapper image) to daemon script.
  (cd "${REFLEX_REPO_ROOT}" && "${REFLEX_DAEMON_SCRIPT}")

  if _reflex_is_running; then
    _ok "REFLEX is up"
  else
    _err "Failed to start REFLEX"
  fi
  echo
}

reflex_down() {
  if _reflex_is_running; then
    _title "Stopping REFLEX..."
    docker stop "${REFLEX_CONTAINER}" >/dev/null
    docker rm "${REFLEX_CONTAINER}" >/dev/null 2>&1 || true
    _ok "Container stopped"
  else
    _warn "REFLEX not running"
  fi
  echo
}

reflex_restart() {
  _title "Restarting REFLEX..."
  reflex_down
  reflex_up
}

reflex_shell() {
  # Ensure container exists before attaching interactive shell.
  reflex_up
  _title "Opening REFLEX shell..."
  docker exec -it "${REFLEX_CONTAINER}" bash -i
}

reflex_exec() {
  reflex_up
  if [[ $# -eq 0 ]]; then
    _err "Usage: reflex_exec <command>"
    return 2
  fi

  _title "Executing in REFLEX:"
  echo -e "${GRAY}$*${RESET}"
  echo
  # Run through bash -lc so aliases/profile + shell syntax behave predictably.
  docker exec -it "${REFLEX_CONTAINER}" bash -lc "$*"
}

reflex_logs() {
  reflex_up
  _title "Following REFLEX logs..."
  docker logs -f "${REFLEX_CONTAINER}"
}

reflex_cd() {
  cd "${REFLEX_REPO_ROOT}" || return
  _ok "Moved to REFLEX root"
}

reflex_rebuild() {
  _title "Rebuilding ${REFLEX_BASE_IMAGE}..."
  # Rebuild base image, then restart so running container picks it up.
  (cd "${REFLEX_REPO_ROOT}" && \
    docker build -t "${REFLEX_BASE_IMAGE}" -f docker/Dockerfile .)

  _ok "Image rebuilt"
  reflex_restart
}

reflex_reset_wrapper() {
  _title "Removing wrapper image..."
  docker rmi "${REFLEX_WRAPPER_IMAGE}" 2>/dev/null || true
  _ok "Wrapper removed (will rebuild on next start)"
}

# --- Help command ---

reflex_help() {
  # ---------- Colors ----------
  local RESET="\033[0m"
  local BOLD="\033[1m"

  local TITLE="\033[1;36m"      # Bright Cyan
  local SECTION="\033[1;34m"    # Bright Blue
  local CMD="\033[1;32m"        # Bright Green
  local ARG="\033[1;33m"        # Bright Yellow
  local DESC="\033[90m"         # Gray
  local LINE="\033[2;37m"       # Dim White

  echo
  echo -e "${TITLE}╔═══════════════════════════════════════════╗${RESET}"
  echo -e "${TITLE}║             REFLEX CLI v$(_reflex_version)             ║${RESET}"
  echo -e "${TITLE}╚═══════════════════════════════════════════╝${RESET}"
  echo

  # ---------- Lifecycle ----------
  echo -e "${SECTION}Container Lifecycle${RESET}"
  echo -e "${LINE}────────────────────────────────────────────${RESET}"
  printf "  ${CMD}%-22s${RESET} ${DESC}%s${RESET}\n" \
    "reflex_up"       "Start container if not running"
  printf "  ${CMD}%-22s${RESET} ${DESC}%s${RESET}\n" \
    "reflex_down"     "Stop and remove container"
  printf "  ${CMD}%-22s${RESET} ${DESC}%s${RESET}\n" \
    "reflex_restart"  "Restart container (down → up)"
  printf "  ${CMD}%-22s${RESET} ${DESC}%s${RESET}\n" \
    "reflex_status"   "Show container status"
  echo

  # ---------- Access ----------
  echo -e "${SECTION}Container Access${RESET}"
  echo -e "${LINE}────────────────────────────────────────────${RESET}"
  printf "  ${CMD}%-22s${RESET} ${DESC}%s${RESET}\n" \
    "reflex_shell"    "Open interactive shell inside container"
  printf "  ${CMD}%-22s${ARG}<cmd>${RESET} ${DESC}%s${RESET}\n" \
    "reflex_exec"     "Execute command inside container"
  printf "  ${CMD}%-22s${RESET} ${DESC}%s${RESET}\n" \
    "reflex_logs"     "Follow container logs (docker logs -f)"
  echo

  # ---------- Image ----------
  echo -e "${SECTION}Image Management${RESET}"
  echo -e "${LINE}────────────────────────────────────────────${RESET}"
  printf "  ${CMD}%-22s${RESET} ${DESC}%s${RESET}\n" \
    "reflex_rebuild"      "Rebuild base image and restart"
  printf "  ${CMD}%-22s${RESET} ${DESC}%s${RESET}\n" \
    "reflex_reset_wrapper" "Remove wrapper image (forces rocker rebuild)"
  echo

  # ---------- Navigation ----------
  echo -e "${SECTION}Navigation${RESET}"
  echo -e "${LINE}────────────────────────────────────────────${RESET}"
  printf "  ${CMD}%-22s${RESET} ${DESC}%s${RESET}\n" \
    "reflex_cd"       "Jump to REFLEX repository root"
  echo

  # ---------- Examples ----------
  echo -e "${SECTION}Examples${RESET}"
  echo -e "${LINE}────────────────────────────────────────────${RESET}"
  echo -e "  ${CMD}reflex_up${RESET}"
  echo -e "  ${CMD}reflex_shell${RESET}"
  echo -e "  ${CMD}reflex_exec${RESET} \"colcon build --symlink-install\""
  echo -e "  ${CMD}reflex_exec${RESET} \"ros2 launch vanta_gazebo sim.launch.py\""
  echo

  echo -e "${TITLE}══════════════════════════════════════════════${RESET}"
  echo
}

reflex_version() {
  echo "REFLEX CLI v$(_reflex_version)"
}
