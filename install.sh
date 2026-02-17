#!/usr/bin/env bash
set -euo pipefail

# =============================
# REFLEX one-shot installer
# Ubuntu 22.04 / 24.04
# =============================

# -------- Args --------
NO_GPU=0
for arg in "$@"; do
  case "$arg" in
    --no-gpu) NO_GPU=1 ;;
    -h|--help)
      cat <<'EOF'
Usage: ./install.sh [--no-gpu]

Options:
  --no-gpu   Skip NVIDIA GPU checks and do not enforce NVIDIA runtime tooling
EOF
      exit 0
      ;;
  esac
done

# ---- Config ----
# Default image tag used by docker/rocker_daemon.sh and reflex_rebuild.
REFLEX_BASE_IMAGE="${REFLEX_BASE_IMAGE:-reflex:jazzy-harmonic}"
REFLEX_REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REFLEX_BASHRC="${REFLEX_BASHRC:-$HOME/.bashrc}"
REFLEX_SOURCE_LINE="source ${REFLEX_REPO_ROOT}/scripts/host/reflex.sh"

# ---- Helpers ----
log()  { echo -e "\033[1;36m[REFLEX]\033[0m $*"; }
ok()   { echo -e "\033[1;32m✔\033[0m $*"; }
warn() { echo -e "\033[1;33m▶\033[0m $*"; }
err()  { echo -e "\033[1;31m✖\033[0m $*"; }

need_cmd() { command -v "$1" >/dev/null 2>&1; }
require_file() { [[ -f "$1" ]] || { err "Missing required file: $1"; exit 2; }; }
is_ubuntu() { [[ -f /etc/os-release ]] && . /etc/os-release && [[ "${ID:-}" == "ubuntu" ]]; }

# ---- Sanity checks ----
log "Repo root: ${REFLEX_REPO_ROOT}"
require_file "${REFLEX_REPO_ROOT}/docker/Dockerfile"
require_file "${REFLEX_REPO_ROOT}/docker/rocker_daemon.sh"
require_file "${REFLEX_REPO_ROOT}/scripts/host/reflex.sh"

if ! is_ubuntu; then
  err "This installer currently supports Ubuntu only."
  exit 3
fi

# ---- Update apt + prerequisites ----
log "Updating apt index..."
sudo apt-get update -y

log "Installing prerequisites..."
sudo apt-get install -y --no-install-recommends \
  ca-certificates curl gnupg lsb-release \
  python3 python3-pip python3-venv \
  git

# ---- Docker Engine ----
if need_cmd docker; then
  ok "Docker already installed: $(docker --version | head -n1)"
else
  log "Installing Docker Engine from Docker's official repo..."

  sudo install -m 0755 -d /etc/apt/keyrings
  curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
  sudo chmod a+r /etc/apt/keyrings/docker.gpg

  . /etc/os-release
  ARCH="$(dpkg --print-architecture)"
  CODENAME="${VERSION_CODENAME}"

  echo \
"deb [arch=${ARCH} signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
${CODENAME} stable" | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null

  sudo apt-get update -y
  sudo apt-get install -y --no-install-recommends \
    docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

  ok "Docker installed: $(docker --version | head -n1)"
fi

log "Enabling Docker service..."
sudo systemctl enable --now docker >/dev/null 2>&1 || true
ok "Docker service enabled."

# ---- docker group ----
NEEDS_RELOGIN=0
if groups "$USER" | grep -q "\bdocker\b"; then
  ok "User '$USER' already in docker group."
else
  log "Adding '$USER' to docker group..."
  # Non-root Docker usage requires group membership to apply in a new login session.
  sudo usermod -aG docker "$USER"
  NEEDS_RELOGIN=1
  warn "Group change requires log out/in (or reboot) to use docker without sudo."
fi

# ---- pipx + rocker ----
if ! need_cmd pipx; then
  log "Installing pipx..."
  sudo apt-get install -y --no-install-recommends pipx
  ok "pipx installed."
else
  ok "pipx already installed."
fi

export PATH="$HOME/.local/bin:$PATH"

if need_cmd rocker; then
  ok "rocker already installed."
else
  log "Installing rocker via pipx..."
  pipx install rocker >/dev/null || pipx upgrade rocker >/dev/null
  ok "rocker installed."
fi

# ---- GPU / NVIDIA checks (optional) ----
if [[ "$NO_GPU" -eq 1 ]]; then
  warn "GPU checks skipped (--no-gpu)."
else
  log "Checking GPU/NVIDIA prerequisites..."
  if need_cmd nvidia-smi; then
    ok "nvidia-smi detected."
  else
    warn "nvidia-smi not found. If you want GPU acceleration, install NVIDIA drivers on host."
  fi

  # NVIDIA container toolkit check (best-effort)
  if docker info 2>/dev/null | grep -qi 'Runtimes:.*nvidia'; then
    ok "Docker NVIDIA runtime detected."
  else
    warn "Docker NVIDIA runtime not detected."
    warn "If you need GPU inside container, install NVIDIA Container Toolkit."
  fi
fi

# ---- Make scripts executable ----
log "Making scripts executable..."
chmod +x "${REFLEX_REPO_ROOT}/docker/rocker_daemon.sh" || true
chmod +x "${REFLEX_REPO_ROOT}/install.sh" || true
ok "Scripts ready."

# ---- Build image ----
log "Building image: ${REFLEX_BASE_IMAGE}"
( cd "${REFLEX_REPO_ROOT}" && docker build -t "${REFLEX_BASE_IMAGE}" -f docker/Dockerfile . )
ok "Image built: ${REFLEX_BASE_IMAGE}"

# ---- Add source line to ~/.bashrc (idempotent) ----
log "Updating ${REFLEX_BASHRC}..."
if [[ -f "${REFLEX_BASHRC}" ]] && grep -Fxq "${REFLEX_SOURCE_LINE}" "${REFLEX_BASHRC}"; then
  ok "Already sourced in .bashrc"
else
  # Append once so shell helpers are available in new terminals.
  {
    echo
    echo "# REFLEX CLI"
    echo "${REFLEX_SOURCE_LINE}"
  } >> "${REFLEX_BASHRC}"
  ok "Added REFLEX CLI source line to .bashrc"
fi

# ---- Final message ----
echo
ok "Install complete."
echo

if [[ "$NEEDS_RELOGIN" -eq 1 ]]; then
  warn "IMPORTANT: log out/in (or reboot) before using docker without sudo."
  echo
fi

log "Next:"
echo "  source ~/.bashrc"
echo "  reflex_help"
echo "  reflex_up"
echo "  reflex_shell"
echo
