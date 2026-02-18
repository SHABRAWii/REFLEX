#!/usr/bin/env bash
set -euo pipefail

# Runtime image used for long-lived dev container.
IMAGE_BASE="${IMAGE_BASE:-reflex:jazzy-harmonic}"
# Rocker-generated wrapper image that bakes host integration.
WRAPPER_IMAGE="${WRAPPER_IMAGE:-reflex:rocker}"
CONTAINER_NAME="${CONTAINER_NAME:-reflex_dev}"
HOSTNAME_NAME="${HOSTNAME_NAME:-REFLEX}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

NO_GPU=0
for arg in "$@"; do
  case "$arg" in
    # Allow CPU-only fallback on hosts without NVIDIA runtime.
    --no-gpu) NO_GPU=1 ;;
  esac
done

# Optional joystick
JOY_DEV="/dev/input/js0"
DOCKER_DEV_FLAGS=()
ROCKER_DEV_FLAGS=()
if [[ -e "${JOY_DEV}" ]]; then
  ROCKER_DEV_FLAGS=(--devices "${JOY_DEV}")
  DOCKER_DEV_FLAGS=(--device "${JOY_DEV}:${JOY_DEV}")
fi

# GPU flags (toggleable)
ROCKER_GPU_FLAGS=()
DOCKER_GPU_FLAGS=()
if [[ "${NO_GPU}" -eq 0 ]]; then
  # NOTE: Force GLVND version to 22.04 because ubuntu24.04 tag may not exist on your machine.
  ROCKER_GPU_FLAGS=(--nvidia --nvidia-glvnd-version 22.04)
  DOCKER_GPU_FLAGS=(--gpus all)
else
  echo "Running in CPU-only mode (--no-gpu)."
fi

is_running() {
  docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"
}

# 1) If already running, just report
if is_running; then
  echo "REFLEX container is already running: ${CONTAINER_NAME}"
  echo "Attach with: docker exec -it ${CONTAINER_NAME} bash -i"
  exit 0
fi

# 2) If container exists but stopped, remove it
if docker ps -a --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
  echo "Removing existing stopped container: ${CONTAINER_NAME}"
  docker rm "${CONTAINER_NAME}" >/dev/null || true
fi

echo "Ensuring rocker wrapper image exists: ${WRAPPER_IMAGE}"
echo "Base image: ${IMAGE_BASE}"
echo "Repo mounted: ${REPO_ROOT} -> /ws"

# 3) Build/update wrapper image with rocker (cached after first time)
# IMPORTANT: Do NOT pass '--gpus all' to rocker; rocker uses '--nvidia' and your docker must support --gpus.
rocker \
  --hostname "${HOSTNAME_NAME}" \
  --x11 \
  "${ROCKER_GPU_FLAGS[@]}" \
  --user \
  --volume "${REPO_ROOT}:/ws" \
  --image-name "${WRAPPER_IMAGE}" \
  "${ROCKER_DEV_FLAGS[@]}" \
  -- \
  "${IMAGE_BASE}" \
  /bin/true >/dev/null

# 4) Start persistent container detached
echo "Starting REFLEX container in background: ${CONTAINER_NAME}"

# Keep the container alive; users attach with docker exec.
docker run -d \
  --name "${CONTAINER_NAME}" \
  --hostname "${HOSTNAME_NAME}" \
  "${DOCKER_GPU_FLAGS[@]}" \
  -e DISPLAY \
  -e TERM=xterm-256color \
  -e QT_X11_NO_MITSHM=1 \
  "${DOCKER_DEV_FLAGS[@]}" \
  -v "${REPO_ROOT}:/ws" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /etc/localtime:/etc/localtime:ro \
  "${WRAPPER_IMAGE}" \
  sleep infinity >/dev/null

echo "✅ REFLEX is running: ${CONTAINER_NAME}"
echo "Open a shell with: docker exec -it ${CONTAINER_NAME} bash -i"
echo "Stop it with: docker stop ${CONTAINER_NAME}"
