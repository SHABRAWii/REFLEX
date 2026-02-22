#!/usr/bin/env bash
set -euo pipefail

# If stdin is not a TTY, try to reattach it to the controlling terminal.
# This prevents docker/rocker from printing tty-resize warnings that break JSON parsing.
if [[ ! -t 0 && -e /dev/tty ]]; then
  exec </dev/tty
fi

# Default image (can override: IMAGE=reflex:jazzy-harmonic ./docker/rocker.sh)
IMAGE="${IMAGE:-reflex:jazzy-harmonic}"

# Stable container name (can override)
CONTAINER_NAME="${CONTAINER_NAME:-reflex_dev}"

# Repo root = one level above docker/
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Optional joystick device (don’t fail if missing)
JOY_DEV="/dev/input/js0"
JOY_FLAG=()
if [[ -e "${JOY_DEV}" ]]; then
  JOY_FLAG=(--devices "${JOY_DEV}")
fi

# Fail fast if image is empty (prevents "required: image" rocker error)
if [[ -z "${IMAGE}" ]]; then
  echo "ERROR: IMAGE is empty. Set IMAGE=reflex:jazzy-harmonic" >&2
  exit 2
fi

# Default command
CMD=("/bin/bash")
# If user passes a command, run it instead
if [[ $# -gt 0 ]]; then
  CMD=("$@")
fi

echo "Launching container '${CONTAINER_NAME}' with image '${IMAGE}'"
# Rocker injects host X11/user/NVIDIA context before launching command.
rocker \
  --name "${CONTAINER_NAME}" \
  --hostname REFLEX \
  "${JOY_FLAG[@]}" \
  --x11 \
  --nvidia \
  --nvidia-glvnd-version 22.04 \
  --user \
  --home \
  --volume "${REPO_ROOT}:/ws" \
  --image-name reflex:rocker \
  -- \
  "${IMAGE}" \
  "${CMD[@]}" \
