#!/usr/bin/env bash
set -e

# Keep entrypoint behavior transparent: run exactly what Docker passes in.
exec "$@"
