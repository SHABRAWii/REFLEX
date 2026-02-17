# REFLEX Docker Environment

This directory contains all Docker-related infrastructure required to
build and run the REFLEX robotics development environment.

------------------------------------------------------------------------

## 📦 Overview

The Docker environment provides:

-   ROS 2 base image (Jazzy / Harmonic compatible)
-   GPU-enabled runtime (NVIDIA support)
-   X11 forwarding for GUI applications (Gazebo / RViz)
-   Persistent development container mode
-   Rocker-based wrapper image for host integration

------------------------------------------------------------------------

## 📁 File Structure

    docker/
     ├── Dockerfile
     ├── entrypoint.sh
     ├── profile.sh
     ├── rocker.sh
     ├── rocker_daemon.sh

### Dockerfile

Defines the base ROS 2 development image used by REFLEX.

### entrypoint.sh

Executed when the container starts. Responsible for environment
preparation.

### profile.sh

Custom shell profile loaded inside the container.

### rocker.sh

Runs container in interactive mode.

### rocker_daemon.sh

Runs container in persistent background mode (recommended workflow).

------------------------------------------------------------------------

## 🚀 Usage

### Start Persistent Container

``` bash
./docker/rocker_daemon.sh
```

### Open Shell

``` bash
docker exec -it reflex_dev bash
```

Or use host CLI:

``` bash
reflex_shell
```

------------------------------------------------------------------------

## 🖥 GPU Support

The container attempts to use NVIDIA GPU by default.

To disable GPU:

``` bash
./docker/rocker_daemon.sh --no-gpu
```

------------------------------------------------------------------------

## 📌 Notes

-   The workspace is mounted from the repository root into `/ws`.
-   The container hostname is set to `REFLEX`.
-   The container name is `reflex_dev`.

------------------------------------------------------------------------

## 🔁 Development Workflow

> **TO DO**\
> Final ROS 2 package structure and build workflow are still under
> development.