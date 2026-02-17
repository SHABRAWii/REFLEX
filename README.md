# REFLEX

**REFLEX** is a fully reproducible robotics development environment
built on Docker, ROS 2, and Rocker.

It provides:

-   🐳 Containerized ROS 2 development
-   🚀 One-command environment startup
-   🛠 Host CLI utilities (`reflex_up`, `reflex_shell`, etc.)
-   🔁 GPU + CPU fallback modes
-   ⚙ Automated Ubuntu installer

------------------------------------------------------------------------

## 🚀 Quick Start

### 1️⃣ Install

``` bash
chmod +x install.sh
./install.sh
```

If prompted, log out and log back in (only required if you were added to
the docker group).

Then activate:

``` bash
source ~/.bashrc
```

------------------------------------------------------------------------

### 2️⃣ Start Environment

``` bash
reflex_up
reflex_shell
```

------------------------------------------------------------------------

## 🧠 Available Commands

Run:

``` bash
reflex_help
```

Core commands:

  Command               Description
  --------------------- ----------------------------------
  `reflex_up`           Start container if not running
  `reflex_shell`        Open interactive container shell
  `reflex_exec <cmd>`   Run command inside container
  `reflex_down`         Stop container
  `reflex_restart`      Restart container
  `reflex_rebuild`      Rebuild Docker image
  `reflex_status`       Show container status

------------------------------------------------------------------------

## 🐳 Docker Structure

    docker/
     ├── Dockerfile
     ├── entrypoint.sh
     ├── profile.sh
     ├── rocker.sh
     ├── rocker_daemon.sh

-   `rocker.sh` → interactive mode\
-   `rocker_daemon.sh` → background container mode

------------------------------------------------------------------------

## 🧩 Project Structure

    REFLEX/
     ├── docker/
     ├── scripts/host/
     │    └── reflex.sh
     ├── install.sh
     ├── VERSION
     ├── LICENSE
     └── README.md

------------------------------------------------------------------------

## 🖥 GPU Support

By default, REFLEX attempts to use NVIDIA GPU acceleration.

To run without GPU:

``` bash
./docker/rocker_daemon.sh --no-gpu
```

Or modify your CLI workflow accordingly.

------------------------------------------------------------------------

## 🔁 Development Workflow

> **TO DO**  
> The ROS 2 packages and launch workflows are still under active development.  
> This section will be updated once the workspace layout, dependencies, and standard build/launch commands are finalized.

------------------------------------------------------------------------

## 📌 Versioning

Current version is defined in:

    VERSION

Check via:

``` bash
reflex_version
```

------------------------------------------------------------------------

## 📜 License

All rights reserved by the company unless otherwise specified in the
LICENSE file.

------------------------------------------------------------------------

## 👨‍💻 Author

Yousef ElShabrawii\
Robotics & Systems Engineer