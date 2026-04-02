# docker/Dockerfile
FROM ubuntu:24.04

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# -------------------------
# Base OS tooling
# -------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg \
    lsb-release \
    locales \
    sudo \
    git \
    nano \
    vim \
    tmux \
    bash-completion \
    python3-pip \
    python3-venv \
    pipx \
    python3-setuptools \
    libxcb-cursor0 \
    && rm -rf /var/lib/apt/lists/*

ENV PATH="/root/.local/bin:${PATH}"


# vcstool + rosdep via pipx 
RUN pipx ensurepath \
 && pipx install vcstool \
 && pipx install rosdep

# Python libraries for GUI/debug tools
RUN python3 -m pip install --no-cache-dir \
    pyqtgraph \
    PySide6

RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# -------------------------
# ROS 2 Jazzy apt repo
# -------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  > /etc/apt/sources.list.d/ros2.list

# -------------------------
# Gazebo (Harmonic) apt repo
# -------------------------
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
  | gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  > /etc/apt/sources.list.d/gazebo-stable.list

# -------------------------
# Install ROS + Gazebo + bridges + control
# -------------------------
ENV ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    cmake \
    pkg-config \
    gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (safe if it was already done)
RUN rosdep init 2>/dev/null || true
RUN rosdep update

# -------------------------
# Quality-of-life + REFLEX env (repo-mounted)
# -------------------------

# Entrypoint (will symlink ~/.bashrc -> repo bashrc when repo is mounted)
COPY docker/entrypoint.sh /reflex_entrypoint.sh
RUN chmod +x /reflex_entrypoint.sh

# Ensure repo bashrc loads for interactive shells (bash -i / docker exec -it ... bash -i)
# This will do nothing if the repo isn't mounted yet.
RUN printf '%s\n' \
'[[ $- == *i* ]] || return 0' \
'[[ -f /ws/scripts/container/reflex_bashrc ]] && source /ws/scripts/container/reflex_bashrc' \
>> /etc/bash.bashrc

# Provide a default bashrc for users created later (e.g., by rocker --user)
# It will also defer to the repo-mounted bashrc when available.
RUN printf '%s\n' \
'export TERM=${TERM:-xterm-256color}' \
'if [[ $- == *i* ]]; then' \
'  export PS1="\\[\\e[1;32m\\]\\u@\\h\\[\\e[0m\\]:\\[\\e[1;34m\\]\\w\\[\\e[0m\\]\\$ "' \
'  [[ -f /ws/scripts/container/reflex_bashrc ]] && source /ws/scripts/container/reflex_bashrc' \
'fi' \
> /etc/skel/.bashrc

# Default mount/workspace path (you’ll mount your repo here with rocker)
ENV REFLEX_NAME=REFLEX
ENV REFLEX_WS=/ws
WORKDIR /ws

ENTRYPOINT ["/reflex_entrypoint.sh"]
CMD ["/bin/bash"]
