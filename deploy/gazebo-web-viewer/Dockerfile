FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    VLA_HOME=/opt/VLN-VLA \
    DISPLAY=:0 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    QT_X11_NO_MITSHM=1 \
    GZ_SIM_RENDER_ENGINE=ogre

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg lsb-release software-properties-common \
    && install -d -m 0755 /usr/share/keyrings \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
       -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
       > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake pkg-config cppzmq-dev iptables iproute2 \
    libgz-cmake3-dev libgz-plugin2-dev libgz-sim8-dev \
    libgz-transport13-dev libgz-msgs10-dev libgz-common5-dev \
    libgz-common5-profiler-dev libgz-math7-dev gz-harmonic \
    xvfb x11vnc novnc websockify openbox dbus-x11 xterm \
    libgl1-mesa-dri libglx-mesa0 mesa-utils fonts-liberation \
    python3-numpy python3-opencv python3-pip python3-venv \
    && install -d -m 0755 /usr/share/keyrings \
    && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
       > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
       ros-jazzy-ros-base ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim \
       ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
       python3-colcon-common-extensions python3-gz-transport13 python3-gz-msgs10 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/VLN-VLA
COPY . /opt/VLN-VLA

RUN source /opt/ros/jazzy/setup.bash \
    && cmake -S vla_ws/src/vla_follow/gz_plugins \
       -B vla_ws/src/vla_follow/gz_plugins/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build vla_ws/src/vla_follow/gz_plugins/build --parallel 2 \
    && cd vla_ws \
    && colcon build --symlink-install --packages-select vla_follow

COPY deploy/gazebo-web-viewer/entrypoint.sh /usr/local/bin/gazebo-web-viewer
RUN chmod +x /usr/local/bin/gazebo-web-viewer

EXPOSE 6080
STOPSIGNAL SIGTERM
CMD ["/usr/local/bin/gazebo-web-viewer"]
