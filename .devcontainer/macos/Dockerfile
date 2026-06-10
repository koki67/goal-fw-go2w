FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Same package set as the desktop Dockerfile, plus the in-container GUI stack
# (Xvfb + fluxbox + x11vnc + websockify + novnc + Mesa llvmpipe). XQuartz on
# Apple Silicon cannot serve the OpenGL 3.3 context Ogre/RViz require, so the
# macOS profile renders inside the container with software OpenGL and exposes
# the framebuffer to the Mac browser via noVNC on port 6080.
RUN apt-get update -q && \
    apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        dbus-x11 \
        fluxbox \
        iproute2 \
        iputils-ping \
        less \
        libeigen3-dev \
        libgl1-mesa-dri \
        libglu1-mesa \
        libomp-dev \
        libpcap-dev \
        libpcl-dev \
        mesa-utils \
        procps \
        python3-colcon-common-extensions \
        python3-pip \
        ros-humble-nav2-bringup \
        ros-humble-nav2-bt-navigator \
        ros-humble-nav2-lifecycle-manager \
        ros-humble-nav2-mppi-controller \
        ros-humble-nav2-msgs \
        ros-humble-nav2-navfn-planner \
        ros-humble-nav2-simple-commander \
        ros-humble-nav2-velocity-smoother \
        ros-humble-octomap-msgs \
        ros-humble-octomap-rviz-plugins \
        ros-humble-octomap-server \
        ros-humble-pcl-conversions \
        ros-humble-pcl-ros \
        ros-humble-pointcloud-to-laserscan \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-ros2bag \
        ros-humble-rosbag2 \
        ros-humble-rosbag2-storage-default-plugins \
        ros-humble-robot-state-publisher \
        ros-humble-rosidl-generator-dds-idl \
        ros-humble-rviz2 \
        ros-humble-slam-toolbox \
        ros-humble-tf2-tools \
        ros-humble-tf-transformations \
        websockify \
        x11-utils \
        x11vnc \
        xauth \
        xvfb && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
        distro \
        transforms3d \
        websocket-client

# Install upstream noVNC 1.4 (Debian's package is from 2018 and the JS bundle
# does not initialize cleanly in modern browsers, leaving the tab on a forever
# spinner). Pinned tarball, extracted to /opt/novnc, with a vnc.html alias at
# the web root so opening port 6080 lands on the client immediately.
ARG NOVNC_VERSION=1.4.0
RUN curl -fsSL "https://github.com/novnc/noVNC/archive/refs/tags/v${NOVNC_VERSION}.tar.gz" \
        | tar -xz -C /opt && \
    mv "/opt/noVNC-${NOVNC_VERSION}" /opt/novnc && \
    ln -s /opt/novnc/vnc.html /opt/novnc/index.html

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0
ENV ROS_LOCALHOST_ONLY=1
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:1
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV QT_QUICK_BACKEND=software

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
