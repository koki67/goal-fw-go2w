FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update -q && \
    apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        dbus-x11 \
        iproute2 \
        iputils-ping \
        less \
        libeigen3-dev \
        libomp-dev \
        libpcap-dev \
        libpcl-dev \
        mesa-utils \
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
        xauth && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
        distro \
        pypcd4==1.4.3 \
        small_gicp==1.0.0 \
        transforms3d \
        websocket-client

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0
ENV QT_X11_NO_MITSHM=1

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
