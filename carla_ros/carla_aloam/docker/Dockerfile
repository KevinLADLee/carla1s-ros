FROM harbor.isus.tech/osrf/ros:noetic-desktop-focal

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENV PCL_VERSION="1.8.0"
ENV ROS_DISTRO="noetic"
ENV CATKIN_WS=/root/catkin_ws

RUN sed -i "s@http://.*archive.ubuntu.com@http://mirrors.huaweicloud.com@g" /etc/apt/sources.list && \
    sed -i "s@http://.*security.ubuntu.com@http://mirrors.huaweicloud.com@g" /etc/apt/sources.list && \
    sed -i "s@http://packages.ros.org/ros/ubuntu@https://mirror.tuna.tsinghua.edu.cn/ros/ubuntu/@g" /etc/apt/sources.list.d/ros1-latest.list

    # setup processors number used to compile library
RUN apt-get update && apt-get install -y \
      git \
      cmake \
      wget \
      curl \
      libatlas-base-dev \
      libeigen3-dev \
      libgoogle-glog-dev \
      libsuitesparse-dev \
      libceres-dev \
      libflann-dev \
      python3-catkin-tools \
      python3-osrf-pycommon \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-message-filters \
      ros-${ROS_DISTRO}-tf \
      ros-${ROS_DISTRO}-pcl-ros && \
    rm -rf /var/lib/apt/lists/* && \
    mkdir -p $CATKIN_WS/src/A-LOAM/

# WORKDIR $CATKIN_WS/src

# Copy A-LOAM
COPY ./ $CATKIN_WS/src/A-LOAM/

# Build A-LOAM
WORKDIR $CATKIN_WS

ENV PYTHONIOENCODING UTF-8

RUN catkin config \
      --extend /opt/ros/$ROS_DISTRO \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release && \
    catkin build && \
    sed -i '/exec "$@"/i \
            source "/root/catkin_ws/devel/setup.bash"' /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh       

SHELL [ "/bin/bash" ]

CMD ["roslaunch", "aloam_velodyne", "aloam_velodyne_Carla.launch"]