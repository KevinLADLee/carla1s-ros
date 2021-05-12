
ARG ROS_VERSION='noetic'
ARG CARLA_VERSION='0.9.11'
ARG EDD_FILE='carla-0.9.11-py3.7-linux-x86_64.egg'

FROM harbor.isus.tech/carlasim/carla:$CARLA_VERSION$CARLA_BUILD as carla
FROM harbor.isus.tech/carla-ros/gtsam-builder:latest as gtsam_builder
FROM harbor.isus.tech/osrf/ros:$ROS_VERSION-desktop

ARG ROS_VERSION
ARG MIRROR='http://mirrors.cra.moe'
ARG ROS_MIRROR='http://mirrors.cra.moe'
RUN sed -i "s@http://.*archive.ubuntu.com@$MIRROR@g" /etc/apt/sources.list && \
    sed -i "s@http://.*security.ubuntu.com@$MIRROR@g" /etc/apt/sources.list && \
    sed -i "s@http://packages.ros.org@$ROS_MIRROR@g" /etc/apt/sources.list.d/ros1-latest.list && \   
    apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-pip  && \
    pip3 install --no-cache-dir pip -U -i https://mirrors.aliyun.com/pypi/simple/ --trusted-host https://mirrors.aliyun.com && \
    pip3 config set global.index-url https://mirrors.aliyun.com/pypi/simple/ && \
    pip3 config set install.trusted-host mirrors.aliyun.com

RUN apt-get update && \
    if [ "$ROS_VERSION" = "noetic" ] ; then export PYTHON_SUFFIX=3 ; else export PYTHON_SUFFIX="" ; fi && \
    apt-get install -y --no-install-recommends \
    libpng16-16 \
    ros-$ROS_VERSION-tf \
    ros-$ROS_VERSION-derived-object-msgs \
    ros-$ROS_VERSION-cv-bridge \
    ros-$ROS_VERSION-pcl-conversions \
    ros-$ROS_VERSION-pcl-ros \
    ros-$ROS_VERSION-pointcloud-to-laserscan \
    ros-$ROS_VERSION-map-server \
    ros-$ROS_VERSION-ackermann-msgs \
    ros-$ROS_VERSION-derived-object-msgs \
    ros-$ROS_VERSION-behaviortree-cpp-v3 \
    vim git wget curl \
    ninja-build \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 --no-cache-dir install --upgrade \
        networkx distro pygame simple-pid numpy==1.18.4 transforms3d
ARG CARLA_VERSION

COPY --from=carla --chown=root /home/carla/PythonAPI /opt/carla/PythonAPI
COPY --from=gtsam_builder --chown=root /opt/gtsam-install /usr/local

# COPY ros-bridge /opt/carla-ros-bridge/src/ros-bridge
# RUN cd /opt/carla-ros-bridge/src/ros-bridge
# RUN git clone https://github.com/carla-simulator/ros-bridge.git --depth=1 /opt/carla-ros-bridge/src/ros-bridge
#COPY carla-navigation /opt/carla-ros-bridge/src/carla-navigation

COPY . /opt/carla-ros-bridge/src/carla-ros-bridge
RUN cd /opt/carla-ros-bridge/src/carla-ros-bridge && \
    git submodule update --init

ARG EDD_FILE

RUN if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; else export USE_PROC=$(($(nproc)/2)) ; fi && \
    /bin/bash -c 'source /opt/ros/$ROS_VERSION/setup.bash; cd /opt/carla-ros-bridge; catkin_make -DCMAKE_BUILD_TYPE=Release --use-ninja -j${USE_PROC}' && \
    ln -s /usr/bin/python3 /usr/bin/python && \
    echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /opt/carla-ros-bridge/devel/setup.bash" >> /root/.bashrc && \
    echo "export PYTHONPATH=\$PYTHONPATH:/opt/carla/PythonAPI/carla/dist/$EDD_FILE:/opt/carla/PythonAPI/carla/" >> /root/.bashrc 

# replace entrypoint
COPY ./docker/content/ros_entrypoint.sh /

COPY ./docker/scripts /opt/carla-ros-bridge

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

WORKDIR /opt/carla-ros-bridge
SHELL [ "/bin/bash" ]