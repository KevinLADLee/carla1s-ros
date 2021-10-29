FROM harbor.isus.tech/carla1s/container-base:1.1.0

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && \
    apt-get install -y ros-noetic-desktop-full && \
    apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential && \
    apt-get install -y libpng16-16 ros-noetic-tf ros-noetic-derived-object-msgs ros-noetic-cv-bridge ros-noetic-pcl-conversions ros-noetic-pcl-ros \
     ros-noetic-pointcloud-to-laserscan ros-noetic-map-server ros-noetic-ackermann-msgs ros-noetic-derived-object-msgs ros-noetic-behaviortree-cpp-v3 \
     vim git wget curl ninja-build libomp-dev && \
    pip3 --no-cache-dir install --upgrade networkx distro pygame simple-pid numpy==1.18.4 transforms3d pep8 autopep8 cmake_format==0.6.11 pylint pexpect scipy empy catkin_pkg netifaces defusedxml

WORKDIR /ros-agent
ARG GITLAB_USERNAME
ARG GITLAB_PASSWORD
RUN mkdir -p /ros-agent/src && \
    cd /ros-agent/src && \
    git clone --recurse-submodules -b avp_demo https://${GITLAB_USERNAME}:${GITLAB_PASSWORD}@gitlab.isus.tech/carla1s/ros-agent/carla1s-ros.git && \
    cd /ros-agent && \
    source /opt/ros/noetic/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release --use-ninja