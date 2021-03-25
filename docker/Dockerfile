
ARG ROS_VERSION='noetic'
ARG CARLA_VERSION='0.9.10.1'
ARG EDD_FILE='carla-0.9.10-py3.7-linux-x86_64.egg'

FROM harbor.isus.tech/carlasim/carla:$CARLA_VERSION$CARLA_BUILD as carla
FROM harbor.isus.tech/osrf/ros:$ROS_VERSION-desktop

ARG ROS_VERSION
ARG MIRROR='http://mirrors.sustech.edu.cn'
ARG PYPI_MIRROR='https://mirrors.aliyun.com'
ARG ROS_MIRROR='http://mirrors.sustech.edu.cn'
RUN sed -i "s@http://.*archive.ubuntu.com@$MIRROR@g" /etc/apt/sources.list && \
    sed -i "s@http://.*security.ubuntu.com@$MIRROR@g" /etc/apt/sources.list && \
    sed -i "s@http://packages.ros.org@$ROS_MIRROR@g" /etc/apt/sources.list.d/ros1-latest.list && \   
    apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-pip  && \
    rm -rf /var/lib/apt/lists/* && \
    pip3 install --no-cache-dir pip -U -i $PYPI_MIRROR/pypi/simple/  && \
    pip3 config set global.index-url $PYPI_MIRROR/pypi/simple/ 


RUN apt-get update && \
    if [ "$ROS_VERSION" = "noetic" ] ; then export PYTHON_SUFFIX=3 ; else export PYTHON_SUFFIX="" ; fi && \
    apt-get install -y --no-install-recommends \
    libpng16-16 \
    libceres-dev \
    ros-$ROS_VERSION-tf \
    ros-$ROS_VERSION-derived-object-msgs \
    ros-$ROS_VERSION-cv-bridge \
    ros-$ROS_VERSION-pcl-conversions \
    ros-$ROS_VERSION-pcl-ros \
    ros-$ROS_VERSION-pointcloud-to-laserscan \
    ros-$ROS_VERSION-map-server \
    ros-$ROS_VERSION-move-base \
    ros-$ROS_VERSION-teb-local-planner \
    python$PYTHON_SUFFIX-osrf-pycommon \
    python$PYTHON_SUFFIX-catkin-tools \
    vim git wget curl \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 --no-cache-dir install --upgrade \
        networkx distro pygame simple-pid numpy==1.18.4
ARG CARLA_VERSION

COPY --from=carla --chown=root /home/carla/PythonAPI /opt/carla/PythonAPI

COPY ros-bridge /opt/carla-ros-bridge/src/ros-bridge
# RUN git clone https://github.com/carla-simulator/ros-bridge.git --depth=1 /opt/carla-ros-bridge/src/ros-bridge
RUN git clone https://gitlab.isus.tech/kevinlad/ackermann_msgs.git --depth=1 /opt/carla-ros-bridge/src/ros-bridge/ackermann_msgs && \
    git clone https://gitlab.isus.tech/kevinlad/astuff_sensor_msgs.git --depth=1 /opt/carla-ros-bridge/src/ros-bridge/astuff_sensor_msgs && \
    cd /opt/carla-ros-bridge/src/ros-bridge/ && \
    mv astuff_sensor_msgs/derived_object_msgs derived_object_msgs && \
    rm -rf astuff_sensor_msgs
COPY carla-navigation /opt/carla-ros-bridge/src/carla-navigation

RUN if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; else export USE_PROC=$(($(nproc)/2)) ; fi && \
    cd /opt/ && \
    wget -qc http://cdn.isus.tech/cdn/library/gtsam-4.0.3.tar.gz && \
    tar -zxvf gtsam-4.0.3.tar.gz && \
    cd gtsam-4.0.3 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j${USE_PROC} && \
    make install


ARG EDD_FILE
RUN /bin/bash -c 'source /opt/ros/$ROS_VERSION/setup.bash; cd /opt/carla-ros-bridge; catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release' && \ 
    ln -s /usr/bin/python3 /usr/bin/python && \
    echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /opt/carla-ros-bridge/devel/setup.bash" >> /root/.bashrc && \
    echo "export PYTHONPATH=\$PYTHONPATH:/opt/carla/PythonAPI/carla/dist/$EDD_FILE:/opt/carla/PythonAPI/carla/" >> /root/.bashrc 

# replace entrypoint
COPY ./docker/content/ros_entrypoint.sh /

COPY ./docker/scripts /

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

SHELL [ "/bin/bash" ]

CMD ["/bin/bash", "/carla_ego.sh"]
