ARG CARLA_VERSION='0.9.10.1'
ARG ROS_VERSION='noetic'
ARG CARLA_BUILD=''
ARG EDD_FILE='carla-0.9.10-py3.7-linux-x86_64.egg'

FROM harbor.isus.tech/carlasim/carla:$CARLA_VERSION$CARLA_BUILD as carla
#FROM harbor.isus.tech/library/ros:$ROS_VERSION-ros-base
FROM harbor.isus.tech/osrf/ros:noetic-desktop-full-focal

ARG ROS_VERSION

RUN sed -i "s@http://.*archive.ubuntu.com@http://mirrors.huaweicloud.com@g" /etc/apt/sources.list && \
    sed -i "s@http://.*security.ubuntu.com@http://mirrors.huaweicloud.com@g" /etc/apt/sources.list && \
    sed -i "s@http://packages.ros.org/ros/ubuntu@https://mirror.tuna.tsinghua.edu.cn/ros/ubuntu/@g" /etc/apt/sources.list.d/ros1-latest.list && \
    apt-get update && \
    if [ "$ROS_VERSION" = "noetic" ] ; then export PYTHON_SUFFIX=3 ; else export PYTHON_SUFFIX="" ; fi && \
    apt-get install -y --no-install-recommends \
    libpng16-16 \
    ros-$ROS_VERSION-ackermann-msgs \
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
    python3-pip  \
    vim \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 --no-cache-dir install --upgrade --trusted-host https://mirrors.huaweicloud.com -i https://mirrors.huaweicloud.com/repository/pypi/simple \
        networkx distro pygame simple-pid numpy==1.18.4
ARG CARLA_VERSION

COPY --from=carla --chown=root /home/carla/PythonAPI /opt/carla/PythonAPI

COPY carla_ackermann_control /opt/carla-ros-bridge/src/carla_ackermann_control
COPY carla_common /opt/carla-ros-bridge/src/carla_common
COPY carla_ego_vehicle /opt/carla-ros-bridge/src/carla_ego_vehicle
COPY carla_infrastructure /opt/carla-ros-bridge/src/carla_infrastructure
COPY carla_manual_control /opt/carla-ros-bridge/src/carla_manual_control
COPY carla_msgs /opt/carla-ros-bridge/src/carla_msgs
COPY carla_ros_bridge /opt/carla-ros-bridge/src/carla_ros_bridge
COPY carla_ros_scenario_runner /opt/carla-ros-bridge/src/carla_ros_scenario_runner
COPY carla_ros_scenario_runner_types /opt/carla-ros-bridge/src/carla_ros_scenario_runner_types
COPY carla_spectator_camera /opt/carla-ros-bridge/src/carla_spectator_camera
COPY carla_twist_to_control /opt/carla-ros-bridge/src/carla_twist_to_control
COPY carla_walker_agent /opt/carla-ros-bridge/src/carla_walker_agent
COPY carla_waypoint_publisher /opt/carla-ros-bridge/src/carla_waypoint_publisher
COPY carla_waypoint_types /opt/carla-ros-bridge/src/carla_waypoint_types
COPY pcl_recorder /opt/carla-ros-bridge/src/pcl_recorder
COPY carla_ad_demo /opt/carla-ros-bridge/src/carla_ad_demo
COPY carla_ad_agent /opt/carla-ros-bridge/src/carla_ad_agent
COPY rviz_carla_plugin /opt/carla-ros-bridge/src/rviz_carla_plugin
COPY rqt_carla_control /opt/carla-ros-bridge/src/rqt_carla_control
COPY carla_2d_nav /opt/carla-ros-bridge/src/carla_2d_nav

RUN /bin/bash -c 'source /opt/ros/$ROS_VERSION/setup.bash; cd /opt/carla-ros-bridge; catkin_make -DCMAKE_BUILD_TYPE=Release' && \ 
#/bin/bash -c 'source /opt/ros/$ROS_VERSION/setup.bash; cd /opt/carla-ros-bridge; catkin config --install; catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release' && \
    ln -s /usr/bin/python3 /usr/bin/python && \
    echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /opt/carla-ros-bridge/devel/setup.bash" >> /root/.bashrc && \
    echo "export PYTHONPATH=\$PYTHONPATH:/opt/carla/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg" >> /root/.bashrc 

# replace entrypoint
COPY ./docker/content/ros_entrypoint.sh /

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

SHELL [ "/bin/bash" ]

CMD ["roslaunch", "carla_ros_bridge", "carla_ros_bridge.launch"]
