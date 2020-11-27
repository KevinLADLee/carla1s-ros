ARG CARLA_VERSION='0.9.10.1'
ARG ROS_VERSION='noetic'
ARG CARLA_BUILD=''
ARG EDD_FILE='carla-0.9.10-py3.7-linux-x86_64.egg'

FROM harbor.isus.tech/carlasim/carla:$CARLA_VERSION$CARLA_BUILD as carla

FROM harbor.isus.tech/library/ros:$ROS_VERSION-ros-base

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
    python$PYTHON_SUFFIX-osrf-pycommon \
    python$PYTHON_SUFFIX-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

ARG CARLA_VERSION

COPY --from=carla --chown=root /home/carla/PythonAPI /opt/carla/PythonAPI
ENV PYTHONPATH=/opt/carla/PythonAPI/carla/dist/$EGG_FILE

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

RUN /bin/bash -c 'source /opt/ros/$ROS_VERSION/setup.bash; cd /opt/carla-ros-bridge; catkin config --install; catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release' && \
    ln -s /usr/bin/python3 /usr/bin/python && \
    echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /opt/carla-ros-bridge/devel/setup.bash" >> /root/.bashrc

# replace entrypoint
COPY ./docker/content/ros_entrypoint.sh /

CMD ["roslaunch", "carla_ros_bridge", "carla_ros_bridge.launch"]
