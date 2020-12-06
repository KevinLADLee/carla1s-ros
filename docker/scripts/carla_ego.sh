#!/bin/bash

source /opt/ros/noetic/setup.bash
source /opt/carla-ros-bridge/devel/setup.bash
export PYTHONPATH=$PYTHONPATH:/opt/carla/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg

roslaunch carla_2d_nav carla_ego.launch & 

sleep 3

python3 /opt/carla/PythonAPI/examples/spawn_npc.py -n 30  &

sleep 3 

rostopic pub /carla/ego_vehicle/enable_autopilot std_msgs/Bool "data: true"