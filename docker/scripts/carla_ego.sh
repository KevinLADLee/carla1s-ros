#!/bin/bash

source /opt/ros/noetic/setup.bash
source /opt/carla-ros-bridge/devel/setup.bash
export PYTHONPATH=$PYTHONPATH:/opt/carla/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg

python3 /opt/carla/PythonAPI/examples/spawn_npc.py -n 30 -w 10  &

sleep 3

roslaunch carla_2d_nav carla_example_ego_vehicle.launch

# sleep 3 

# rostopic pub /carla/ego_vehicle/enable_autopilot std_msgs/Bool "data: true"
