#!/bin/bash

source /opt/ros/noetic/setup.bash
source /opt/carla-ros-bridge/devel/setup.bash
export PYTHONPATH=$PYTHONPATH:/opt/carla/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg

python3 /opt/carla/PythonAPI/examples/spawn_npc.py -n 45 -w 5