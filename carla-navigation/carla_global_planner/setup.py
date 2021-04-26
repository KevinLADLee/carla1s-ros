# -*- coding: utf-8 -*-

"""
Setup for carla_waypoint_publisher
"""
import os
from glob import glob
ROS_VERSION = int(os.environ['ROS_VERSION'])

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['carla_global_planner'],
    package_dir={'': 'src'},
)

setup(**d)

