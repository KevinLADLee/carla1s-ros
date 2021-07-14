import os
from glob import glob
ROS_VERSION = int(os.environ['ROS_VERSION'])

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
        packages=['carla_vertical_parking'],
        package_dir={'': 'src'},
    )

setup(**d)