cmake_minimum_required(VERSION 3.0.5)
project(carla_visualization)

find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs geometry_msgs
        derived_object_msgs tf roslaunch)

catkin_python_setup()

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS src/carla_visualization/carla_visualization.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
