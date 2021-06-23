cmake_minimum_required(VERSION 3.0.2)
project(carla_global_planner)

find_package(ros_environment REQUIRED)

set(ROS_VERSION $ENV{ROS_VERSION})

find_package(catkin REQUIRED COMPONENTS rospy roslaunch carla_nav_msgs)

catkin_python_setup()

if(CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch)
endif()

catkin_package(CATKIN_DEPENDS rospy)

catkin_install_python(
        PROGRAMS
        src/carla_global_planner/carla_global_planner.py
        src/carla_global_planner/carla_map_visualization.py
        src/carla_global_planner/reeds_ros_inter.py
        src/carla_global_planner/reeds_shepp.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

