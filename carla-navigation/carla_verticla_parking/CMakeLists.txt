cmake_minimum_required(VERSION 3.0.2)
project(carla_vertical_parking)

find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

find_package(catkin REQUIRED COMPONENTS rospy roslaunch carla_nav_msgs)

catkin_python_setup()

if(CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch)
endif()

catkin_package(CATKIN_DEPENDS rospy)


include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
        src/carla_vertical_parking/carla_vertical_parking_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

