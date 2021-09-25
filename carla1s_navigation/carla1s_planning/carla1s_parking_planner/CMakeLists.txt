cmake_minimum_required(VERSION 3.0.2)
project(carla1s_parking_planner)

find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

find_package(catkin REQUIRED COMPONENTS rospy roslaunch carla1s_msgs)

catkin_python_setup()

if(CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch)
endif()

catkin_package(CATKIN_DEPENDS rospy)


include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
        src/carla1s_parking_planner/carla1s_parking_planner_node.py
        src/carla1s_parking_planner/carla1s_parking_planner_tracking_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

