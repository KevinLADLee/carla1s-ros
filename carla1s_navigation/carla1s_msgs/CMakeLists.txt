cmake_minimum_required(VERSION 3.12.0)
project(carla1s_msgs)

find_package(ros_environment REQUIRED)

set(ROS_VERSION $ENV{ROS_VERSION})

set(ACTION_FILES
        PathTracking.action
        PathPlanner.action
        ParkingPlanner.action
        )

set(MESSAGE_FILES
        ParkingSpotArray.msg
        ParkingSpot.msg
        PathArray.msg
        )

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        nav_msgs
        geometry_msgs
        diagnostic_msgs
        )

#add_message_files(DIRECTORY msg FILES ${MSG_FILES})

add_message_files(DIRECTORY msg FILES ${MESSAGE_FILES})
add_action_files(DIRECTORY action FILES ${ACTION_FILES})


generate_messages(DEPENDENCIES std_msgs geometry_msgs diagnostic_msgs nav_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs nav_msgs diagnostic_msgs)
