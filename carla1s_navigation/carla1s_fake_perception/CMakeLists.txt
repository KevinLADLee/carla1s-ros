cmake_minimum_required(VERSION 3.0.2)
project(carla1s_fake_perception)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  std_msgs
  derived_object_msgs
  message_filters
  visualization_msgs

  carla_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES carla1s_fake_perception
#  CATKIN_DEPENDS carla_msgs geometry_msgs nav_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/carla1s_fake_perception.cpp
# )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(traffic_light_node src/traffic_light_perception.cpp src/traffic_light_node.cpp)
target_link_libraries(traffic_light_node
        ${catkin_LIBRARIES}
        )
add_dependencies(traffic_light_node carla_msgs_generate_messages)

add_executable(object_detection_node src/object_detection.cpp)
target_link_libraries(object_detection_node
        ${catkin_LIBRARIES}
        )
add_dependencies(traffic_light_node carla_msgs_generate_messages)