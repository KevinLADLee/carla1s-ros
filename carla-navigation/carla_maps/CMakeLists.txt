cmake_minimum_required(VERSION 3.0.2)
project(carla_maps)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  std_msgs
  carla_msgs
)

find_package(PugiXML REQUIRED)

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
#  LIBRARIES carla_fake_perception
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

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(carla_maps_node src/carla_maps_node.cpp src/carla_maps.cpp)

target_link_libraries(carla_maps_node
        ${catkin_LIBRARIES}
        ${PUGIXML_LIBRARY}
        )

add_dependencies(carla_maps_node carla_nav_msgs_generate_messages carla_msgs_generate_messages)

