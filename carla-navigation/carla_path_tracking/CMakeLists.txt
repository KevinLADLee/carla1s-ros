cmake_minimum_required(VERSION 3.0.2)
project(carla_path_tracking)
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  geometry_msgs
  tf
  ackermann_msgs
  visualization_msgs
  carla_msgs
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES carla_drive_with_uncertainty
  #  CATKIN_DEPENDS roscpp rospy std_msgs
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
#   src/${PROJECT_NAME}/carla_drive_with_uncertainty.cpp
# )

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(pure_pursuit_node src/pure_pursuit.cpp)
target_link_libraries(pure_pursuit_node ${catkin_LIBRARIES})
add_dependencies(pure_pursuit_node carla_msgs_generate_messages)