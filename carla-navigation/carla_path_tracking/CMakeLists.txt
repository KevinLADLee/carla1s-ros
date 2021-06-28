cmake_minimum_required(VERSION 3.0.2)
project(carla_path_tracking)


set(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++14)
## Find catkin macros and libraries

#set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
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

find_package(Eigen3 REQUIRED)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES carla_drive_with_uncertainty
  # CATKIN_DEPENDS carla_msgs
  # DEPENDS carla_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        # include
        src
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/carla_drive_with_uncertainty.cpp
# )

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(pure_pursuit src/impl/pure_pursuit.cpp )
target_link_libraries(pure_pursuit ${EIGEN3_LIBRARIES})

add_library(pid src/impl/pid.cpp src/impl/pid.h)

add_executable(carla_path_tracking_node src/path_tracking_node.cpp src/path_tracking.cpp)
add_dependencies(carla_path_tracking_node ${catkin_EXPORTED_TARGETS} carla_nav_msgs_generate_messages)
target_link_libraries(carla_path_tracking_node ${catkin_LIBRARIES} pure_pursuit pid ${EIGEN3_LIBRARIES})

