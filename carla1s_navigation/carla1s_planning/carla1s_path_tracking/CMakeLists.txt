cmake_minimum_required(VERSION 3.0.2)
project(carla1s_path_tracking)

#set(CMAKE_BUILD_TYPE RelWithDebug)
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
        carla1s_common
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

#add_library(pid_lon_controller src/lon_controller/pid_lon_controller.cpp)
#target_link_libraries(pid_lon_controller ${EIGEN3_LIBRARIES})
#
#add_library(pid_lat_controller src/lat_controller/pid_lat_controller.cpp)
#target_link_libraries(pid_lat_controller ${EIGEN3_LIBRARIES})
#
#add_library(pure_pursuit src/lat_controller/pure_pursuit.cpp)
#target_link_libraries(pure_pursuit ${EIGEN3_LIBRARIES})

#add_executable(test_pid src/lon_controller/pid_lon_controller_test.cpp src/lon_controller/pid_lon_controller.cpp)
#add_dependencies(test_pid ${catkin_EXPORTED_TARGETS} carla1s_msgs_generate_messages)
#target_link_libraries(test_pid  ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(path_tracking_node
        src/path_tracking_node.cpp
        src/path_tracking.cpp
        src/lat_controller/pure_pursuit.cpp
        src/lon_controller/pid_lon_controller.cpp
        src/lat_controller/pid_lat_controller.cpp
        src/common/path_handler.cpp
        )
add_dependencies(path_tracking_node carla_msgs_generate_messages carla1s_msgs_generate_messages)
target_link_libraries(path_tracking_node
#        pid_lon_controller
#        pid_lat_controller
#        pure_pursuit
        ${catkin_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        )

