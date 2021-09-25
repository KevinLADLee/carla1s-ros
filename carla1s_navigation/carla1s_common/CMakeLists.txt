project(carla1s_common)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        carla_msgs
        carla1s_msgs
        actionlib
        )

find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS
        include
)

include_directories(include)

# add_executable(test_math src/test_math.cpp)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )