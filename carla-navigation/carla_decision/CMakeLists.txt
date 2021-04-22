cmake_minimum_required(VERSION 3.5.1)
project(carla_decision)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs behaviortree_cpp_v3)

find_package(BehaviorTreeV3)

set(ROS_VERSION $ENV{ROS_VERSION})

catkin_package(
#        INCLUDE_DIRS src
#        LIBRARIES ${PROJECT_NAME}
)


# Build BT Nodes to shared library
file(GLOB_RECURSE nodes_src "src/bt_nodes/*/*.cpp")
foreach(node_src ${nodes_src})
    get_filename_component(file_name ${node_src} NAME_WE)
    get_filename_component(dir_path "${node_src}/.." ABSOLUTE)
    get_filename_component(dir_name "${dir_path}" NAME)
    set(target_name bt_${dir_name}_${file_name})
    add_library(${target_name} SHARED ${node_src})
    target_link_libraries(${target_name} BT::behaviortree_cpp_v3)
    target_compile_definitions(${target_name} PRIVATE BT_PLUGIN_EXPORT)
#    if (CMAKE_BUILD_TYPE EQUAL "DEBUG")
#    set_target_properties(${target_name} PROPERTIES
#                          LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
#    else()
#        set_target_properties(${target_name} PROPERTIES
#                LIBRARY_OUTPUT_DIRECTORY ${CATKIN_PACKAGE_LIB_DESTINATION})
#    endif()
    list(APPEND node_libs ${target_name})
endforeach()

include_directories(src)

add_executable(${PROJECT_NAME}_node src/carla_decision.cpp src/carla_decision_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES}
        BT::behaviortree_cpp_v3
        ${node_libs}
)