cmake_minimum_required(VERSION 3.5.1)
project(carla1s_decision)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        actionlib
        nav_msgs
        geometry_msgs
        carla_msgs
        carla1s_msgs
        carla1s_common
        behaviortree_cpp_v3
)

find_package(Eigen3 REQUIRED)

find_package(BehaviorTreeV3)

set(ROS_VERSION $ENV{ROS_VERSION})

catkin_package(
#        INCLUDE_DIRS src
#        LIBRARIES ${PROJECT_NAME}
)

include_directories(
        src
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

# Build BT Nodes to shared library
file(GLOB_RECURSE nodes_src "src/bt_nodes/*/*.cpp")
foreach(node_src ${nodes_src})
    get_filename_component(file_name ${node_src} NAME_WE)
    get_filename_component(dir_path "${node_src}/.." ABSOLUTE)
    get_filename_component(dir_name "${dir_path}" NAME)
    set(target_name bt_${dir_name}_${file_name})
    add_library(${target_name} SHARED ${node_src})
    target_link_libraries(${target_name} BT::behaviortree_cpp_v3 ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES})
    target_compile_definitions(${target_name} PRIVATE BT_PLUGIN_EXPORT)
    add_dependencies(${target_name} carla_msgs_generate_messages carla1s_msgs_generate_messages )
    list(APPEND node_libs ${target_name})
endforeach()

add_executable(${PROJECT_NAME}_node src/carla1s_decision.cpp src/carla1s_decision_node.cpp src/blackboard_handler.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        BT::behaviortree_cpp_v3
        ${node_libs}
)
