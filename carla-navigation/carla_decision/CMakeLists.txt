cmake_minimum_required(VERSION 3.0.2)
project(carla_decision)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs behaviortree_cpp_v3)

find_package(BehaviorTreeV3)

set(ROS_VERSION $ENV{ROS_VERSION})

# file(GLOB condition_nodes "src/condition/*.cpp")
# add_library(bt_condition_goal_updated SHARED src/condition/goal_updated.cpp)
# list(APPEND node_libs bt_condition_goal_updated)

# add_library(bt_action_move_to_goal SHARED src/action/move_to_goal.cpp)
# list(APPEND node_libs bt_action_move_to_goal)

# add_library(bt_action_compute_path_to_goal SHARED src/action/compute_path_to_goal.cpp)
# list(APPEND node_libs bt_action_compute_path_to_goal)


# Build BT Nodes to shared library
file(GLOB_RECURSE nodes_src "src/bt_nodes/*/*.cpp")
# message("*************")
foreach(node_src ${nodes_src})
    get_filename_component(file_name ${node_src} NAME_WE)
    get_filename_component(dir_path "${node_src}/.." ABSOLUTE)
    get_filename_component(dir_name "${dir_path}" NAME)
    set(target_name bt_${dir_name}_${file_name})
    # message(${target_name})
    add_library(${target_name} SHARED ${node_src})
    target_link_libraries(${target_name} BT::behaviortree_cpp_v3)
    target_compile_definitions(${target_name} PRIVATE BT_PLUGIN_EXPORT)
    list(APPEND node_libs ${target_name})
endforeach()
# message(${node_libs})
# message("*************")

include_directories(src)

add_executable(test_bt_node src/test_bt.cpp src/test_bt_node.cpp)
target_link_libraries(test_bt_node ${catkin_LIBRARIES}
        BT::behaviortree_cpp_v3
        ${node_libs}
        )
