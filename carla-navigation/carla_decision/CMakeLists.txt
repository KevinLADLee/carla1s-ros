cmake_minimum_required(VERSION 3.0.2)
project(carla_decision)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs behaviortree_cpp_v3)

find_package(BehaviorTreeV3)

set(ROS_VERSION $ENV{ROS_VERSION})

add_executable(test_bt_node src/test_bt.cpp src/test_bt_node.cpp)
target_link_libraries(test_bt_node ${catkin_LIBRARIES} BT::behaviortree_cpp_v3)