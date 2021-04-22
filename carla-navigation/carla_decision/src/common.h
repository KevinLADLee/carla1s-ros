//
// Created by kevinlad on 2021/4/22.
//

#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_COMMON_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_COMMON_H_

#include <iostream>

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseStamped.h>

#include "util/log.h"

#ifndef NDEBUG
#define DLOG_INFO(INFO) std::cout << "[Debug]: " << INFO << std::endl;
#else
#define DLOG_INFO(INFO)
#endif



#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_COMMON_H_
