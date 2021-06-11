//
// Created by kevinlad on 2021/4/22.
//

#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_COMMON_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_COMMON_H_

#include <iostream>
#include <fstream>


#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/utils/shared_library.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <sys/stat.h>
#include <dlfcn.h>

#include "bt_node_base/bt_action_node.h"
//#include "bt_node_base/bt_planner_node.h"
#include "util/log.h"
#include "util/bt_conversions.h"

#ifndef NDEBUG
#define DLOG_INFO(INFO) std::cout <<  "[Debug]: " << INFO << std::endl;
#else
#define DLOG_INFO(INFO)
#endif

inline bool CheckSharedLibExist(const std::string& path) {
  auto _handle = dlopen(path.c_str(), RTLD_NOW | RTLD_GLOBAL);
  if (!_handle){
    return false;
  }else{
    return true;
  }
}



#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_COMMON_H_
