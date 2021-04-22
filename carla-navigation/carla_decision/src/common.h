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
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseStamped.h>

#include <sys/stat.h>

#include "util/log.h"

#ifndef NDEBUG
#define DLOG_INFO(INFO) std::cout << "[Debug]: " << INFO << std::endl;
#else
#define DLOG_INFO(INFO)
#endif


inline bool CheckFile(const std::string& name) {
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}



#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_COMMON_H_
