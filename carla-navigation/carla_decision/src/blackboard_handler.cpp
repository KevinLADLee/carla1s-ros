//
// Created by kevinlad on 2021/4/27.
//

#include "blackboard_handler.h"

namespace carla_decision{

BlackboardHandler::BlackboardHandler(BT::Blackboard::Ptr bb_ptr,
                                     ros::NodeHandlePtr nh_ptr,
                                     const std::string &role_name)
    : bb_ptr_(bb_ptr), nh_ptr_(nh_ptr), role_name_(role_name) {

  odom_sub_ = nh_ptr_->subscribe<nav_msgs::Odometry>("/carla/" + role_name_ + "/odometry",
                                                     10,
                                                     boost::bind(&BlackboardHandler::OdomCallback, this, _1));
}

void BlackboardHandler::OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
  bb_ptr_->set<nav_msgs::Odometry>("odom", *odom_msg);
}

}