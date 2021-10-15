#include <std_msgs/Float64.h>
#include "blackboard_handler.h"

namespace carla1s_decision{

BlackboardHandler::BlackboardHandler(BT::Blackboard::Ptr bb_ptr,
                                     ros::NodeHandlePtr nh_ptr,
                                     const std::string &role_name)
    : bb_ptr_(bb_ptr), nh_ptr_(nh_ptr), role_name_(role_name) {


  bb_ptr_->set<bool>("goal_received", false);
  bb_ptr_->set<bool>("need_update_path", false);
  bb_ptr_->set<double>("collision_avoid_speed", -1.0);

  ros_sub_vec_.push_back(nh_ptr_->subscribe<nav_msgs::Odometry>("/carla/" + role_name_ + "/odometry",
                                                     10,
                                                     &BlackboardHandler::OdomCallback,
                                                     this));

  ros_sub_vec_.push_back(nh_ptr_->subscribe<std_msgs::Bool>("/carla1s/"+role_name_+"/fake_perception/traffic_light_passable",
                                                          1,
                                                          &BlackboardHandler::TrafficLightCallback,
                                                          this));

  ros_sub_vec_.push_back(nh_ptr_->subscribe<derived_object_msgs::Object>("/carla1s/"+role_name_+"/fake_perception/detected_object",
                                                                       1,
                                                                       &BlackboardHandler::ObjectCallback,
                                                                       this));

  ros_sub_vec_.push_back(nh_ptr_->subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal",
                                                             1,
                                                             &BlackboardHandler::GoalCallback,
                                                             this));

  ros_sub_vec_.push_back(nh_ptr_->subscribe<std_msgs::Float64>("/carla1s/"+role_name_+"/fake_perception/collision_avoid_speed",
                                                               1,
                                                               &BlackboardHandler::CollisionAvoidSpeedCallback,
                                                               this));

}

BlackboardHandler::~BlackboardHandler() {
  nh_ptr_.reset();
  bb_ptr_.reset();
}

void BlackboardHandler::OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
  bb_ptr_->set<nav_msgs::Odometry>("odom", *odom_msg);
}

void BlackboardHandler::CollisionAvoidSpeedCallback(const std_msgs::Float64ConstPtr &speed_msg) {
  bb_ptr_->set<double>("collision_avoid_speed", speed_msg->data);
}

void BlackboardHandler::TrafficLightCallback(const std_msgs::BoolConstPtr& msg) {
  bb_ptr_->set<bool>("traffic_light_passable", msg->data);
}

void BlackboardHandler::GoalCallback(const geometry_msgs::PoseStampedConstPtr &goal) {
  bb_ptr_->set<geometry_msgs::PoseStamped>("goal", *goal);
  bb_ptr_->set<bool>("goal_received", true);
  ROS_INFO( "BT Blackboard: Received new goal: (%f,%f)", goal->pose.position.x, goal->pose.position.y);
}

void BlackboardHandler::ObjectCallback(const derived_object_msgs::ObjectConstPtr &object_msg) {
//  ROS_INFO("Object id: %d", object_msg->id);
}

}