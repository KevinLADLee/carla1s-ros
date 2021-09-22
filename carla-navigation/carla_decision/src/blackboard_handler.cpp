#include "blackboard_handler.h"

namespace carla_decision{

BlackboardHandler::BlackboardHandler(BT::Blackboard::Ptr bb_ptr,
                                     ros::NodeHandlePtr nh_ptr,
                                     const std::string &role_name)
    : bb_ptr_(bb_ptr), nh_ptr_(nh_ptr), role_name_(role_name) {


  bb_ptr_->set<bool>("goal_received", false);

  odom_sub_ = nh_ptr_->subscribe<nav_msgs::Odometry>("/carla/" + role_name_ + "/odometry",
                                                     10,
                                                     boost::bind(&BlackboardHandler::OdomCallback, this, _1));

  traffic_light_sub_ = nh_ptr_->subscribe<std_msgs::Bool>("/carla/"+role_name_+"/fake_perception/traffic_light_passable", 1, &BlackboardHandler::TrafficLightCallback, this);

  goal_sub_ = nh_ptr_->subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &BlackboardHandler::GoalCallback, this);

}

BlackboardHandler::~BlackboardHandler() {
  nh_ptr_.reset();
  bb_ptr_.reset();
}

void BlackboardHandler::OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
  bb_ptr_->set<nav_msgs::Odometry>("odom", *odom_msg);
}

void BlackboardHandler::TrafficLightCallback(const std_msgs::BoolConstPtr& msg) {
  bb_ptr_->set<bool>("traffic_light_passable", msg->data);
}

void BlackboardHandler::GoalCallback(const geometry_msgs::PoseStampedConstPtr &goal) {
  bb_ptr_->set<geometry_msgs::PoseStamped>("goal", *goal);
  bb_ptr_->set<bool>("goal_received", true);
  ROS_INFO( "BT Blackboard: Received new goal: (%f,%f)", goal->pose.position.x, goal->pose.position.y);
}

}