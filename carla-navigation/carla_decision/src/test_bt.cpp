//
// Created by kevinlad on 2021/4/14.
//

#include "test_bt.h"

CarlaDecision::CarlaDecision() {
  goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &CarlaDecision::GoalCallback, this);
}

bool CarlaDecision::LoadBehaviorTree(const std::string &filename) {
  std::ifstream xml_file(filename);
  if (!xml_file.good()) {
    ROS_ERROR("Couldn't open input XML file: %s", filename.c_str());
    return false;
  }

  auto xml_string = std::string(
      std::istreambuf_iterator<char>(xml_file),
      std::istreambuf_iterator<char>());

  // Create the blackboard that will be shared by all of the nodes in the tree
  bt_blackboard_ = BT::Blackboard::create();

  // Create the Behavior Tree from the XML input
  bt_factory_.registerNodeType<GoalUpdatedCondition>("GoalUpdated");
  bt_factory_.registerNodeType<MoveToGoal>("MoveToGoal");
  bt_factory_.registerSimpleAction("StopAndWait", std::bind(StopAndWait));
  bt_factory_.registerNodeType<CheckTrafficLight>("CheckTrafficLight");
  try {
    bt_tree_ = bt_factory_.createTreeFromText(xml_string, bt_blackboard_);
  } catch (BT::RuntimeError & exp) {
    ROS_ERROR("%s: %s", filename.c_str(), exp.what());
    return false;
  }

  // This logger publish status changes using ZeroMQ. Used by Groot
  publisher_zmq_ptr_ = std::make_shared<BT::PublisherZMQ>(bt_tree_);
  return true;
}

void CarlaDecision::Tick() {
  ros::Rate rate(10);
  ros::AsyncSpinner spinner(4);
  spinner.start();
  while (ros::ok()){
    bt_tree_.tickRoot();
    rate.sleep();
  }
}

void CarlaDecision::GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal) {
  bt_blackboard_->set<geometry_msgs::PoseStamped>("goal", *goal);
}
