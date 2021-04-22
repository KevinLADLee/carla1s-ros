//
// Created by kevinlad on 2021/4/14.
//


#include "carla_decision.h"

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

  // Nodes Register
  const std::vector<std::string> plugin_libs = {
      "bt_action_stop_and_wait",
//      "bt_action_compute_path_to_goal",
      "bt_action_move_to_goal",
      "bt_condition_goal_updated",
      "bt_condition_check_traffic_light"
  };
  for (const auto & p : plugin_libs) {
    auto path_to_lib = BT::SharedLibrary::getOSName(p);
    if(CheckFile(path_to_lib)){
      path_to_lib.insert(0, "../");
    }
    bt_factory_.registerFromPlugin(path_to_lib);
  }

  // Create the Behavior Tree from the XML input
  try {
    bt_tree_ = bt_factory_.createTreeFromText(xml_string, bt_blackboard_);
  } catch (BT::RuntimeError & exp) {
    ROS_ERROR("%s: %s", filename.c_str(), exp.what());
    return false;
  }

  // This logger publish status changes using ZeroMQ. Used by Groot
  publisher_zmq_ptr_ = std::make_unique<BT::PublisherZMQ>(bt_tree_);
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
