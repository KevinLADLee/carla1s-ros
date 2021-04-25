//
// Created by kevinlad on 2021/4/14.
//


#include "carla_decision.h"

CarlaDecision::CarlaDecision() {
  nh_ptr_ = boost::make_shared<ros::NodeHandle>();
}

bool CarlaDecision::LoadBehaviorTree(const std::string &filename) {
  std::cout << "Loading behavior tree..." << std::endl;
  // Check and load behavior tree xml file as xml_string
  std::ifstream xml_file(filename);
  if (!xml_file.good()) {
    ROS_ERROR("Couldn't open input XML file: %s", filename.c_str());
    return false;
  }
  auto xml_string = std::string(std::istreambuf_iterator<char>(xml_file),
                                std::istreambuf_iterator<char>());

  // Create the blackboard that will be shared by all of the nodes in the tree
  bt_blackboard_ = BT::Blackboard::create();
  bt_blackboard_->set<ros::NodeHandlePtr>("node_handler", nh_ptr_);

  // Nodes Register
  for (const auto & plugin : plugin_libs_) {
    auto path_to_lib = DetermineLibPath(plugin);
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

  std::cout << "Loading success!" << std::endl;
  return true;
}

void CarlaDecision::Tick() {
  ros::Rate rate(100);
  ros::AsyncSpinner spinner(6);
  spinner.start();
  while (ros::ok()){
    bt_tree_.tickRoot();
    rate.sleep();
  }
}

std::string CarlaDecision::DetermineLibPath(const std::string &plugin_name) {
  auto path = BT::SharedLibrary::getOSName(plugin_name);
  if (CheckSharedLibExist(path)) {
    return std::move(path);
  } else {
    path.insert(0, "../");
    return std::move(path);
  }
}
