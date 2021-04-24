
#include "carla_decision.h"
#include <ros/package.h>

int main(int argc, char** argv){

  ros::init(argc, argv, "carla_decision");
  CarlaDecision carla_bt_tree;

  std::string xml_path;
  if(argc > 1) {
    xml_path = argv[1];
  } else{
    xml_path = ros::package::getPath("carla_decision") + "/trees/goal_behavior.xml";
  }
  carla_bt_tree.LoadBehaviorTree(xml_path);
  carla_bt_tree.Tick();

  return 0;
}