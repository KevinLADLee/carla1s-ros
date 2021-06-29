
#include "carla_decision.h"
#include <ros/package.h>

int main(int argc, char** argv){

  ros::init(argc, argv, "carla_decision");
  CarlaDecision carla_bt_tree;

  std::string xml_base_path;
  if(argc > 1) {
    xml_base_path = std::string(argv[1]) + "/trees/";
  } else{
    xml_base_path = ros::package::getPath("carla_decision") + "/trees/";
  }
  carla_bt_tree.LoadBehaviorTree(xml_base_path);
  carla_bt_tree.Tick();

  return 0;
}