#include "stop_and_wait.h"

StopAndWait::StopAndWait(const std::string &name, const BT::NodeConfiguration &config) : AsyncActionNode(name,
                                                                                                         config) {}

BT::NodeStatus StopAndWait::tick() {
  std::cout << "StopAndWait" << std::endl;
  return BT::NodeStatus::SUCCESS;
}


BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<StopAndWait>("StopAndWait");
}
