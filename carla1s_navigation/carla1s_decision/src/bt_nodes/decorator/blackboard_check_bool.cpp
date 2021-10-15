
#include "blackboard_check_bool.h"

carla1s_decision::BlackboardCheckBool::BlackboardCheckBool(const std::string &name, const BT::NodeConfiguration &config)
    : DecoratorNode(name, config) {}

BT::NodeStatus carla1s_decision::BlackboardCheckBool::tick() {

  setStatus(BT::NodeStatus::RUNNING);
  std::string port_name;
  getInput<std::string>("key", port_name);
  auto value = config().blackboard->get<bool>(port_name);
//  std::cout << "Check: " << value << std::endl;
  if(!value) {
    return BT::NodeStatus::SUCCESS;
  }

  const BT::NodeStatus child_state = child_node_->executeTick();

  switch (child_state) {
    case BT::NodeStatus::RUNNING:
      return BT::NodeStatus::RUNNING;
    case BT::NodeStatus::SUCCESS:
      return BT::NodeStatus::SUCCESS;
    case BT::NodeStatus::FAILURE:
    default:
      return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<carla1s_decision::BlackboardCheckBool>("BlackboardCheckBool");
}

