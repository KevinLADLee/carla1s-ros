
#ifndef CARLA1S_ROS_CARLA1S_NAVIGATION_CARLA1S_DECISION_SRC_BT_NODES_DECORATOR_BLACKBOARD_CHECK_BOOL_H_
#define CARLA1S_ROS_CARLA1S_NAVIGATION_CARLA1S_DECISION_SRC_BT_NODES_DECORATOR_BLACKBOARD_CHECK_BOOL_H_

#include "behaviortree_cpp_v3/decorator_node.h"

namespace carla1s_decision {

class BlackboardCheckBool : public BT::DecoratorNode {
 public:
  /**
   * @brief A constructor for carla1s_decision::BlackboardCheckBool
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  BlackboardCheckBool(
      const std::string & name,
      const BT::NodeConfiguration & conf);

  BlackboardCheckBool() = delete;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<bool>("key", false, "BB Value to check")
    };
  }

 private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

};

} // namespace carla1s_decision

#endif //CARLA1S_ROS_CARLA1S_NAVIGATION_CARLA1S_DECISION_SRC_BT_NODES_DECORATOR_BLACKBOARD_CHECK_BOOL_H_
