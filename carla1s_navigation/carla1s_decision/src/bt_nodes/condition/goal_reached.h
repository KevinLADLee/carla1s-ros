
#ifndef CARLA1S_ROS_CARLA1S_NAVIGATION_CARLA1S_DECISION_SRC_BT_NODES_CONDITION_GOAL_REACHED_H_
#define CARLA1S_ROS_CARLA1S_NAVIGATION_CARLA1S_DECISION_SRC_BT_NODES_CONDITION_GOAL_REACHED_H_

#include "carla1s_decision_common.h"

namespace carla1s_decision {

class GoalReached : public BT::ConditionNode {
 public:
  GoalReached(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts() {
    return {
        BT::BidirectionalPort<geometry_msgs::PoseStamped>("goal"),
        BT::InputPort<double>("goal_radius")
    };
  };

 protected:
  BT::NodeStatus tick() override;

 private:
  bool first_ = true;

};

}

#endif //CARLA1S_ROS_CARLA1S_NAVIGATION_CARLA1S_DECISION_SRC_BT_NODES_CONDITION_GOAL_REACHED_H_
