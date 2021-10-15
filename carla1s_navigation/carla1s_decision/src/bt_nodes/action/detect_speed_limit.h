
#ifndef CARLA1S_ROS_CARLA1S_NAVIGATION_CARLA1S_DECISION_SRC_BT_NODES_ACTION_DETECT_SPEED_LIMIT_H_
#define CARLA1S_ROS_CARLA1S_NAVIGATION_CARLA1S_DECISION_SRC_BT_NODES_ACTION_DETECT_SPEED_LIMIT_H_

#include "carla1s_decision_common.h"

namespace carla1s_decision {

class DetectSpeedLimit : public BT::AsyncActionNode {
 public:
  DetectSpeedLimit(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts() {
    return {
        BT::OutputPort<double>("target_speed"),
        BT::InputPort<double>("default_speed")
    };
  };

 protected:
  BT::NodeStatus tick() override;

 private:
  double default_speed_ = 15.0;

};

}

#endif //CARLA1S_ROS_CARLA1S_NAVIGATION_CARLA1S_DECISION_SRC_BT_NODES_ACTION_DETECT_SPEED_LIMIT_H_
