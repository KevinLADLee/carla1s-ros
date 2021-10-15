
#ifndef CARLA1S_ROS_CARLA1S_NAVIGATION_CARLA1S_DECISION_SRC_BT_NODES_ACTION_DETECHT_TRAFFIC_LIGHT_H_
#define CARLA1S_ROS_CARLA1S_NAVIGATION_CARLA1S_DECISION_SRC_BT_NODES_ACTION_DETECHT_TRAFFIC_LIGHT_H_

#include "carla1s_decision_common.h"

namespace carla1s_decision {

class DetectTrafficLight : public BT::AsyncActionNode {
 public:
  DetectTrafficLight(const std::string &name,
                     const BT::NodeConfiguration &conf);

  DetectTrafficLight() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() {
    return {
        BT::BidirectionalPort<double>("target_speed")
    };
  };

 private:
  bool traffic_light_passable_ = true;
}
;
}

#endif //CARLA1S_ROS_CARLA1S_NAVIGATION_CARLA1S_DECISION_SRC_BT_NODES_ACTION_DETECHT_TRAFFIC_LIGHT_H_
