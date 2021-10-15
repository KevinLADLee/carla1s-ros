
#ifndef CARLA1S_ROS_CARLA1S_NAVIGATION_CARLA1S_DECISION_SRC_BT_NODES_ACTION_DETECT_VEHICLE_H_
#define CARLA1S_ROS_CARLA1S_NAVIGATION_CARLA1S_DECISION_SRC_BT_NODES_ACTION_DETECT_VEHICLE_H_

#include "carla1s_decision_common.h"

namespace carla1s_decision {

class DetectVehicle : public BT::AsyncActionNode {
 public:
  DetectVehicle(const std::string &name, const BT::NodeConfiguration &config);

  DetectVehicle() = delete;

  static BT::PortsList providedPorts() {
    return {
        BT::BidirectionalPort<double>("target_speed")
    };
  };

 protected:
  BT::NodeStatus tick() override;

};

}

#endif //CARLA1S_ROS_CARLA1S_NAVIGATION_CARLA1S_DECISION_SRC_BT_NODES_ACTION_DETECT_VEHICLE_H_
