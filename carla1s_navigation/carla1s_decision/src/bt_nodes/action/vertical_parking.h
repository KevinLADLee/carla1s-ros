#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_BT_NODES_ACTION_PARKING_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_BT_NODES_ACTION_PARKING_H_

#include "carla1s_decision_common.h"
#include "util/bt_conversions.h"
#include <carla1s_msgs/ParkingPlannerAction.h>

class VerticalParking : public BT::RosActionNode<carla1s_msgs::ParkingPlannerAction> {
 public:
  VerticalParking(const std::string &name, const std::string &action_client_name, const BT::NodeConfiguration &conf);

  VerticalParking() = delete;

  void on_tick() override;

  static BT::PortsList providedPorts(){
    return {
      BT::InputPort<Pose2d>("parking_spot_pose"),
      BT::InputPort<double>("parking_spot_length"),
      BT::InputPort<double>("parking_spot_width")
    };
  };

  BT::NodeStatus on_result(const ResultType &res) override;

};

#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_BT_NODES_ACTION_PARKING_H_
