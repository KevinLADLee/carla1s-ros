
#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_BT_NODES_ACTION_TRACKING_PATH_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_BT_NODES_ACTION_TRACKING_PATH_H_

#include "carla1s_decision_common.h"

#include <carla1s_msgs/PathTrackingAction.h>

class TrackingPath : public BT::RosActionNode<carla1s_msgs::PathTrackingAction> {
 public:
  TrackingPath(const std::string &name, const std::string &action_client_name, const BT::NodeConfiguration &conf);
  TrackingPath() = delete;

  void on_tick() override;

  static BT::PortsList providedPorts(){
    return {
        BT::InputPort<carla1s_msgs::PathArray>("path"),
        BT::InputPort<double>("target_speed")
    };
  };

  BT::NodeStatus on_result(const ResultType &res) override;
  BT::NodeStatus on_failed_request(FailureCause failure) override;
};

#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_BT_NODES_ACTION_TRACKING_PATH_H_
