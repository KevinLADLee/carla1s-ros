
#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_BT_NODES_ACTION_TRACKING_PATH_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_BT_NODES_ACTION_TRACKING_PATH_H_

#include "carla_decision_common.h"

#include <carla_nav_msgs/PathTrackingAction.h>

class TrackingPath : public BT::RosActionNode<carla_nav_msgs::PathTrackingAction> {
 public:
  TrackingPath(const std::string &name, const std::string &action_client_name, const BT::NodeConfiguration &conf);
  TrackingPath() = delete;

  void on_tick() override;

  static BT::PortsList providedPorts(){
    return {
        BT::InputPort<carla_nav_msgs::Path>("path")
    };
  };

  BT::NodeStatus on_result(const ResultType &res) override;
};

#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_BT_NODES_ACTION_TRACKING_PATH_H_
