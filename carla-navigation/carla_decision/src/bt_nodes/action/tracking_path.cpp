//
// Created by kevinlad on 2021/4/23.
//

#include "tracking_path.h"
TrackingPath::TrackingPath(const std::string &name,
                           const std::string &action_client_name,
                           const BT::NodeConfiguration &conf) : RosActionNode(name, action_client_name, conf) {}

void TrackingPath::on_tick() {
  getInput<nav_msgs::Path>("path", goal_.path);
}




#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<TrackingPath>(
            name, "tracking_path", config);
      };
  factory.registerBuilder<TrackingPath>(
      "TrackingPath", builder);
}
