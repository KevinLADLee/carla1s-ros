
#include "parking.h"

Parking::Parking(const std::string &name,
                 const std::string &action_client_name,
                 const BT::NodeConfiguration &conf)
    : RosActionNode(name, action_client_name, conf) {}

void Parking::on_tick() {
  config().blackboard->get<float>("road_width", goal_.road_width);
  config().blackboard->get<float>("parking_spot_length", goal_.parking_spot_length);
  config().blackboard->get<std::vector<geometry_msgs::Point>>("parking_spot_corners", goal_.parking_spot_corners);
}

BT::NodeStatus Parking::on_result(const ResultType &res) {
  return BT::NodeStatus::SUCCESS;
}
