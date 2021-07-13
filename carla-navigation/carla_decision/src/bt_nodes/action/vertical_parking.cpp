
#include "vertical_parking.h"

VerticalParking::VerticalParking(const std::string &name,
                 const std::string &action_client_name,
                 const BT::NodeConfiguration &conf)
    : RosActionNode(name, action_client_name, conf) {}

void VerticalParking::on_tick() {
  auto spot_pose = getInput<Pose2d>("parking_spot_pose");
  auto spot_width = getInput<double>("parking_spot_width");
  auto spot_length = getInput<double>("parking_spot_length");
  if(!spot_pose || !spot_width || !spot_width){
    throw BT::RuntimeError("VerticalParking: error reading port");
  }
  goal_.parking_spot_pose = Pose2dToRosMsg(spot_pose.value());
  goal_.parking_spot_width = spot_width.value();
  goal_.parking_spot_length = spot_length.value();
}

BT::NodeStatus VerticalParking::on_result(const ResultType &res) {
  return BT::NodeStatus::SUCCESS;
}
