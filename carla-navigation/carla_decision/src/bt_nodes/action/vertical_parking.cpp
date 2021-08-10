
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
  goal_.parking_spot.header.frame_id = "map";
  goal_.parking_spot.center_pose = Pose2dToRosMsg(spot_pose.value());
  goal_.parking_spot.width = spot_width.value();
  goal_.parking_spot.length = spot_length.value();
  ros::Rate r(0.01);
  while (ros::ok()){
    r.sleep();
  }
}

BT::NodeStatus VerticalParking::on_result(const ResultType &res) {
  return BT::NodeStatus::SUCCESS;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<VerticalParking>(
            name, "vertical_parking", config);
      };

  factory.registerBuilder<VerticalParking>(
      "VerticalParking", builder);
}
