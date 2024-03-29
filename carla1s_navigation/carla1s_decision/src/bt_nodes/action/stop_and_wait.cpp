#include "stop_and_wait.h"

StopAndWait::StopAndWait(const std::string &name, const BT::NodeConfiguration &config) : AsyncActionNode(name,
                                                                                                         config) {
  config.blackboard->get<ros::NodeHandlePtr>("node_handler", nh_ptr_);
  config.blackboard->get<std::string>("role_name", role_name_);
  cmd_pub_ = nh_ptr_->advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/" + role_name_ + "/vehicle_control_cmd", 10);
}

BT::NodeStatus StopAndWait::tick() {
//  ROS_INFO("BT Node: StopAndWait");
  setStatus(BT::NodeStatus::RUNNING);
  ros::Rate r(20);
  while (status() == BT::NodeStatus::RUNNING) {
    carla_msgs::CarlaEgoVehicleControl control_msg;
    control_msg.steer = 0.0;
    control_msg.throttle = 0.0;
    control_msg.brake = 1.0;
    control_msg.hand_brake = 0;
    control_msg.manual_gear_shift = 0;
    cmd_pub_.publish(control_msg);
    auto odom = ros::topic::waitForMessage<nav_msgs::Odometry>("/carla/"+role_name_+"/odometry");
    if(odom->twist.twist.linear.x == 0.0){
      break;
    }
    r.sleep();
  }
  return BT::NodeStatus::SUCCESS;
}
void StopAndWait::halt() {
  setStatus(BT::NodeStatus::SUCCESS);
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<StopAndWait>("StopAndWait");
}
