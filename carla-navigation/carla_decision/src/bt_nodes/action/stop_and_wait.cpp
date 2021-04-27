#include "stop_and_wait.h"

StopAndWait::StopAndWait(const std::string &name, const BT::NodeConfiguration &config) : AsyncActionNode(name,
                                                                                                         config) {
  config.blackboard->get<ros::NodeHandlePtr>("node_handler", nh_ptr_);
  cmd_vel_pub_ = nh_ptr_->advertise<ackermann_msgs::AckermannDrive>("/carla/ego_vehicle/ackermann_cmd", 10);
}

BT::NodeStatus StopAndWait::tick() {
  std::cout << "StopAndWait" << std::endl;
  setStatus(BT::NodeStatus::RUNNING);
  while (status() == BT::NodeStatus::RUNNING) {
    ackermann_msg_.speed = 0;
    ackermann_msg_.steering_angle = 0;
    cmd_vel_pub_.publish(ackermann_msg_);
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
