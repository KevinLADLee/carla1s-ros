#include "check_traffic_light.h"

CheckTrafficLight::CheckTrafficLight(const std::string &condition_name,
                                     const BT::NodeConfiguration &conf) : BT::ConditionNode(condition_name, conf){
  conf.blackboard->get<ros::NodeHandlePtr>("node_handler", nh_ptr_);
  conf.blackboard->get<std::string>("role_name", role_name_);
  traffic_light_sub_ = nh_ptr_->subscribe<std_msgs::Bool>("/carla/"+role_name_+"/fake_perception/traffic_light_passable", 1, &CheckTrafficLight::TrafficLightCallback, this);
}

BT::NodeStatus CheckTrafficLight::tick()
{
  if(!traffic_light_passable_){
    return BT::NodeStatus::FAILURE;
  } else{
    return BT::NodeStatus::SUCCESS;
  }
}

void CheckTrafficLight::TrafficLightCallback(const std_msgs::BoolConstPtr& msg) {
  traffic_light_passable_ = msg->data;
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<CheckTrafficLight>("CheckTrafficLight");
}
