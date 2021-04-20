//
// Created by kevinlad on 2021/4/20.
//

#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_NODE_TYPES_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_NODE_TYPES_H_

#include <ros/ros.h>
#include <std_msgs/Int16.h>


 class MoveToGoal : public BT::SyncActionNode
{
 public:
  MoveToGoal(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
  {}


   MoveToGoal() = delete;



  static BT::PortsList providedPorts()
  {
    return {
    };
  }

  // This Action writes a value into the port "text"
  BT::NodeStatus tick() override
  {
    // the output may change at each tick(). Here we keep it simple.
std::cout <<  "MoveToGoal" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }


};


static BT::NodeStatus StopAndWait()
{
  std::cout << "StopAndWait" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

class CheckTrafficLight : public BT::ConditionNode{
  enum TrafficLightStats{
    RED = 0,
    YELLOW = 1,
    GREEN = 2
  };

 public:
  CheckTrafficLight(const std::string &condition_name,
                    const BT::NodeConfiguration &conf) : BT::ConditionNode(condition_name, conf){
    traffic_sub_ = nh.subscribe<std_msgs::Int16>("/traffic_light_stats", 10,
                                                 boost::bind(&CheckTrafficLight::TrafficCallback, this, _1));
  };

  void TrafficCallback(const std_msgs::Int16::ConstPtr &msg){
    traffic_light_status_ = msg->data;
  }

  static BT::PortsList providedPorts()
  {
    // This action has a single input port called "message"
    // Any port must have a name. The type is optional.
    return { BT::InputPort<std::string>("traffic_light_status") };
  }

  BT::NodeStatus tick() override
  {


    std::cout << "CheckTrafficLight" << std::endl;

//    if (!traffic_light_status_)
//    {
//      throw BT::RuntimeError("missing required input [message]: ",
//                             traffic_light_status.error() );
//    }

    switch (traffic_light_status_) {
      case TrafficLightStats::RED:
      case TrafficLightStats::YELLOW:
        std::cout << "stop" << std::endl;
        return BT::NodeStatus::FAILURE;
        break;
      case TrafficLightStats::GREEN:
        default:
          std::cout << "running" << std::endl;
    }

    return BT::NodeStatus::SUCCESS;
  }
 private:
  ros::NodeHandle nh;
  int traffic_light_status_;
  ros::Subscriber traffic_sub_;

};


#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_NODE_TYPES_H_
