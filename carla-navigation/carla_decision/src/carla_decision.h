#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_CARLA_DECISION_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_CARLA_DECISION_H_

#include "common.h"

class CarlaDecision{
 public:
  CarlaDecision();

  bool LoadBehaviorTree(const std::string & filename);

  void Tick();

  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);

 private:
  BT::Tree bt_tree_;
  BT::Blackboard::Ptr bt_blackboard_;
  BT::BehaviorTreeFactory bt_factory_;
  ros::NodeHandle nh_;
  std::unique_ptr<BT::PublisherZMQ> publisher_zmq_ptr_;

  ros::Subscriber goal_sub_;

};




#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_CARLA_DECISION_H_
