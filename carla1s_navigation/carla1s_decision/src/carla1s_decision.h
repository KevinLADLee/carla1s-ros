#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_CARLA_DECISION_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_CARLA_DECISION_H_

#include "carla1s_decision_common.h"
#include "blackboard_handler.h"

class CarlaDecision{
 public:
  CarlaDecision();

  bool LoadBehaviorTree(const std::string & xml_base_path);

  void Tick();

 private:
  std::string DetermineLibPath(const std::string &path);

 private:
  //! Parameters
  std::string bt_tree_filename;
  std::string role_name;
  bool use_groot = false;

  ros::NodeHandlePtr nh_ptr_;

  BT::Tree bt_tree_;
  BT::Blackboard::Ptr bt_blackboard_;
  BT::BehaviorTreeFactory bt_factory_;
  std::unique_ptr<BT::PublisherZMQ> publisher_zmq_ptr_;
  std::unique_ptr<carla1s_decision::BlackboardHandler> bt_blackboard_handler_;

  const std::vector<std::string> plugin_libs_ = {
      "bt_action_stop_and_wait",
      "bt_action_compute_path_to_goal",
      "bt_action_tracking_path",
      "bt_action_cruise",
      "bt_action_vertical_parking",
      "bt_action_detect_traffic_light",
      "bt_action_detect_vehicle",
      "bt_action_detect_speed_limit",
      "bt_condition_goal_updated",
      "bt_condition_goal_reached",
      "bt_condition_check_traffic_light",
      "bt_control_pipeline_sequence",
      "bt_decorator_rate_controller",
      "bt_decorator_blackboard_check_bool"
  };

};




#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_CARLA_DECISION_H_
