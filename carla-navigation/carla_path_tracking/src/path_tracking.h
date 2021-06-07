//
// Created by kevinlad on 2021/5/26.
//

#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PATH_TRACKING_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PATH_TRACKING_H_

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDrive.h>

#include <carla_nav_msgs/TrackingPathAction.h>
#include <carla_msgs/CarlaEgoVehicleInfo.h>

#include "planner_common.h"

class PathTracking {
 public:
  using ActionT = carla_nav_msgs::TrackingPathAction;
  using ActionGoalT   = typename ActionT::_action_goal_type::_goal_type;
  using ActionResultT = typename ActionT::_action_result_type::_result_type;
  using ActionServerT = actionlib::SimpleActionServer<ActionT>;
  using LockGuardMutex = std::lock_guard<std::mutex>;

  PathTracking();

  void ActionExecuteCallback(const ActionGoalT::ConstPtr & goal_msg);

  void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);

//  void VehicleInfoCallback(const carla_msgs::CarlaEgoVehicleInfoConstPtr &vehicle_info_msg);

//  void PathCallback(const nav_msgs::Path::ConstPtr &path_msg);

  const NodeState &GetNodeState();

  void SetNodeState(const NodeState & node_state);

 private:

  void StartPathTracking();

  void StopPathTracking();

  void PathTrackingLoop();

  bool UpdateParam();

  bool GoalReached();

 private:

  //! Parameters
  bool use_vehicle_info = true;
  std::string role_name = "ego_vehicle";
  int controller_freq = 100;
  float goal_radius = 0.5;
  float vehicle_wheelbase = 2.0;
  float vehicle_track = 2.0;
  float base_angle = 0.0;

  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_, vehicle_info_sub_;
  ros::Publisher cmd_vel_pub_;
  std::unique_ptr<ActionServerT> as_;

  ackermann_msgs::AckermannDrive ackermann_cmd_;
  nav_msgs::Odometry odom_;
  nav_msgs::Path path_in_map_;
  geometry_msgs::PoseStamped goal_pose_;

  NodeState node_state_ = NodeState::IDLE;
  bool goal_reached_ = false;

  std::mutex path_mutex_, node_state_mutex_, odom_mutex_;
  std::condition_variable plan_condition_;
  std::thread path_tracking_thread_;

  float speed_;
  float steering_;



};

#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PATH_TRACKING_H_
