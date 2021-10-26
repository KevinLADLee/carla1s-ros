
#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PATH_TRACKING_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PATH_TRACKING_H_

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>

#include <carla1s_msgs/PathTrackingAction.h>
#include <carla_msgs/CarlaEgoVehicleInfo.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <queue>

#include "common/timer.h"
#include "lat_controller/pure_pursuit.h"
#include "lat_controller/stanley.h"
#include "lat_controller/pid_lat_controller.h"
#include "lon_controller/pid_lon_controller.h"

class PathTracking {
 public:
  using ActionT = carla1s_msgs::PathTrackingAction;
  using ActionGoalT   = typename ActionT::_action_goal_type::_goal_type;
  using ActionResultT = typename ActionT::_action_result_type::_result_type;
  using ActionFeedbackT   = typename ActionT::_action_feedback_type::_feedback_type;
  using ActionServerT = actionlib::SimpleActionServer<ActionT>;
  using LockGuardMutex = std::lock_guard<std::mutex>;
  // using LatControllerT = PurePursuit;
 using LatControllerT = Stanley;
//  using LatControllerT = PidLatController;
  using LonControllerT = PidLonController;

 public:
  PathTracking();

  void ActionExecuteCallback(const ActionGoalT::ConstPtr & goal_msg);

  void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);

  const NodeState &GetNodeState();

  void SetNodeState(const NodeState & node_state);

  const double &GetTargetSpeed();

  void SetTargetSpeed(const double &target_speed);

 private:
  bool UpdateParam();

  void InitMarkers(const geometry_msgs::PoseStamped &goal_pose);

  void PublishMarkers(const Pose2d &vehicle_pose, const Pose2d &track_point);

  void StartPathTracking();

  void PathTrackingLoop();

  int UpdatePath(const carla1s_msgs::PathArray &path_array_msg);

  bool UpdateCurrentPath();

  VehicleState GetVehicleState();

  bool StopVehicle();

  void PublishVehicleCmd(const double &throttle, const double &steer);

  bool IsGoalReached() ;

 private:

  //! Parameters
  bool use_vehicle_info = true;
  std::string role_name = "ego_vehicle";
  int controller_freq = 20;
  float goal_radius = 0.5;
  float vehicle_wheelbase = 2.0;
  double max_steer_angle = 1.0;

  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_, vehicle_info_sub_;
  ros::Publisher markers_pub_, control_cmd_pub_;
  ros::Publisher current_path_pub_;
  ros::Publisher debug_data_pub_;
  std_msgs::Float64 debug_data_msgs_;

  std::unique_ptr<ActionServerT> as_;

  carla_msgs::CarlaEgoVehicleControl vehicle_cmd_msg_;
  visualization_msgs::MarkerArray visualize_markers_;

  std::mutex odom_mutex_;
  VehicleState vehicle_state_;

  NodeState node_state_ = NodeState::IDLE;
  std::mutex path_mutex_, node_state_mutex_;
  std::thread path_tracking_thread_;
  std::unique_ptr<carla1s::Timer> path_tracking_timer_;
  std::unique_ptr<LatControllerT> lat_controller_ptr_;
  std::unique_ptr<LonControllerT> lon_controller_ptr_;

  carla1s_msgs::PathArray path_array_msgs_;
  int current_path_idx_ = 0;
  geometry_msgs::PoseStamped current_goal_;
  std::shared_ptr<DirectedPath2d> current_path_ptr_;

  std::mutex target_speed_mutex_;
  double target_speed_ = 15.0; // km/h
};

#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PATH_TRACKING_H_
