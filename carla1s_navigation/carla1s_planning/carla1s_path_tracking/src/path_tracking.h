
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
#include <std_msgs/Int32.h>

#include <carla1s_msgs/PathTrackingAction.h>
#include <carla_msgs/CarlaEgoVehicleInfo.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>

#include "common/timer.h"
#include "lat_controller/pure_pursuit.h"
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
  using LatControllerT = PurePursuit;
//  using LatControllerT = PidLatController;
  using LonControllerT = PidLonController;

 public:
  PathTracking();

  void ActionExecuteCallback(const ActionGoalT::ConstPtr & goal_msg);

  void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);

  const NodeState &GetNodeState();

  void SetNodeState(const NodeState & node_state);

 private:

  void InitMarkers(const geometry_msgs::PoseStamped &goal_pose);

  void PublishMarkers(const Pose2d &vehicle_pose, const Pose2d &track_point);

  void StartPathTracking();

  void StopPathTracking();

  void PathTrackingLoop();

  bool UpdateParam();

  bool StopVehicle();

  bool IsGoalReached(const Pose2d &vehicle_pose);

  DrivingDirection MsgToDirection(const int8_t &dire_msg);

 private:

  //! Parameters
  bool use_vehicle_info = true;
  std::string role_name = "ego_vehicle";
  int controller_freq = 20;
  float goal_radius = 0.2;
  float vehicle_wheelbase = 2.0;
  float vehicle_track = 2.0;
  float base_angle = 0.0;
  float max_forward_velocity = 15.0; // km/h
  float max_backwards_velocity = 5.0; // km/h
  double max_steer_angle = 1.0;

  //! PID Parameters
  double pid_Kp;
  double pid_Ki;
  double pid_Kd;
  double pid_dt;
  double pid_max_value;
  double pid_min_value;

  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_, vehicle_info_sub_;
  ros::Publisher markers_pub_, control_cmd_pub_;
  ros::Publisher current_path_pub_;

  std::unique_ptr<ActionServerT> as_;

  carla_msgs::CarlaEgoVehicleControl vehicle_control_msg_;
  visualization_msgs::MarkerArray visualize_markers_;

  std::mutex odom_mutex_;
  Pose2d vehicle_pose_;
  DrivingDirection current_driving_direction_;
  nav_msgs::Path current_path_msg_;
  Path2dPtr current_path_ptr_;
  geometry_msgs::PoseStamped current_goal_;
  double vehicle_speed_ = 0;

  NodeState node_state_ = NodeState::IDLE;

  std::mutex path_mutex_, node_state_mutex_;
  std::thread path_tracking_thread_;
  std::unique_ptr<carla1s::Timer> path_tracking_timer_;
  std::unique_ptr<LatControllerT> lat_controller_ptr_;
  std::unique_ptr<LonControllerT> lon_controller_ptr_;

  ros::Publisher lat_error_pub_, lon_error_pub_;
  std_msgs::Int32 lat_error_msg_, lon_error_msgs_;

};

#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PATH_TRACKING_H_
