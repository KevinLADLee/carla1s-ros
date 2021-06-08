//
// Created by kevinlad on 2021/5/26.
//

#include "path_tracking.h"

PathTracking::PathTracking() : nh_({ros::NodeHandle()}){
  as_ = std::make_unique<ActionServerT>(nh_,
                                  "path_tracking_action",
                                  boost::bind(&PathTracking::ActionExecuteCallback,this,_1),
                                  false);

  UpdateParam();

  path_tracker_ptr_ = std::make_unique<PathTrackerT>();
  path_tracker_ptr_->Initialize(vehicle_wheelbase);


  odom_sub_ = nh_.subscribe("/carla/ego_vehicle/odometry", 1, &PathTracking::OdomCallback, this);
  cmd_vel_pub_ = nh_.advertise<ackermann_msgs::AckermannDrive>("/carla/ego_vehicle/ackermann_cmd", 10);

  as_->start();
  SetNodeState(NodeState::IDLE);
  goal_reached_ = false;


}

bool PathTracking::UpdateParam() {
  nh_.param<std::string>("role_name", role_name, "ego_vehicle");
  nh_.param<int>("controller_frequency", controller_freq, 100);
  nh_.param<float>("goal_tolerance_xy", 0.5);
  nh_.param<float>("goal_tolerance_yaw", 0.2);
  nh_.param<bool>("use_vehicle_info", use_vehicle_info,true);
  nh_.param<float>("base_angle", base_angle, 0.0);

  if(use_vehicle_info) {
    auto vehicle_info_msg = ros::topic::waitForMessage<carla_msgs::CarlaEgoVehicleInfo>("/carla/"+role_name+"/vehicle_info", nh_, ros::Duration(120));
    auto wheels = vehicle_info_msg->wheels;
    vehicle_wheelbase = std::abs(wheels.at(0).position.x - wheels.at(2).position.x);
    ROS_INFO("VehicleInfo: wheel_base = %f", vehicle_wheelbase);
  }
  return true;
}

void PathTracking::ActionExecuteCallback(const ActionGoalT::ConstPtr &action_goal_msg) {

  auto node_state = GetNodeState();

  if(node_state == NodeState::FAILURE){
    ActionResultT  result;
    result.error_code = NodeState::FAILURE;
    as_->setAborted(result, "Failed to track path!");
    return;
  }

  if(path_mutex_.try_lock()){
    path_tracker_ptr_->SetPlan(RosPathToPath2d(action_goal_msg->path), PathDirection::FWD);
    path_mutex_.unlock();
    plan_condition_.notify_one();
  }

  ROS_INFO("Start tracking!");
  if(node_state == NodeState::IDLE){
    StartPathTracking();
  }

  while (ros::ok()) {
    std::this_thread::sleep_for(std::chrono::microseconds(1));

    if (as_->isPreemptRequested()) {
      ROS_INFO("Action Preempted");
      StopPathTracking();
      SetNodeState(NodeState::IDLE);
      carla_nav_msgs::TrackingPathResult result;
      result.error_code = NodeState::SUCCESS;
      goal_reached_ = false;
      as_->setPreempted(result);
      break;
    }
    node_state = GetNodeState();

    if (node_state == NodeState::RUNNING ||
        node_state == NodeState::SUCCESS ||
        node_state == NodeState::FAILURE)
    {
      carla_nav_msgs::TrackingPathFeedback feedback;
      carla_nav_msgs::TrackingPathResult result;

      feedback.error_code = node_state;
      as_->publishFeedback(feedback);

      if(node_state == NodeState::SUCCESS) {
        result.error_code = node_state;
        as_->setSucceeded(result,"Tracking succeed!");
        StopPathTracking();
        break;
      } else if(node_state == NodeState::FAILURE) {
        result.error_code = node_state;
        as_->setAborted(result, "Error!");
        StopPathTracking();
        break;
      }
    }
  }
  ROS_INFO("Path Tracking Action Succeed!");
}

void PathTracking::OdomCallback(const nav_msgs::Odometry_<std::allocator<void>>::ConstPtr &odom_msg) {
  std::lock_guard<std::mutex> lock_guard(odom_mutex_);
  odom_ = *odom_msg;
}

//void PathTracking::VehicleInfoCallback(const carla_msgs::CarlaEgoVehicleInfoConstPtr &vehicle_info_msg) {
//    auto wheels = vehicle_info_msg->wheels;
//    vehicle_wheelbase = std::abs(wheels.at(0).position.x - wheels.at(2).position.x);
//    std::cout << "VehicleInfo: wheel_base = " << vehicle_wheelbase << std::endl;
//}

//void PathTracking::PathCallback(const nav_msgs::Path_<std::allocator<void>>::ConstPtr &path_msg) {
//  std::lock_guard<std::mutex> lock_guard(path_mutex_);
//  path_in_map_ = *path_msg;
//}

const NodeState &PathTracking::GetNodeState (){
  LockGuardMutex lock_guard(node_state_mutex_);
  return node_state_;
}

void PathTracking::SetNodeState(const NodeState &node_state) {
  LockGuardMutex lock_guard(node_state_mutex_);
  node_state_ = node_state;
}


//bool PathTracking::GoalReached(){
//  std::lock_guard<std::mutex> lock_guard(odom_mutex_);
//
//  return false;
//}

void PathTracking::StartPathTracking() {
  if(path_tracking_thread_.joinable()){
    path_tracking_thread_.join();
  }
  SetNodeState(NodeState::RUNNING);
  path_tracking_thread_ = std::thread([this] { PathTrackingLoop(); });
}

void PathTracking::StopPathTracking() {
  SetNodeState(NodeState::IDLE);
  if(path_tracking_thread_.joinable()){
    path_tracking_thread_.join();
  }
  ROS_INFO("stop tracking path");
}

void PathTracking::PathTrackingLoop() {
  auto sleep_time = std::chrono::milliseconds(0);
  while (GetNodeState() == NodeState::RUNNING){
    std::unique_lock<std::mutex> path_lock(path_mutex_);
    plan_condition_.wait_for(path_lock, sleep_time);
    auto begin_time = std::chrono::steady_clock::now();

    Pose2d vehicle_pose;
    {
      std::lock_guard<std::mutex> guard(odom_mutex_);
      // get vehicle_pose in map frame
      vehicle_pose.x = odom_.pose.pose.position.x;
      vehicle_pose.y = odom_.pose.pose.position.y;
      vehicle_pose.yaw = tf2::getYaw(odom_.pose.pose.orientation);
    }
    AckermannCmd ackermann_cmd;
    path_tracker_ptr_->ComputeAckermannCmd(vehicle_pose, ackermann_cmd);

    auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin_time);
    int need_time = 1000 / controller_freq;
    sleep_time = std::chrono::milliseconds(need_time) - cost_time;
    if (sleep_time <= std::chrono::milliseconds(0)) {
      sleep_time = std::chrono::milliseconds(0);
    }

    ackermann_cmd_.speed = ackermann_cmd.speed;
    ackermann_cmd_.steering_angle = ackermann_cmd.steering_angle;
    cmd_vel_pub_.publish(ackermann_cmd_);

    // TODO: Publish markers for debug
    auto track_point = path_tracker_ptr_->GetCurrentTrackPoint();

    if(path_tracker_ptr_->IsGoalReached()){
      ROS_INFO("Reached goal!");
      SetNodeState(NodeState::SUCCESS);
    }
  }
  ROS_INFO("Stop path tracking loop...");
  ackermann_cmd_.speed = 0;
  ackermann_cmd_.steering_angle = 0;
}

Path2d PathTracking::RosPathToPath2d(const nav_msgs::Path &ros_path) {
  Path2d path2d;
  path2d.poses.reserve(ros_path.poses.size());
  for(const auto& pose_ros_it : ros_path.poses){
    auto yaw = tf2::getYaw(pose_ros_it.pose.orientation);
    path2d.poses.emplace_back(pose_ros_it.pose.position.x, pose_ros_it.pose.position.y, yaw);
  }
  return path2d;
}


