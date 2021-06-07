//
// Created by kevinlad on 2021/5/26.
//

#include "path_tracking.h"

PathTracking::PathTracking() : nh_({ros::NodeHandle()}){
  std::make_unique<ActionServerT>(nh_,
                                  "path_tracking_action",
                                  boost::bind(&PathTracking::ActionExecuteCallback,this,_1),
                                  false);

  UpdateParam();

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
    path_in_map_ = action_goal_msg->path;
    goal_pose_ = path_in_map_.poses.back();
    path_mutex_.unlock();
    plan_condition_.notify_one();
  }

  ROS_INFO("Start tracking!");
  if(node_state == NodeState::IDLE){
    StartPathTracking();
  }

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


bool PathTracking::GoalReached(){
  std::lock_guard<std::mutex> lock_guard(odom_mutex_);

  return false;
}

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

    geometry_msgs::Pose vehicle_pose;
    geometry_msgs::Twist vehicle_vel;
    {
      std::lock_guard<std::mutex> guard(odom_mutex_);
      // get vehicle_pose in map frame
      vehicle_pose = odom_.pose.pose;
      vehicle_vel = odom_.twist.twist;
    }



  }
}
