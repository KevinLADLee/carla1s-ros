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

  // TODO: Add algorithm selector
  path_tracker_ptr_ = std::make_unique<PathTrackerT>();
  path_tracker_ptr_->Initialize(vehicle_wheelbase);

  odom_sub_ = nh_.subscribe("/carla/ego_vehicle/odometry", 1, &PathTracking::OdomCallback, this);
  cmd_vel_pub_ = nh_.advertise<ackermann_msgs::AckermannDrive>("/carla/ego_vehicle/ackermann_cmd", 1);
  markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/carla/path_tracking/markers", 1);

  as_->start();
  SetNodeState(NodeState::IDLE);

}

bool PathTracking::UpdateParam() {
  nh_.param<std::string>("role_name", role_name, "ego_vehicle");
  nh_.param<int>("controller_frequency", controller_freq, 50);
  nh_.param<float>("goal_tolerance_xy", 0.5);
  nh_.param<float>("goal_tolerance_yaw", 0.2);
  nh_.param<bool>("use_vehicle_info", use_vehicle_info,true);
  nh_.param<float>("base_angle", base_angle, 0.0);
  nh_.param<float>("max_speed", max_speed, 8.0);

  if(use_vehicle_info) {
    auto vehicle_info_msg = ros::topic::waitForMessage<carla_msgs::CarlaEgoVehicleInfo>("/carla/"+role_name+"/vehicle_info", nh_, ros::Duration(120));
    auto wheels = vehicle_info_msg->wheels;
    vehicle_wheelbase = std::abs(wheels.at(0).position.x - wheels.at(2).position.x);
    ROS_INFO("VehicleInfo: wheel_base = %f", vehicle_wheelbase);
  }
  return true;
}

void PathTracking::ActionExecuteCallback(const ActionGoalT::ConstPtr &action_goal_msg) {

  int current_path_index = 0;
  bool preempted = false;
  unsigned int path_num = action_goal_msg->path.paths.size();
  for(current_path_index = 0; current_path_index < path_num; current_path_index++) {
    if(preempted){
      preempted = false;
      break;
    }
    ROS_INFO("Path Tracking Goal Received!");
    auto node_state = GetNodeState();

    if (node_state == NodeState::FAILURE) {
      ActionResultT result;
      result.error_code = NodeState::FAILURE;
      as_->setAborted(result, "Failed to track path!");
      return;
    }

    if (path_mutex_.try_lock()) {
      auto driving_direction = DrivingDirection::FORWARD;
      if (action_goal_msg->path.driving_direction.at(current_path_index) == carla_nav_msgs::Path::BACKWARDS) {
        driving_direction = DrivingDirection::BACKWARDS;
      }
      path_tracker_ptr_->SetPlan(RosPathToPath2d(action_goal_msg->path.paths.at(current_path_index)),
                                 driving_direction);
      InitMarkers(action_goal_msg->path.paths.at(current_path_index).poses.back());
      path_mutex_.unlock();
      plan_condition_.notify_one();
    }

    ROS_INFO("Start tracking!");
    if (node_state == NodeState::IDLE) {
      StartPathTracking();
    }

    while (ros::ok()) {
      std::this_thread::sleep_for(std::chrono::microseconds(1));

      if (as_->isPreemptRequested()) {
        ROS_INFO("Action Preempted");
        StopPathTracking();
        SetNodeState(NodeState::IDLE);
        preempted = true;
        ActionResultT result;
        result.error_code = NodeState::SUCCESS;
        as_->setPreempted(result);
        break;
      }
      node_state = GetNodeState();

      if (node_state == NodeState::RUNNING ||
          node_state == NodeState::SUCCESS ||
          node_state == NodeState::FAILURE) {
        ActionFeedbackT feedback;
        ActionResultT result;

        feedback.error_code = node_state;
        as_->publishFeedback(feedback);

        if (node_state == NodeState::SUCCESS && current_path_index == path_num - 1) {
          result.error_code = node_state;
          as_->setSucceeded(result, "Tracking succeed!");
          StopPathTracking();
          break;
        } else if (node_state == NodeState::FAILURE) {
          result.error_code = node_state;
          as_->setAborted(result, "Error!");
          StopPathTracking();
          break;
        }
      }
    }
  }
  ROS_INFO("Path Tracking Action Succeed!");
}

void PathTracking::OdomCallback(const nav_msgs::Odometry_<std::allocator<void>>::ConstPtr &odom_msg) {
  std::lock_guard<std::mutex> lock_guard(odom_mutex_);
  odom_ = *odom_msg;
}

const NodeState &PathTracking::GetNodeState (){
  LockGuardMutex lock_guard(node_state_mutex_);
  return node_state_;
}

void PathTracking::SetNodeState(const NodeState &node_state) {
  LockGuardMutex lock_guard(node_state_mutex_);
  node_state_ = node_state;
}

void PathTracking::InitMarkers(const geometry_msgs::PoseStamped &goal_pose) {
  visualize_markers_.markers.clear();
  visualize_markers_.markers.reserve(2);
  visualization_msgs::Marker goal_marker;
  goal_marker.header.frame_id = "map";
  goal_marker.header.stamp = ros::Time();
  goal_marker.id = 0;
  goal_marker.action = visualization_msgs::Marker::ADD;
  goal_marker.type = visualization_msgs::Marker::SPHERE;
  goal_marker.pose.position = goal_pose.pose.position;
  goal_marker.scale.x = goal_radius;
  goal_marker.scale.y = goal_radius;
  goal_marker.scale.z = goal_radius;
  goal_marker.color.a = 0.8;
  goal_marker.color.r = 0.0;
  goal_marker.color.g = 0.8;
  goal_marker.color.b = 0.0;
  visualize_markers_.markers.push_back(goal_marker);
  visualization_msgs::Marker line_marker;
  line_marker.header.frame_id = "map";
  line_marker.header.stamp = ros::Time();
  line_marker.id = 1;
  line_marker.action = visualization_msgs::Marker::ADD;
  line_marker.type = visualization_msgs::Marker::LINE_STRIP;
  line_marker.color.a = 0.8;
  line_marker.color.r = 0.0;
  line_marker.color.g = 0.0;
  line_marker.color.b = 0.8;
  line_marker.scale.x = 0.2;
  line_marker.points.resize(2);
  visualize_markers_.markers.push_back(line_marker);
}

void PathTracking::PublishMarkers(const Pose2d &vehicle_pose, const Pose2d &track_point) {
  auto line_marker_it = visualize_markers_.markers.begin() + 1;
  line_marker_it->points.at(0).x = vehicle_pose.x;
  line_marker_it->points.at(0).y = vehicle_pose.y;
  line_marker_it->points.at(1).x = track_point.x;
  line_marker_it->points.at(1).y = track_point.y;
  markers_pub_.publish(visualize_markers_);
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
    ackermann_cmd_.acceleration = ackermann_cmd.acceleration;
    ackermann_cmd_.steering_angle = ackermann_cmd.steering_angle;
    cmd_vel_pub_.publish(ackermann_cmd_);

    PublishMarkers(vehicle_pose, path_tracker_ptr_->GetCurrentTrackPoint());

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


