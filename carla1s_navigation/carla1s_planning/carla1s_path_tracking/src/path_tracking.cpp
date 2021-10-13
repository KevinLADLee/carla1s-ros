
#include "path_tracking.h"

PathTracking::PathTracking() : nh_({ros::NodeHandle()}){

  auto success = UpdateParam();
  if(!success){
    ROS_FATAL("PathTracking: UpdateParam failed!");
    ros::shutdown();
    return;
  }

  path_tracking_timer_ = std::make_unique<carla1s::Timer>(controller_freq);
  lat_controller_ptr_ = std::make_unique<LatControllerT>();
  lon_controller_ptr_ = std::make_unique<LonControllerT>();


  odom_sub_ = nh_.subscribe("/carla/"+role_name+"/odometry", 1, &PathTracking::OdomCallback, this);
  control_cmd_pub_ = nh_.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/"+role_name+"/vehicle_control_cmd", 10);

  markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/carla1s/"+role_name+"/path_tracking/markers", 1);
  current_path_pub_ = nh_.advertise<nav_msgs::Path>("/carla1s/"+role_name+"/path_tracking/current_path", 1);
  debug_data_pub_ = nh_.advertise<std_msgs::Float64>("/carla1s/"+role_name+"/path_tracking/debug_data", 1);

  as_ = std::make_unique<ActionServerT>(nh_,
                                        "/carla1s/"+role_name+"/path_tracking_action",
                                        boost::bind(&PathTracking::ActionExecuteCallback,this,_1),
                                        false);

  StartPathTracking();
  as_->start();

}

const NodeState &PathTracking::GetNodeState (){
  LockGuardMutex lock_guard(node_state_mutex_);
  return node_state_;
}

void PathTracking::SetNodeState(const NodeState &node_state) {
  LockGuardMutex lock_guard(node_state_mutex_);
  node_state_ = node_state;
}

const double &PathTracking::GetTargetSpeed() {
  LockGuardMutex lock_guard(target_speed_mutex_);
  return target_speed_;
}

void PathTracking::SetTargetSpeed(const double &target_speed) {
  LockGuardMutex lock_guard(target_speed_mutex_);
  target_speed_ = target_speed;
}

void PathTracking::InitMarkers(const geometry_msgs::PoseStamped &goal_pose) {
  visualize_markers_.markers.clear();
  visualize_markers_.markers.reserve(2);
  visualization_msgs::Marker goal_marker;
  goal_marker.ns = "path_tracking";
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
  line_marker.ns = "path_tracking";
  line_marker.header.stamp = ros::Time();
  line_marker.id = 1;
  line_marker.action = visualization_msgs::Marker::ADD;
  line_marker.type = visualization_msgs::Marker::LINE_STRIP;
  line_marker.color.a = 0.8;
  line_marker.color.r = 0.0;
  line_marker.color.g = 0.0;
  line_marker.color.b = 0.8;
  line_marker.scale.x = 0.2;
  line_marker.pose.orientation.w = 1;
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

bool PathTracking::UpdateParam() {
  auto ph = ros::NodeHandle("~");
  ph.param<std::string>("role_name", role_name, "ego_vehicle");
  ph.param<int>("controller_frequency", controller_freq, 20);
  ph.param<float>("goal_tolerance_xy", goal_radius,0.5);
  ph.param<float>("goal_tolerance_yaw", 0.2);
  ph.param<bool>("use_vehicle_info", use_vehicle_info,true);

  if(use_vehicle_info) {
    ROS_INFO("PathTracking: Waiting for VehicleInfo message...");
    auto vehicle_info_msg = ros::topic::waitForMessage<carla_msgs::CarlaEgoVehicleInfo>("/carla/"+role_name+"/vehicle_info", nh_, ros::Duration(120));
    if(vehicle_info_msg == nullptr){
      ROS_FATAL("VehicleInfo Not Found: %s", role_name.c_str());
      return false;
    }
    auto wheels = vehicle_info_msg->wheels;
    vehicle_wheelbase = std::abs(wheels.at(0).position.x - wheels.at(2).position.x);
    max_steer_angle = std::abs(wheels.at(0).max_steer_angle);
    ROS_INFO("PathTracking: VehicleInfo: wheel_base = %f max_steer_angle = %f", vehicle_wheelbase, max_steer_angle);
  }
  ROS_INFO("PathTracking: Waiting for first odom message...");
  auto wait_first_odom = ros::topic::waitForMessage<nav_msgs::Odometry>("/carla/"+role_name+"/odometry");
  return true;
}

void PathTracking::OdomCallback(const nav_msgs::Odometry_<std::allocator<void>>::ConstPtr &odom_msg) {
  LockGuardMutex lock_guard(odom_mutex_);
  vehicle_state_.vehicle_pose.x = (*odom_msg).pose.pose.position.x;
  vehicle_state_.vehicle_pose.y = (*odom_msg).pose.pose.position.y;
  vehicle_state_.vehicle_pose.yaw = tf2::getYaw((*odom_msg).pose.pose.orientation);
  vehicle_state_.vehicle_speed = std::sqrt((*odom_msg).twist.twist.linear.x * (*odom_msg).twist.twist.linear.x
                                               + (*odom_msg).twist.twist.linear.y * (*odom_msg).twist.twist.linear.y
                                               + (*odom_msg).twist.twist.linear.z * (*odom_msg).twist.twist.linear.z) * 3.6;
}

void PathTracking::ActionExecuteCallback(const ActionGoalT::ConstPtr &action_goal_msg) {

  ActionResultT result;
  //TODO: Determine if path updated
  if(action_goal_msg->path_updated){
    UpdatePath(action_goal_msg->path_array);
    SetNodeState(NodeState::IDLE);
    result.error_code = NodeState::IDLE;
    as_->setSucceeded(result, "update path success");
  }else{
    auto node_state = GetNodeState();
    if(node_state == FAILURE){
      // TODO: Code for NeedForceUpdatePath
      result.error_code = node_state;
      result.error_info = "Need force update path";
      as_->setAborted(result);
    }else{
      SetTargetSpeed(action_goal_msg->target_speed);
      result.error_code = node_state;
      as_->setSucceeded(result);
    }
  }
}

int PathTracking::UpdatePath(const carla1s_msgs::PathArray &path_array_msg) {
  LockGuardMutex lock_guard(path_mutex_);
  path_array_msgs_ = path_array_msg;
  current_path_idx_ = 0;
  return 0;
}

void PathTracking::StartPathTracking() {
  if(path_tracking_thread_.joinable()){
    path_tracking_thread_.join();
  }
  SetNodeState(NodeState::IDLE);
  ROS_INFO("PathTracking: start tracking path thread");
  path_tracking_thread_ = std::thread([this] { PathTrackingLoop(); });
}

VehicleState PathTracking::GetVehicleState() {
  LockGuardMutex lock_guard(odom_mutex_);
  return vehicle_state_;
}

bool PathTracking::UpdateCurrentPath() {
  LockGuardMutex lock_guard(path_mutex_);
  if(current_path_idx_ < path_array_msgs_.paths.size()) {
    auto path_ptr = std::make_shared<Path2d>(RosPathToPath2d(path_array_msgs_.paths.at(current_path_idx_)));
    auto driving_direction = DrivingDirectionMsgToDirection(path_array_msgs_.driving_direction.at(current_path_idx_));
    current_path_ptr_ = std::make_shared<DirectedPath2d>(path_ptr, driving_direction);
    lon_controller_ptr_->Reset(current_path_ptr_);
    lat_controller_ptr_->Reset(current_path_ptr_);
    current_goal_ = *(path_array_msgs_.paths.at(current_path_idx_).poses.end() - 1);
    current_path_pub_.publish(path_array_msgs_.paths.at(current_path_idx_));
    InitMarkers(current_goal_);
    current_path_idx_++;
    return true;
  }else{
    return false;
  }
}

void PathTracking::PathTrackingLoop() {
  auto sleep_time = std::chrono::milliseconds(0);
  while (ros::ok()){
    path_tracking_timer_->TimerStart();
    auto node_state = GetNodeState();
    switch (node_state) {
      case NodeState::FAILURE:
        StopVehicle();
        break;
      case NodeState::IDLE:
        if(UpdateCurrentPath()){
          SetNodeState(NodeState::RUNNING);
        }else{
          StopVehicle();
        }
        break;
      case NodeState::RUNNING:
        auto vehicle_state = GetVehicleState();
        if(IsGoalReached(vehicle_state.vehicle_pose)){
          StopVehicle();
          SetNodeState(NodeState::IDLE);
        }else{
          double throttle = 0.0;
          double steer = 0.0;
          auto target_speed = 15.0;
          auto lon_status = lon_controller_ptr_->RunStep(vehicle_state,
                                                         target_speed,
                                                         1.0 / controller_freq,
                                                         throttle);
          auto lat_status = lat_controller_ptr_->RunStep(vehicle_state,
                                                         target_speed,
                                                         1.0 / controller_freq,
                                                         steer);
          if(lon_status == NodeState::FAILURE || lat_status == NodeState::FAILURE){
            StopVehicle();
            SetNodeState(NodeState::FAILURE);
          }else{
            PublishMarkers(GetVehicleState().vehicle_pose, lat_controller_ptr_->GetCurrentWaypoint());
            PublishVehicleCmd(throttle, steer);
          }
        }
        break;
    }
    std::this_thread::sleep_for(path_tracking_timer_->TimerEnd());
  }
  StopVehicle();
}

bool PathTracking::IsGoalReached(const Pose2d &vehicle_pose) const {
  auto dist = std::hypot(current_goal_.pose.position.x - vehicle_pose.x,
                         current_goal_.pose.position.y - vehicle_pose.y);
  if(dist <= goal_radius){
    return true;
  } else{
    return false;
  }
}

void PathTracking::PublishVehicleCmd(const double &throttle, const double &steer) {
  vehicle_cmd_msg_.reverse = current_path_ptr_->reverse();
  vehicle_cmd_msg_.throttle = static_cast<float>(throttle);
  vehicle_cmd_msg_.steer = static_cast<float>(steer);
  vehicle_cmd_msg_.brake = 0.0;
  vehicle_cmd_msg_.hand_brake = false;
  vehicle_cmd_msg_.manual_gear_shift = false;
  control_cmd_pub_.publish(vehicle_cmd_msg_);
}

bool PathTracking::StopVehicle() {
  vehicle_cmd_msg_.throttle = 0.0;
  vehicle_cmd_msg_.steer = 0.0;
  vehicle_cmd_msg_.brake = 1.0;
  vehicle_cmd_msg_.hand_brake = false;
  vehicle_cmd_msg_.manual_gear_shift = false;
  control_cmd_pub_.publish(vehicle_cmd_msg_);
  return true;
}



