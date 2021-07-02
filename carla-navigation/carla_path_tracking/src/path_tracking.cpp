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

  lateral_controller_ptr_ = std::make_unique<LateralControllerT>();
  lateral_controller_ptr_->Initialize(vehicle_wheelbase, goal_radius);
  longitudinal_controller_ptr = std::make_unique<LongitudinalControllerT>();
  longitudinal_controller_ptr->ResetParam(pid_Kp, pid_Ki, pid_Kd);

  odom_sub_ = nh_.subscribe("/carla/"+role_name+"/odometry", 1, &PathTracking::OdomCallback, this);
  markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/carla/"+role_name+"/path_tracking/markers", 1);
  control_cmd_pub_ = nh_.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/"+role_name+"/vehicle_control_cmd", 10);

  as_->start();
  SetNodeState(NodeState::IDLE);
}

bool PathTracking::UpdateParam() {
  nh_.param<std::string>("role_name", role_name, "ego_vehicle");
  nh_.param<int>("controller_frequency", controller_freq, 20);
  nh_.param<float>("goal_tolerance_xy", goal_radius,0.5);
  nh_.param<float>("goal_tolerance_yaw", 0.2);
  nh_.param<bool>("use_vehicle_info", use_vehicle_info,true);
  nh_.param<float>("base_angle", base_angle, 0.0);
  nh_.param<float>("max_forward_velocity", max_forward_velocity, 15.0);
  nh_.param<float>("max_backwards_velocity", max_backwards_velocity, 5.0);

  //1.0, 0.5, 0.0, 0.206, 0.0206, 0.515
  nh_.param<double>("pid_Kp", pid_Kp, 0.2);
  nh_.param<double>("pid_Ki", pid_Ki, 0.02);
  nh_.param<double>("pid_Kd", pid_Kd, 0.5);
  nh_.param<double>("pid_dt", pid_dt, 1.0);
  nh_.param<double>("pid_max", pid_max_value, 1.0);
  nh_.param<double>("pid_min", pid_min_value, 0.0);

  if(use_vehicle_info) {
    auto vehicle_info_msg = ros::topic::waitForMessage<carla_msgs::CarlaEgoVehicleInfo>("/carla/"+role_name+"/vehicle_info", nh_, ros::Duration(120));
    auto wheels = vehicle_info_msg->wheels;
    vehicle_wheelbase = std::abs(wheels.at(0).position.x - wheels.at(2).position.x);
    ROS_INFO("PathTracking: VehicleInfo: wheel_base = %f", vehicle_wheelbase);
  }
  return true;
}

void PathTracking::ActionExecuteCallback(const ActionGoalT::ConstPtr &action_goal_msg) {
  bool preempted = false;
  auto current_path_it = action_goal_msg->path.paths.begin();
  auto current_direction_it = action_goal_msg->path.driving_direction.begin();
  auto path_num = action_goal_msg->path.paths.size();
  while(current_path_it != action_goal_msg->path.paths.end()) {
    if(preempted){
      preempted = false;
      break;
    }
    ROS_INFO("PathTracking: Path Tracking Goal Received!");
    ROS_INFO("PathTracking: Current Path Index: %ld / Total %ld", 1 + current_path_it - action_goal_msg->path.paths.begin(), path_num);
    auto node_state = GetNodeState();

    if (node_state == NodeState::FAILURE) {
      ActionResultT result;
      result.error_code = NodeState::FAILURE;
      as_->setAborted(result, "Failed to track path!");
      return;
    }

    // TODO: Refactor set plan method
    if (path_mutex_.try_lock()) {
      auto driving_direction = DrivingDirection::FORWARD;
      target_speed_ = max_forward_velocity;
      vehicle_control_msg_.reverse = false;
      if (*current_direction_it == carla_nav_msgs::Path::BACKWARDS) {
        driving_direction = DrivingDirection::BACKWARDS;
        vehicle_control_msg_.reverse = true;
        target_speed_ = max_backwards_velocity;
      }
      longitudinal_controller_ptr->SetDrivingDirection(driving_direction);
      lateral_controller_ptr_->SetPlan(RosPathToPath2d(*current_path_it),
                                 driving_direction);
      InitMarkers((*current_path_it).poses.back());
      path_mutex_.unlock();
//      plan_condition_.notify_one();
    }

    ROS_INFO("PathTracking: Start tracking this path!");
    if (node_state == NodeState::IDLE) {
      StartPathTracking();
    }

    while (ros::ok()) {
      std::this_thread::sleep_for(std::chrono::microseconds(1));

      if (as_->isPreemptRequested()) {
        ROS_INFO("PathTracking: Action Preempted");
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

        if (node_state == NodeState::SUCCESS) {
          if(current_path_it != action_goal_msg->path.paths.end()-1){
            SetNodeState(NodeState::IDLE);
            StopPathTracking();
            break;
          }else {
            result.error_code = node_state;
            as_->setSucceeded(result, "Tracking succeed!");
            StopPathTracking();
            break;
          }
        }
        else if (node_state == NodeState::FAILURE) {
          result.error_code = node_state;
          as_->setAborted(result, "Error!");
          StopPathTracking();
          break;
        }
      }
    }
    current_path_it++;
    current_direction_it++;
  }
  ROS_INFO("PathTracking: Action Succeed!");
}

void PathTracking::OdomCallback(const nav_msgs::Odometry_<std::allocator<void>>::ConstPtr &odom_msg) {
  std::lock_guard<std::mutex> lock_guard(odom_mutex_);
  vehicle_pose_.x = (*odom_msg).pose.pose.position.x;
  vehicle_pose_.y = (*odom_msg).pose.pose.position.y;
  vehicle_pose_.yaw = tf2::getYaw((*odom_msg).pose.pose.orientation);
  vehicle_speed_ = std::sqrt((*odom_msg).twist.twist.linear.x * (*odom_msg).twist.twist.linear.x
                                 + (*odom_msg).twist.twist.linear.y * (*odom_msg).twist.twist.linear.y
                                 + (*odom_msg).twist.twist.linear.z * (*odom_msg).twist.twist.linear.z) * 3.6;
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
  ROS_INFO("PathTracking: start tracking path thread");
  path_tracking_thread_ = std::thread([this] { PathTrackingLoop(); });
}

void PathTracking::StopPathTracking() {
  SetNodeState(NodeState::IDLE);
  if(path_tracking_thread_.joinable()){
    path_tracking_thread_.join();
  }
  ROS_INFO("PathTracking: stop tracking path thread");
}

void PathTracking::PathTrackingLoop() {
  auto sleep_time = std::chrono::milliseconds(0);

  while (GetNodeState() == NodeState::RUNNING){
    auto begin_time = std::chrono::steady_clock::now();
    Pose2dPtr vehicle_pose;
    double vehicle_speed;
    {
      std::lock_guard<std::mutex> guard(odom_mutex_);
      // get vehicle_pose in map frame
      vehicle_pose = std::make_shared<Pose2d>(vehicle_pose_);
      vehicle_speed = vehicle_speed_;
    }

    // TODO: Refactor control step
    std::unique_lock<std::mutex> path_lock(path_mutex_);
    vehicle_control_msg_.steer = static_cast<float>(lateral_controller_ptr_->RunStep(vehicle_pose));
    path_lock.unlock();

    vehicle_control_msg_.throttle = static_cast<float>(longitudinal_controller_ptr->RunStep(target_speed_, vehicle_speed));
    vehicle_control_msg_.brake = 0.0;
    control_cmd_pub_.publish(vehicle_control_msg_);
    //std::cout << "target: " << target_speed_ << " current: " << vehicle_speed << std::endl;
    //std::cout << "steering: " << vehicle_control_msg_.steer << std::endl;

    PublishMarkers(*vehicle_pose, lateral_controller_ptr_->GetCurrentTrackPoint());
    if(lateral_controller_ptr_->IsGoalReached()){
      ROS_INFO("PathTracking: Reached goal!");
      SetNodeState(NodeState::SUCCESS);
      StopVehicle();
    }
    auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin_time);
    int need_time = 1000 / controller_freq;
    sleep_time = std::chrono::milliseconds(need_time) - cost_time;
    if (sleep_time <= std::chrono::milliseconds(0)) {
      sleep_time = std::chrono::milliseconds(0);
    }
    std::this_thread::sleep_for(sleep_time);
  }
  ROS_INFO("PathTracking: Stop path tracking loop succeed! ");
  StopVehicle();
}

bool PathTracking::StopVehicle() {
  vehicle_control_msg_.throttle = 0.0;
  vehicle_control_msg_.steer = 0.0;
  vehicle_control_msg_.brake = 1.0;
  vehicle_control_msg_.hand_brake = false;
  vehicle_control_msg_.manual_gear_shift = false;
  control_cmd_pub_.publish(vehicle_control_msg_);
  return true;
}

Path2d PathTracking::RosPathToPath2d(const nav_msgs::Path &ros_path) {
  Path2d path2d;
  path2d.reserve(ros_path.poses.size());
  for(const auto& pose_ros_it : ros_path.poses){
    auto yaw = tf2::getYaw(pose_ros_it.pose.orientation);
    path2d.emplace_back(pose_ros_it.pose.position.x, pose_ros_it.pose.position.y, yaw);
  }
  return path2d;
}


