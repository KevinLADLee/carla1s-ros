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

  path_tracking_timer_ = std::make_unique<carla1s::Timer>(controller_freq);
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
  auto ph = ros::NodeHandle("~");
  ph.param<std::string>("role_name", role_name, "ego_vehicle");
  ph.param<int>("controller_frequency", controller_freq, 20);
  ph.param<float>("goal_tolerance_xy", goal_radius,0.5);
  ph.param<float>("goal_tolerance_yaw", 0.2);
  ph.param<bool>("use_vehicle_info", use_vehicle_info,true);
  ph.param<float>("base_angle", base_angle, 0.0);
  ph.param<float>("max_forward_velocity", max_forward_velocity, 15.0);
  ph.param<float>("max_backwards_velocity", max_backwards_velocity, 5.0);
  // 1.0, 0.5, 0.0, 0.206, 0.0206, 0.515
  ph.param<double>("pid_Kp", pid_Kp, 0.2);
  ph.param<double>("pid_Ki", pid_Ki, 0.02);
  ph.param<double>("pid_Kd", pid_Kd, 0.5);
  ph.param<double>("pid_dt", pid_dt, 1.0);
  ph.param<double>("pid_max", pid_max_value, 1.0);
  ph.param<double>("pid_min", pid_min_value, 0.0);

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
      if (*current_direction_it == carla_nav_msgs::Path::BACKWARDS) {
        current_driving_direction_ = DrivingDirection::BACKWARDS;
      }else{
        current_driving_direction_ = DrivingDirection::FORWARD;
      }
      current_goal_ = current_path_it->poses.back();
      current_path_ptr_ = std::make_shared<Path2d>(RosPathToPath2d(*current_path_it));
      path_mutex_.unlock();
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
  longitudinal_controller_ptr->SetDrivingDirection(current_driving_direction_);
  lateral_controller_ptr_->SetDrivingDirection(current_driving_direction_);
  InitMarkers(current_goal_);

  while (GetNodeState() == NodeState::RUNNING){
    path_tracking_timer_->TimerStart();
    Pose2dPtr vehicle_pose_ptr;
    double vehicle_speed;
    {
      std::lock_guard<std::mutex> guard(odom_mutex_);
      vehicle_pose_ptr = std::make_shared<Pose2d>(vehicle_pose_);
      vehicle_speed = vehicle_speed_;
    }

    // TODO: Refactor control step
    std::unique_lock<std::mutex> path_lock(path_mutex_);
    vehicle_control_msg_.steer = static_cast<float>(lateral_controller_ptr_->RunStep(vehicle_pose_ptr, current_path_ptr_));
    PublishMarkers(*vehicle_pose_ptr, lateral_controller_ptr_->GetCurrentTrackPoint());
    path_lock.unlock();

    if(longitudinal_controller_ptr->GetDrivingDirection() == DrivingDirection::BACKWARDS){
      vehicle_control_msg_.reverse = 1;
      target_speed_ = max_backwards_velocity;
    } else{
      vehicle_control_msg_.reverse = 0;
      target_speed_ = max_forward_velocity;
    }
    vehicle_control_msg_.throttle = static_cast<float>(longitudinal_controller_ptr->RunStep(target_speed_, vehicle_speed));
    vehicle_control_msg_.brake = 0.0;
    control_cmd_pub_.publish(vehicle_control_msg_);

    if(IsGoalReached(*vehicle_pose_ptr)){
      ROS_INFO("PathTracking: Reached goal!");
      SetNodeState(NodeState::SUCCESS);
      StopVehicle();
    }

    std::this_thread::sleep_for(path_tracking_timer_->TimerEnd());
  }
  ROS_INFO("PathTracking: Stop path tracking loop succeed! ");
  StopVehicle();
}

bool PathTracking::IsGoalReached(const Pose2d &vehicle_pose) {
  auto dist = std::hypot(current_goal_.pose.position.x - vehicle_pose.x,
                    current_goal_.pose.position.y - vehicle_pose.y);
  if(dist <= goal_radius){
    return true;
  } else{
    return false;
  }
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




