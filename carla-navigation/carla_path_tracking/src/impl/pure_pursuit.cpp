//
// Created by kevinlad on 2021/6/7.
//

#include "pure_pursuit.h"

int PurePursuit::ComputeAckermannCmd(const Pose2d &vehicle_pose,
                                     AckermannCmd &ackermann_cmd) {

  vehicle_pose_ = vehicle_pose;
  float steering = 0.0;
  int status = CalculateSteering(vehicle_pose, steering);
  if(found_valid_waypoint_){
    steering_ =  base_angle + steering * steering_gain;
    GetTargetSpeedAndAcc(acc_, speed_);
  }

  auto dist = DistToGoal(vehicle_pose);
  if(IsGoalReached()){
    speed_ = 0.0;
    steering_ = 0.0;
  } else if(dist < safe_dist){
    speed_ = 1.0;
  }

  // ackermann_cmd.acceleration = acc_;
  ackermann_cmd.speed = speed_;
  // ackermann_cmd.steering_angle = 0.0;
  ackermann_cmd.steering_angle = steering_;
//  std::cout << "Ackermann: " << (int)path_direction_ << std::endl;
//  std::cout << "Speed: " << ackermann_cmd.speed << std::endl;
//  std::cout << "Steering: " << (ackermann_cmd.steering_angle) / M_PI * 180.0f << std::endl;
  return status;
}

bool PurePursuit::IsGoalReached() {
  double dist2goal = std::hypot(goal_.x - vehicle_pose_.x,
                                goal_.y - vehicle_pose_.y);
  if (dist2goal < goal_radius)
  {
    return true;
  } else{
    return false;
  }
}

int PurePursuit::SetPlan(const Path2d &path, const DrivingDirection &path_direction) {
  path_ = path;
  path_direction_ = path_direction;
  current_waypoint_it_ = path_.poses.begin();
  current_waypoint_index_ = 0;
  goal_ = path_.poses.back();
  return 0;
}

int PurePursuit::CalculateSteering(const Pose2d &vehicle_pose, float &steering){
  Pose2d valid_waypoint;
  found_valid_waypoint_ = false;
  // make sure vehicle_pose is in map frame
  if (!IsGoalReached()) {
    for (auto wp_it = current_waypoint_it_; wp_it !=path_.poses.end(); wp_it++) {
      auto waypoint_in_map = *wp_it;
      if(IsValidWaypoint(waypoint_in_map, vehicle_pose)){
        valid_waypoint = waypoint_in_map;
        current_waypoint_it_ = wp_it;
        found_valid_waypoint_ = true;
        break;
      }
      current_waypoint_index_++;
    }
    if(current_waypoint_it_ == path_.poses.end()){
      std::cerr << "No more valid waypoints, need replan!" << std::endl;
      return -1;
    }
  } else{
    valid_waypoint = goal_;
    found_valid_waypoint_ = false;
  }
  auto forward_point_in_vehicle_frame = ToVehicleFrame(valid_waypoint, vehicle_pose);
  if(path_direction_ == DrivingDirection::FORWARD){
    float eta = std::atan2(forward_point_in_vehicle_frame.y, forward_point_in_vehicle_frame.x);
    steering = std::atan2(wheel_base * std::sin(eta), (L_fw / 2.0f + l_anchor_fw * std::cos(eta)));
    return 0;
  } else if(path_direction_ == DrivingDirection::BACKWARDS){
    // TODO: Need more test case to verify
    float eta = std::atan2(forward_point_in_vehicle_frame.y, -forward_point_in_vehicle_frame.x);
    steering = std::atan2(wheel_base * std::sin(eta), (L_rv / 2.0f + l_anchor_rv * std::cos(eta)));
    return 0;
  }
  return 1;
}

bool PurePursuit::IsValidWaypoint(const Pose2d &waypoint_pose, const Pose2d &vehicle_pose) {
  auto waypoint_pose_in_vehicle_frame = ToVehicleFrame(waypoint_pose, vehicle_pose);
  float dist = std::hypot(waypoint_pose.x - vehicle_pose.x,
                          waypoint_pose.y - vehicle_pose.y);
  if(waypoint_pose_in_vehicle_frame.x > 0
      && path_direction_ == DrivingDirection::FORWARD
      && dist > L_fw){
    return true;
  } else if(waypoint_pose_in_vehicle_frame.x < 0
      && path_direction_ == DrivingDirection::BACKWARDS
      && dist > L_rv){
    return true;
  }
  else{
    return false;
  }
}

Pose2d PurePursuit::ToVehicleFrame(const Pose2d &point_in_map, const Pose2d &vehicle_pose_in_map) {
  Eigen::Matrix3f vehicle_trans;
  auto theta = vehicle_pose_in_map.yaw;
  using namespace std;
  vehicle_trans << cos(theta), -sin(theta), vehicle_pose_in_map.x,
                       sin(theta), cos(theta), vehicle_pose_in_map.y,
                       0,0,1;
  auto vehicle_trans_inv = vehicle_trans.inverse();
  Eigen::Vector3f point_vec(point_in_map.x, point_in_map.y, 1);
  auto point_vec_in_vehicle_frame = vehicle_trans_inv * point_vec;
  return {point_vec_in_vehicle_frame[0], point_vec_in_vehicle_frame[1], point_in_map.yaw-vehicle_pose_in_map.yaw};
}

float PurePursuit::DistToGoal(const Pose2d &pose) {
  return std::hypot(goal_.x - pose.x,
                    goal_.y - pose.y);
}

Pose2d PurePursuit::GetCurrentTrackPoint() {
  return (*current_waypoint_it_);
}

void PurePursuit::GetTargetSpeedAndAcc(float &acc, float &speed) {
  if(path_direction_ == DrivingDirection::FORWARD){
    speed = max_forward_speed_;
    acc = max_forward_acc_;
  } else if(path_direction_ == DrivingDirection::BACKWARDS){
    speed = max_backwards_speed_;
    acc = max_backwards_acc_;
  } else{
    speed = 0.0;
    acc = 0.0;
  }
}

int PurePursuit::Initialize(float wheelbase, float look_ahead_dist_fwd, float anchor_dist_fwd) {
  wheel_base = wheelbase;
  L_fw = look_ahead_dist_fwd;
  l_anchor_fw = anchor_dist_fwd;
  return 0;
}

