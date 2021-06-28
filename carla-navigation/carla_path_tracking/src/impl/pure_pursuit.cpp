//
// Created by kevinlad on 2021/6/7.
//

#include "pure_pursuit.h"

double PurePursuit::RunStep(const Pose2d &vehicle_pose,
                            const Path2dPtr &waypoints) {

  vehicle_pose_ = vehicle_pose;
  double steering = 0.0;
  int status = CalculateSteering(vehicle_pose, steering);
  steering = -clip(steering, -max_steering_angle_, max_steering_angle_);
  return steering;
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

int PurePursuit::SetPlan(const Path2d &path, const DrivingDirection &driving_direction) {
  path_ = path;
  SetDrivingDirection(driving_direction);
  current_waypoint_it_ = path_.begin();
  current_waypoint_index_ = 0;
  goal_ = path_.back();
  return 0;
}

int PurePursuit::CalculateSteering(const Pose2d &vehicle_pose, double &steering){
  Pose2d valid_waypoint;
  found_valid_waypoint_ = false;
  // make sure vehicle_pose is in map frame
  if (!IsGoalReached()) {
    for (auto wp_it = current_waypoint_it_; wp_it != path_.end(); wp_it++) {
      auto waypoint_in_map = *wp_it;
      if(IsValidWaypoint(waypoint_in_map, vehicle_pose)){
        valid_waypoint = waypoint_in_map;
        current_waypoint_it_ = wp_it;
        found_valid_waypoint_ = true;
        break;
      }
      current_waypoint_index_++;
    }
    if(current_waypoint_it_ == path_.end()){
      std::cerr << "No more valid waypoints, need replan!" << std::endl;
      return -1;
    }
  } else{
    valid_waypoint = goal_;
    found_valid_waypoint_ = false;
  }
  auto forward_point_in_vehicle_frame = ToVehicleFrame(valid_waypoint, vehicle_pose);
  if(GetDrivingDirection() == DrivingDirection::FORWARD){
    float eta = std::atan2(forward_point_in_vehicle_frame.y, forward_point_in_vehicle_frame.x);
    steering = std::atan2(wheel_base * std::sin(eta), (L_fw / 2.0f + l_anchor_fw * std::cos(eta)));
    return 0;
  } else if(GetDrivingDirection() == DrivingDirection::BACKWARDS){
//    // TODO: Need more test case to verify
    float eta = M_PI + std::atan2(forward_point_in_vehicle_frame.y, forward_point_in_vehicle_frame.x);
    steering = -std::atan2(wheel_base * std::sin(eta), (L_rv / 2.0f + l_anchor_rv * std::cos(eta)));

    return 0;
  }

  return 1;
}

bool PurePursuit::IsValidWaypoint(const Pose2d &waypoint_pose, const Pose2d &vehicle_pose) {
  auto waypoint_pose_in_vehicle_frame = ToVehicleFrame(waypoint_pose, vehicle_pose);
  float dist = std::hypot(waypoint_pose.x - vehicle_pose.x,
                          waypoint_pose.y - vehicle_pose.y);
  if(waypoint_pose_in_vehicle_frame.x > 0
      && GetDrivingDirection() == DrivingDirection::FORWARD
      && dist > L_fw){
    return true;
  } else if(waypoint_pose_in_vehicle_frame.x < 0
      && GetDrivingDirection() == DrivingDirection::BACKWARDS
      && dist > L_rv){
    return true;
  }
  else{
    return false;
  }
}

Pose2d PurePursuit::ToVehicleFrame(const Pose2d &point_in_map, const Pose2d &vehicle_pose_in_map) {
  Eigen::Matrix3d vehicle_trans;
  auto theta = vehicle_pose_in_map.yaw;
  using namespace std;
  vehicle_trans << cos(theta), -sin(theta), vehicle_pose_in_map.x,
                       sin(theta), cos(theta), vehicle_pose_in_map.y,
                       0,0,1;
  auto vehicle_trans_inv = vehicle_trans.inverse();
  Eigen::Vector3d point_vec(point_in_map.x, point_in_map.y, 1);
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

int PurePursuit::Initialize(float wheelbase, float look_ahead_dist_fwd, float anchor_dist_fwd) {
  wheel_base = wheelbase;
  L_fw = look_ahead_dist_fwd;
  l_anchor_fw = anchor_dist_fwd;
  return 0;
}

