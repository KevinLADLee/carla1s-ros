//
// Created by kevinlad on 2021/6/7.
//

#include "pure_pursuit.h"

double PurePursuit::RunStep(const Pose2dPtr &vehicle_pose,
                            const Path2dPtr &waypoints_ptr) {
  if(waypoints_ptr_ != waypoints_ptr){
    waypoints_ptr_ = waypoints_ptr;
    current_waypoint_index_ = 0;
  }

  double steering = 0.0;
  Pose2dPtr valid_waypoint_ptr;
  int status = FindValidWaypoint(vehicle_pose, valid_waypoint_ptr);
  if(status < 0 || valid_waypoint_ptr == nullptr){
    return steering;
  } else{
    steering = ComputeSteering(vehicle_pose, valid_waypoint_ptr);
  }
  return steering;
}

int PurePursuit::FindValidWaypoint(const Pose2dPtr &vehicle_pose_ptr, Pose2dPtr &valid_waypoint_ptr){
  found_valid_waypoint_ = false;
  // make sure vehicle_pose is in map frame
  for (auto &wp_index = current_waypoint_index_; wp_index < waypoints_ptr_->size(); wp_index++) {
    auto waypoint_in_map = waypoints_ptr_->at(wp_index);
    if(IsValidWaypoint(waypoint_in_map, *vehicle_pose_ptr)){
      current_waypoint = waypoint_in_map;
      valid_waypoint_ptr = std::make_shared<Pose2d>(waypoint_in_map);
      found_valid_waypoint_ = true;
      break;
    }
  }
  if(current_waypoint_index_ == waypoints_ptr_->size()){
    current_waypoint_index_--;
    valid_waypoint_ptr = nullptr;
    return -1;
  }
  return 0;
}

double PurePursuit::ComputeSteering(const Pose2dPtr &vehicle_pose, const Pose2dPtr &valid_waypoint){
  double steering = 0.0;
  auto forward_point_in_vehicle_frame = ToVehicleFrame(*valid_waypoint, *vehicle_pose);
  if(GetDrivingDirection() == DrivingDirection::FORWARD){
    float eta = std::atan2(forward_point_in_vehicle_frame.y, forward_point_in_vehicle_frame.x);
    steering = std::atan2(wheel_base * std::sin(eta), (L_fw / 2.0f + l_anchor_fw * std::cos(eta)));
  }
  else if(GetDrivingDirection() == DrivingDirection::BACKWARDS){
//    // TODO: Need more test case to verify
    float eta = M_PI + std::atan2(forward_point_in_vehicle_frame.y, forward_point_in_vehicle_frame.x);
    steering = -std::atan2(wheel_base * std::sin(eta), (L_rv / 2.0f + l_anchor_rv * std::cos(eta)));
  } else{
    steering = 0.0;
  }

  steering = -clip(steering, -max_steering_angle_, max_steering_angle_);
  return steering;
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

Pose2d PurePursuit::GetCurrentTrackPoint() {
  return current_waypoint;
}

int PurePursuit::Initialize(float wheelbase, float goal_radius, float look_ahead_dist_fwd, float anchor_dist_fwd) {
  std::cout << "wheel_base: " << wheelbase << "\n" << std::endl;
  wheel_base = wheelbase;
  L_fw = look_ahead_dist_fwd;
  l_anchor_fw = anchor_dist_fwd;
  return 0;
}

