//
// Created by kevinlad on 2021/6/7.
//

#include "pure_pursuit.h"

int PurePursuit::ComputeAckermannCmd(const Pose2d &vehicle_pose,
                                     AckermannCmd &ackermann_cmd) {

  vehicle_pose_ = vehicle_pose;
  auto steering = CalculateSteering(vehicle_pose);
  if(found_forward_point_){
    steering_ =  base_angle + steering * steering_gain;
    speed_ = std::min(speed_ + speed_increment, max_speed);
  }

  auto dist = DistToGoal(vehicle_pose);
  if(IsGoalReached()){
    speed_ = 0.0;
    steering_ = 0.0;
  } else if(dist < safe_dist){
    speed_ = 1.0;
  }

  ackermann_cmd.speed = speed_;
  ackermann_cmd.steering_angle = steering_;
  return 0;
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

int PurePursuit::SetPlan(const Path2d &path, const PathDirection &path_direction) {
  path_ = path;
  path_direction_ = path_direction;
  current_waypoint_it_ = path_.poses.begin();
  goal_ = path_.poses.back();
  return 0;
}

float PurePursuit::CalculateSteering(const Pose2d &vehicle_pose) {
  if(path_direction_ == PathDirection::FWD) {
    Pose2d forward_point;
    found_forward_point_ = false;
    // make sure vehicle_pose is in map frame
    if (!IsGoalReached()) {
      for (auto wp_it = current_waypoint_it_; wp_it !=path_.poses.end(); wp_it++) {
        auto waypoint_in_map = *wp_it;
        if(IsForwardWaypoint(waypoint_in_map, vehicle_pose)){
          if(IsWaypointAwayFromLookAheadDist(waypoint_in_map, vehicle_pose)){
            forward_point = waypoint_in_map;
            current_waypoint_it_ = wp_it;
            found_forward_point_ = true;
            break;
          }
        }
      }
    } else{
      forward_point = goal_;
      found_forward_point_ = false;
    }

    auto forward_point_in_vehicle_frame = ToVehicleFrame(forward_point, vehicle_pose);
    auto eta = std::atan2(forward_point_in_vehicle_frame.y, forward_point_in_vehicle_frame.x);
    float steering = std::atan2(wheel_base * std::sin(eta), (L_fw / 2.0f + l_anchor_fw * std::cos(eta)));
    return steering;
  } else if(path_direction_ == PathDirection::BCK){
    // TODO: Backward tracking
    return 0;
  } else{
    return 0;
  }
}

bool PurePursuit::IsForwardWaypoint(const Pose2d &waypoint_pose, const Pose2d &vehicle_pose) {
  auto waypoint_pose_in_vehicle_frame = ToVehicleFrame(waypoint_pose, vehicle_pose);
  if(waypoint_pose_in_vehicle_frame.x > 0){
    return true;
  } else{
    return false;
  }
}

bool PurePursuit::IsWaypointAwayFromLookAheadDist(const Pose2d &waypoint_pose, const Pose2d &vehicle_pose) {
  float dist = std::hypot(waypoint_pose.x - vehicle_pose.x, waypoint_pose.y - vehicle_pose.y);
  if (dist >= L_fw) {
    return true;
  } else{
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

int PurePursuit::Initialize(float wheelbase, float look_ahead_dist_fwd, float anchor_dist_fwd) {
  wheel_base = wheelbase;
  L_fw = look_ahead_dist_fwd;
  l_anchor_fw = anchor_dist_fwd;
  return 0;
}
