
#include "path_handler.h"

void PathHandler::Update(const VehicleState &vehicle_state,
                         const DirectedPath2dPtr &directed_path_ptr) {
  if(directed_path_ptr != nullptr){
    SetDirectedPathPtr(directed_path_ptr);
  }
  SetVehicleState(vehicle_state);
}

int PathHandler::QueryNearestWaypointIndex() const {
  auto vehicle_pose = GetVehiclePose();
  return QueryNearestWaypointIndex(vehicle_pose);
}

int PathHandler::QueryNearestWaypointIndex(const Pose2d &pose) const {
  double min_dist = std::numeric_limits<double>::max();
  auto path_ptr = directed_path_ptr_->path_ptr;
  auto path_size = static_cast<int>(directed_path_ptr_->path_ptr->size());
  int index = path_size - 1;
  for(int i = 0; i < path_size; i++){
    auto dist = PointDistanceSquare(pose, directed_path_ptr_->path_ptr->at(i));
    if(dist < min_dist){
      min_dist = dist;
      index = i;
    }
  }
  return index;
}

NodeState PathHandler::QueryNearestWaypointIndexWithLookaheadDist(const double &lookahead_dist, int &waypoint_idx) const {
  using namespace std;

  auto lookahead_dist_square = lookahead_dist*lookahead_dist;

  auto vehicle_pose = GetVehiclePose();
  auto vehicle_trans_inv = vehicle_pose.ToTransformMatrixInv();

  auto nearest_waypoint_inx = QueryNearestWaypointIndex();
  auto waypoints_ptr = directed_path_ptr_->path_ptr;
  for(int i = nearest_waypoint_inx; i < waypoints_ptr->size(); i++) {
    auto wp = waypoints_ptr->at(i);
    auto wp_vec = wp.ToPointVec();
    auto wp_in_vehicle_frame = vehicle_trans_inv * wp_vec;
    auto dist = PointDistanceSquare(vehicle_pose, wp);

//    ROS_WARN("Debug: dist: %f wp_x: %f", dist, wp_in_vehicle_frame.x());
//    ROS_WARN("Debug: wp_vec: %f, %f", wp_vec[0], wp_vec[1]);
//    if (GetDrivingDirection() == DrivingDirection::FORWARD
//        && wp_in_vehicle_frame.x() > 0
//        && dist > (lookahead_dist_square)){
//      waypoint_idx = i;
//      return NodeState::SUCCESS;
//    }
//    else if (GetDrivingDirection() == DrivingDirection::BACKWARDS
//        && wp_in_vehicle_frame.x() < 0
//        && dist > (lookahead_dist_square)) {
//      waypoint_idx = i;
//      return NodeState::SUCCESS;
//    }
//    else {
//      continue;
//    }

    auto dir = GetDrivingDirection();
    switch (dir) {
      case DrivingDirection::FORWARD:
        if(wp_in_vehicle_frame.x() > 0.0){
          if(dist > lookahead_dist_square || i == waypoints_ptr->size()-1){
            waypoint_idx = i;
            return NodeState::SUCCESS;
          }
        }
        break;
      case DrivingDirection::BACKWARDS:
        if(wp_in_vehicle_frame.x() < 0.0){
          if(dist > lookahead_dist_square || i == waypoints_ptr->size()-1){
            waypoint_idx = i;
            return NodeState::SUCCESS;
          }
        }
        break;
    }
  }
//  ROS_WARN("Debug: vehicle_pos: %f, %f", vehicle_pose.x, vehicle_pose.y);
//  ROS_WARN("Debug: lookahead_dist: %f, %d", lookahead_dist_square, nearest_waypoint_inx);
  return NodeState::FAILURE;
}

double PathHandler::PointDistanceSquare(const Pose2d &pose_1, const Pose2d &pose_2) const {
  const double dx = pose_1.x - pose_2.x;
  const double dy = pose_1.y - pose_2.y;
  return (dx * dx + dy * dy);
}

void PathHandler::SetDirectedPathPtr(const DirectedPath2dPtr &directed_path_ptr) {
  directed_path_ptr_ = directed_path_ptr;
}
void PathHandler::SetVehicleState(const VehicleState &vehicle_state) {
  vehicle_state_ = vehicle_state;
}

const Pose2d &PathHandler::GetVehiclePose() const {
  return vehicle_state_.vehicle_pose;
}

const DrivingDirection &PathHandler::GetDrivingDirection() const {
  return directed_path_ptr_->driving_direction;
}