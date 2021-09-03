#include "path_tracking_base.h"

int VehicleController::SetDrivingDirection(const DrivingDirection &driving_direction) {
  driving_direction_ = driving_direction;
  return -1;
}

DrivingDirection VehicleController::GetDrivingDirection(){
  return driving_direction_;
}

void VehicleController::ComputeLonErrors(const Pose2dPtr &waypoint,
                                         const Pose2dPtr &vehicle_pose,
                                         const double &vehicle_linear_vel) {

  double dx = vehicle_pose->x - waypoint->x;
  double dy = vehicle_pose->y - waypoint->y;

  double cos_waypoint_theta = std::cos(waypoint->yaw);
  double sin_waypoint_theta = std::sin(waypoint->yaw);

  double diff_vector_sin = cos_waypoint_theta * dy - sin_waypoint_theta * dx;
  double diff_vector_cos = cos_waypoint_theta * dx + sin_waypoint_theta * dy;

}

double VehicleController::PointDistanceSquare(const Pose2d &pose_1, const Pose2d &pose_2){
  const double dx = pose_1.x - pose_2.x;
  const double dy = pose_1.y - pose_2.y;
  return (dx * dx + dy * dy);
}

int VehicleController::FindNearestWaypointIndex(const Pose2dPtr &vehicle_pose,
                                     const Path2dPtr &path){
  double min_dist = std::numeric_limits<double>::max();
  int index = path->size() - 1;
  for(int i = 0; i < path->size(); i++){
    auto dist = PointDistanceSquare(*vehicle_pose, path->at(i));
    if(dist < min_dist){
      min_dist = dist;
      index = i;
    }
  }
  return index;
};




