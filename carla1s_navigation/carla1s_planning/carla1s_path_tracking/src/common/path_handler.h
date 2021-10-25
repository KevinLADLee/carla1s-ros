#ifndef CARLA1S_ROS_CARLA1S_NAVIGATION_CARLA1S_PLANNING_CARLA1S_PATH_TRACKING_SRC_COMMON_PATH_HANDLER_H_
#define CARLA1S_ROS_CARLA1S_NAVIGATION_CARLA1S_PLANNING_CARLA1S_PATH_TRACKING_SRC_COMMON_PATH_HANDLER_H_

#include <carla_nav_types/common_types.h>
#include <carla_nav_types/conversions.h>
#include <carla_nav_math/math_utils.h>

class PathHandler {
 public:
  void Update(const VehicleState &vehicle_state,
              const DirectedPath2dPtr &directed_path_ptr = nullptr);

  int QueryNearestWaypointIndex() const;

  int QueryNearestWaypointIndex(const Pose2d &pose) const;

  NodeState QueryNearestWaypointIndexWithLookaheadDist(const double &lookahead_dist, int &waypoint_idx) const;

  double PointDistanceSquare(const Pose2d &pose_1, const Pose2d &pose_2) const;

  const Pose2d &GetVehiclePose() const;

  const DrivingDirection &GetDrivingDirection() const;

  void SetDirectedPathPtr(const DirectedPath2dPtr &directed_path_ptr);

  void SetVehicleState(const VehicleState &vehicle_state);

 private:
  DirectedPath2dPtr directed_path_ptr_;
  VehicleState vehicle_state_;
};

#endif //CARLA1S_ROS_CARLA1S_NAVIGATION_CARLA1S_PLANNING_CARLA1S_PATH_TRACKING_SRC_COMMON_PATH_HANDLER_H_
