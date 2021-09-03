
#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PATH_TRACKING_BASE_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PATH_TRACKING_BASE_H_

#include "planner_common.h"

class VehicleController{
 public:
  virtual int SetDrivingDirection(const DrivingDirection &driving_direction);

  virtual DrivingDirection GetDrivingDirection();

  /**
   * Reference: Apollo 5.5 -> trajectory_analyzer::ToTrajectoryFrame
   * @param waypoint
   * @param vehicle_pose
   */
  virtual void ComputeLonErrors(const Pose2dPtr &waypoint, const Pose2dPtr &vehicle_pose, const double &vehicle_linear_vel);

  virtual void ComputeLatErrors(const Pose2dPtr &waypoint, const Pose2dPtr &vehicle_pose);

 protected:
  static double PointDistanceSquare(const Pose2d &pose_1, const Pose2d &pose_2);

  virtual int FindNearestWaypointIndex(const Pose2dPtr &vehicle_pose,
                                       const Path2dPtr &path);

 protected:
  Path2dPtr waypoints_ptr_;
  DrivingDirection driving_direction_ = DrivingDirection::FORWARD;
};

class LateralController : public VehicleController{
 public:

  /**
   * @brief
   * @param vehicle_pose
   * @param target_waypoint
   * @return
   */
  virtual double RunStep(const Pose2dPtr &vehicle_pose,
                         const Path2dPtr &target_waypoint){
    double steering = 0.0;
    return steering;
  };

 protected:
  double max_steering_angle_ = 0.0;
  Path2dPtr waypoints_ptr_;
  DrivingDirection driving_direction_ = DrivingDirection::FORWARD;
};

class LongitudinalController : public VehicleController{
 public:

  /**
   * @brief
   * @param target_speed
   * @param vehicle_speed
   * @return
   */
  virtual double RunStep(const double &target_speed,
                         const double &vehicle_speed){
    double speed = 0.0;
    return speed;
  };

  virtual double RunStep(const Pose2dPtr &vehicle_pose,
                         const Path2dPtr &target_waypoint){
    double speed = 0.0;
    return speed;
  };

 protected:
  double max_throttle_ = 1.0;
  DrivingDirection driving_direction_ = DrivingDirection::FORWARD;
};

#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PATH_TRACKING_BASE_H_
