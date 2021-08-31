
#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PATH_TRACKING_BASE_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PATH_TRACKING_BASE_H_

#include "planner_common.h"

class VehicleController{
 public:
  virtual int SetDrivingDirection(const DrivingDirection &driving_direction){
    driving_direction_ = driving_direction;
    return -1;
  };

  virtual DrivingDirection GetDrivingDirection(){
    return driving_direction_;
  }

 protected:
  static double PointDistanceSquare(const Pose2d &pose_1, const Pose2d &pose_2){
    const double dx = pose_1.x - pose_2.x;
    const double dy = pose_1.y - pose_2.y;
    return (dx * dx + dy * dy);
  }

  virtual int FindNearestWaypointIndex(const Pose2dPtr &vehicle_pose,
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

  /**
   *
   * @param driving_direction
   * @return
   */
  virtual int SetDrivingDirection(const DrivingDirection &driving_direction){
    driving_direction_ = driving_direction;
    return -1;
  };

  virtual DrivingDirection GetDrivingDirection(){
    return driving_direction_;
  }

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
