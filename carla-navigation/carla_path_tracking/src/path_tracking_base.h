//
// Created by kevinlad on 2021/6/7.
//

#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PATH_TRACKING_BASE_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PATH_TRACKING_BASE_H_

#include <queue>
#include "planner_common.h"

class PathTrackingBase{
 public:
  virtual ~PathTrackingBase() {

  }

  /***
   * @brief Running step for lateral controller
   * @param vehicle_pose
   * @param waypoint_pose
   * @param driving_direction
   * @return return vehicle control steering in [-1.0, 1.0]
   */
  virtual double RunStep(const Pose2d &vehicle_pose,
                         const Path2dPtr &waypoints) {
    return 0.0;
  } ;

  /***
   * @brief Running step for longitudinal controller
   * @param vehicle_speed
   * @param target_speed
   * @param driving_direction
   * @return return vehicle control throttle in [0.0, 1.0]
   */
  virtual double RunStep(const double &target_speed,
                         const double &vehicle_speed){
    return 0.0;
  };

  virtual int SetPlan(const Path2d &path, const DrivingDirection &driving_direction){
    return -1;
  };

  virtual int SetDrivingDirection(const DrivingDirection &driving_direction){
    driving_direction_ = driving_direction;
    return -1;
  };

  virtual DrivingDirection GetDrivingDirection(){
    return driving_direction_;
  }

 protected:
  Path2dPtr waypoints_ptr_;
  Path2d::iterator waypoint_it_;
  DrivingDirection driving_direction_;

};

#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PATH_TRACKING_BASE_H_
