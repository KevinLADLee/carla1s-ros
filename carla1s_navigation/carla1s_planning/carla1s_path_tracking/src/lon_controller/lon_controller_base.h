
#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_LON_CONTROLLER_LON_CONTROLLER_BASE_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_LON_CONTROLLER_LON_CONTROLLER_BASE_H_

#include "common/vehicle_controller_base.h"

class LonController : public VehicleController{
 public:

  virtual double RunStep(const double &target_speed,
                         const double &vehicle_speed,
                         const double &dt){
    double speed = 0.0;
    return speed;
  };

  double RunStep(const Pose2dPtr &vehicle_pose_ptr,
                 const Path2dPtr &waypoints_ptr,
                 const double &vehicle_speed,
                 const double &dt) override {
    return 0;
  }

  virtual void SetTargetSpeed(double target_speed){
    target_speed_ = target_speed;
  };

  int QueryTargetWaypointIndex(const Pose2dPtr &vehicle_pose_ptr,
                               const Path2dPtr &waypoints_ptr) override {
    double lookahead_dist = 0;
    if(GetDrivingDirection() == DrivingDirection::FORWARD){
      lookahead_dist = max_fwd_lookahead_dist_;
    }else{
      lookahead_dist = max_bck_lookahead_dist_;
    }

    auto index = QueryTargetWaypointIndexWithLookaheadDist(vehicle_pose_ptr, waypoints_ptr, lookahead_dist);
    return index;
  }

  virtual double ComputeLonErrors(const Pose2dPtr &vehicle_pose_ptr,
                                  const Path2dPtr &waypoints_ptr){

    auto idx = QueryTargetWaypointIndex(vehicle_pose_ptr, waypoints_ptr);
    SetCurrentWaypointIndex(idx);
    auto waypoint = waypoints_ptr->at(idx);

    Eigen::Matrix3d waypoint_trans;
    auto theta = waypoint.yaw;
    waypoint_trans << cos(theta), -sin(theta), waypoint.x,
        sin(theta), cos(theta), waypoint.y,
        0,0,1;

    auto waypoint_trans_inv = waypoint_trans.inverse();
    Eigen::Vector3d vehicle_point_vec(vehicle_pose_ptr->x, vehicle_pose_ptr->y, 1);
    auto vehicle_point_in_waypoint_frame = waypoint_trans_inv * vehicle_point_vec;

    auto ds = -vehicle_point_in_waypoint_frame.x();

    return ds;
  };


 protected:
  double max_throttle_ = 1.0;
  double max_fwd_lookahead_dist_ = 2.5;
  double max_bck_lookahead_dist_ = 1.0;
  double max_fwd_speed_ = 12.0;
  double max_bck_speed_ = 3.0;
  double target_speed_ = 0.0;

  DrivingDirection driving_direction_ = DrivingDirection::FORWARD;

};

#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_LON_CONTROLLER_LON_CONTROLLER_BASE_H_
