
#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_VEHICLE_CONTROLLER_BASE_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_VEHICLE_CONTROLLER_BASE_H_

#include <vector>
#include <memory>

#include <carla_nav_types/common_types.h>
#include <carla_nav_types/conversions.h>
#include <carla_nav_math/math_utils.h>

#include <Eigen/Eigen>

class VehicleController{
public:
    virtual double RunStep(const Pose2dPtr &vehicle_pose_ptr,
                           const Path2dPtr &waypoints_ptr,
                           const double &vehicle_speed,
                           const double &dt) = 0;

    virtual int QueryTargetWaypointIndex(const Pose2dPtr &vehicle_pose_ptr,
                                         const Path2dPtr &waypoints_ptr) = 0;

    virtual int SetDrivingDirection(const DrivingDirection &driving_direction){
        driving_direction_ = driving_direction;
        return 0;
    };

    virtual DrivingDirection GetDrivingDirection(){
        return driving_direction_;
    };

protected:
    static double PointDistanceSquare(const Pose2d &pose_1, const Pose2d &pose_2){
        const double dx = pose_1.x - pose_2.x;
        const double dy = pose_1.y - pose_2.y;
        return (dx * dx + dy * dy);
    };

    virtual int QueryNearestWaypointIndex(const Pose2dPtr &vehicle_pose,
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

    virtual int QueryTargetWaypointIndexWithLookaheadDist(const Pose2dPtr &vehicle_pose_ptr,
                                                          const Path2dPtr &waypoints_ptr,
                                                          const double &lookahead_dist){
        using namespace std;

        auto lookahead_dist_square = lookahead_dist*lookahead_dist;

        auto vehicle_trans_inv = vehicle_pose_ptr->ToTransformMatrix().inverse();

        auto nearest_waypoint_inx = QueryNearestWaypointIndex(vehicle_pose_ptr, waypoints_ptr);
        for(int i = nearest_waypoint_inx; i < waypoints_ptr->size(); i++) {
            auto wp = waypoints_ptr->at(i);
            auto wp_vec = wp.ToPointVec();
            auto wp_in_vehicle_frame = vehicle_trans_inv * wp_vec;
            auto dist = PointDistanceSquare(*vehicle_pose_ptr, wp);
            if (driving_direction_ == DrivingDirection::FORWARD
                && wp_in_vehicle_frame.x() > 0
                && dist > (lookahead_dist_square)){
                return i;
            }
            else if (driving_direction_ == DrivingDirection::BACKWARDS
                     && wp_in_vehicle_frame.x() < 0
                     && dist > (lookahead_dist_square)) {
                return i;
            } else {
                continue;
            }
        }
        return waypoints_ptr->size()-1;
    };

public:
    double GetLatestError() const {
        return latest_error_;
    }

    void SetLatestError(double latest_error) {
        latest_error_ = latest_error;
    }

    Path2dPtr GetWaypoints() const{
        return waypoints_ptr_;
    };

    void SetWaypoints(const Path2dPtr &waypoints_ptr){
        waypoints_ptr_ = waypoints_ptr;
    };

    Pose2dPtr GetCurrentTrackPoint(){
        return std::make_shared<Pose2d>(waypoints_ptr_->at(current_waypoint_index_));
    };

    int GetCurrentWaypointIndex() const {
        return current_waypoint_index_;
    }

    void SetCurrentWaypointIndex(int index){
        current_waypoint_index_ = index;
    }

protected:
    Path2dPtr waypoints_ptr_;
    int current_waypoint_index_;
    DrivingDirection driving_direction_ = DrivingDirection::FORWARD;
    double latest_error_ = 0;
};


#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_VEHICLE_CONTROLLER_BASE_H_
