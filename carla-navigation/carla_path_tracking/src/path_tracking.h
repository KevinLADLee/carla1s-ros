
#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PATH_TRACKING_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PATH_TRACKING_H_

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <tf2/utils.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

#include <carla_nav_msgs/PathTrackingAction.h>
#include <carla_msgs/CarlaEgoVehicleInfo.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>

#include "planner_common.h"
#include "impl/pure_pursuit.h"
#include "impl/pid.h"

class PathTracking {
public:
    using ActionT = carla_nav_msgs::PathTrackingAction;
    using ActionGoalT   = typename ActionT::_action_goal_type::_goal_type;
    using ActionResultT = typename ActionT::_action_result_type::_result_type;
    using ActionFeedbackT   = typename ActionT::_action_feedback_type::_feedback_type;
    using ActionServerT = actionlib::SimpleActionServer<ActionT>;
    using LockGuardMutex = std::lock_guard<std::mutex>;
    using LateralControllerT = PurePursuit;
    using LongitudinalControllerT = PID;

public:
    PathTracking();

    void ActionExecuteCallback(const ActionGoalT::ConstPtr & goal_msg);

    void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);

    const NodeState &GetNodeState();

    void SetNodeState(const NodeState & node_state);


private:

    void InitMarkers(const geometry_msgs::PoseStamped &goal_pose);

    void PublishMarkers(const Pose2d &vehicle_pose, const Pose2d &track_point);

    void StartPathTracking();

    void StopPathTracking();

    void PathTrackingLoop();

    bool UpdateParam();

    bool StopVehicle();

//  bool GoalReached();

    Path2d RosPathToPath2d(const nav_msgs::Path &ros_path);

private:

    //! Parameters
    bool use_vehicle_info = true;
    std::string role_name = "ego_vehicle";
    int controller_freq = 20;
    float goal_radius = 0.5;
    float vehicle_wheelbase = 2.0;
    float vehicle_track = 2.0;
    float base_angle = 0.0;
    float max_forward_velocity = 15.0; // km/h
    float max_backwards_velocity = 5.0; // km/h

    //! PID Parameters
    double pid_Kp;
    double pid_Ki;
    double pid_Kd;
    double pid_dt;
    double pid_max_value;
    double pid_min_value;

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_, vehicle_info_sub_;
    ros::Publisher cmd_vel_pub_, markers_pub_, control_cmd_pub_;
    std::unique_ptr<ActionServerT> as_;

    carla_msgs::CarlaEgoVehicleControl vehicle_control_msg_;
    visualization_msgs::MarkerArray visualize_markers_;

    std::mutex odom_mutex_;
    Pose2d vehicle_pose_;
    double vehicle_speed_ = 0;
    double target_speed_ = max_forward_velocity;

    NodeState node_state_ = NodeState::IDLE;

    std::mutex path_mutex_, node_state_mutex_;
    std::condition_variable plan_condition_;
    std::thread path_tracking_thread_;
    std::unique_ptr<LateralControllerT> lateral_controller_ptr_;
    std::unique_ptr<LongitudinalControllerT> longitudinal_controller_ptr;

};

#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PATH_TRACKING_H_
