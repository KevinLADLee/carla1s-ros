#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_BLACKBOARD_HANDLER_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_BLACKBOARD_HANDLER_H_

#include "carla1s_decision_common.h"
#include <carla_msgs/CarlaTrafficLightInfoList.h>
#include <carla_msgs/CarlaTrafficLightStatusList.h>

namespace carla1s_decision{
  class BlackboardHandler{
    typedef std::shared_ptr<BlackboardHandler> Ptr;
   public:

    BlackboardHandler(BT::Blackboard::Ptr bb_ptr, ros::NodeHandlePtr nh_ptr, const std::string &role_name = "ego_vehicle");

    bool LoadParam();

    void TrafficLightCallback(const std_msgs::BoolConstPtr& msg);

    void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);

    void GoalCallback(const geometry_msgs::PoseStampedConstPtr &goal);
    virtual ~BlackboardHandler();

   private:
    BT::Blackboard::Ptr bb_ptr_;
    ros::NodeHandlePtr nh_ptr_;
    std::string role_name_;
    ros::Subscriber odom_sub_, traffic_light_sub_, goal_sub_;
    nav_msgs::Odometry odom_;
  };
}

#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_BLACKBOARD_HANDLER_H_
