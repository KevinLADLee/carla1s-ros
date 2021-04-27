
#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_FAKE_PERCEPTION_SRC_TRAFFIC_LIGHT_PERCEPTION_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_FAKE_PERCEPTION_SRC_TRAFFIC_LIGHT_PERCEPTION_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

#include <carla_msgs/CarlaTrafficLightInfoList.h>
#include <carla_msgs/CarlaTrafficLightStatusList.h>

struct Box{
  geometry_msgs::Pose pose;
  geometry_msgs::Vector3 size;
};

struct TrafficLight{
  TrafficLight() = default;
  TrafficLight(unsigned int id,
               const Box &box,
               bool passable) : id(id), box(box), passable(false){};
  unsigned int id = 0;
  Box box;
  bool passable = false;
};



class TrafficLightPerception {
 public:
  TrafficLightPerception();

  void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);

  void TrafficLightInfoCallback(const carla_msgs::CarlaTrafficLightInfoListConstPtr &info_msg);

  void TrafficLightStatusCallback(const carla_msgs::CarlaTrafficLightStatusListConstPtr &status_msg);

 private:
  Box TlInfoToBox(const carla_msgs::CarlaTrafficLightInfo &info);

  bool CheckPassable();

 private:
  ros::NodeHandle nh_;
  std::string role_name_ = "ego_vehicle";

  ros::Subscriber odom_sub_;
  nav_msgs::Odometry odom_;

  ros::Subscriber traffic_lights_info_sub_, traffic_lights_status_sub_;
  bool traffic_lights_info_received_ = false;
  unsigned int tl_id_min_, tl_id_max_;
  std::vector<TrafficLight> traffic_lights_;

  ros::Publisher tl_passable_pub_;
  std_msgs::Bool tl_passable_msg_;

};

#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_FAKE_PERCEPTION_SRC_TRAFFIC_LIGHT_PERCEPTION_H_
