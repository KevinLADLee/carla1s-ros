
#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_MAPS_SRC_CARLA_MAPS_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_MAPS_SRC_CARLA_MAPS_H_

#include <iostream>
#include <ros/ros.h>
#include <carla_msgs/CarlaWorldInfo.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>

#include <pugixml.hpp>

class ParkingSpot{
};

class CarlaMaps {
 public:
  CarlaMaps();

  void CarlaMapCallback(const std_msgs::StringConstPtr &carla_map_msg);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber carla_world_info_sub_;

  std::string map_name_;
  std::string opendrive_xml_;

  pugi::xml_document pugi_doc_;
  pugi::xml_parse_result pugi_result;

};

#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_MAPS_SRC_CARLA_MAPS_H_
