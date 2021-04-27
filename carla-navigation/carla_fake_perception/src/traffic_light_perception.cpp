//
// Created by kevinlad on 2021/4/27.
//

#include "traffic_light_perception.h"

TrafficLightPerception::TrafficLightPerception() {

  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/carla/"+role_name_+"/odometry", 10, boost::bind(&TrafficLightPerception::OdomCallback, this, _1));

  traffic_lights_info_received_ = false;
  traffic_lights_info_sub_ = nh_.subscribe<carla_msgs::CarlaTrafficLightInfoList>("/carla/traffic_lights/info", 1, boost::bind(&TrafficLightPerception::TrafficLightInfoCallback, this, _1));
  traffic_lights_status_sub_ = nh_.subscribe<carla_msgs::CarlaTrafficLightStatusList>("/carla/traffic_lights/status", 10, boost::bind(&TrafficLightPerception::TrafficLightStatusCallback, this, _1));

  tl_passable_pub_ = nh_.advertise<std_msgs::Bool>("/carla/" + role_name_ + "/fake_perception/traffic_light_passable", 1);

}

void TrafficLightPerception::OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
  odom_ = *odom_msg;
}


void TrafficLightPerception::TrafficLightInfoCallback(const carla_msgs::CarlaTrafficLightInfoListConstPtr &info_msg) {

  traffic_lights_info_received_ = true;

  // TODO:The IDs are known not to start from 0.
  //  In order to prevent ID discontinuity,
  //  here we need to get the maximum and minimum value of IDs.
  auto min_it = std::min_element(info_msg->traffic_lights.begin(),
                                 info_msg->traffic_lights.end(),
                                 [](const carla_msgs::CarlaTrafficLightInfo& tl1, const carla_msgs::CarlaTrafficLightInfo& tl2){
                                   return tl1.id < tl2.id;});
  auto max_it = std::max_element(info_msg->traffic_lights.begin(),
                                 info_msg->traffic_lights.end(),
                                 [](const carla_msgs::CarlaTrafficLightInfo& tl1, const carla_msgs::CarlaTrafficLightInfo& tl2){
                                   return tl1.id > tl2.id;});
  tl_id_min_ = min_it->id;
  tl_id_max_ = max_it->id;

  auto size = (tl_id_max_ - tl_id_min_) + 1;
  traffic_lights_.resize(size);
//  traffic_lights_.resize(info_msg->traffic_lights.size());

  for(auto &tl_info : info_msg->traffic_lights){
    traffic_lights_.at(tl_info.id-tl_id_min_) = TrafficLight(tl_info.id, TlInfoToBox(tl_info), false);
  }

}

void TrafficLightPerception::TrafficLightStatusCallback(const carla_msgs::CarlaTrafficLightStatusListConstPtr &status_msg) {
  for(auto &tl_status : status_msg->traffic_lights){
    switch (tl_status.state) {
      case carla_msgs::CarlaTrafficLightStatus::GREEN:
      case carla_msgs::CarlaTrafficLightStatus::OFF:
        traffic_lights_.at(tl_status.id - tl_id_min_).passable = true;
        break;
      case carla_msgs::CarlaTrafficLightStatus::RED:
      case carla_msgs::CarlaTrafficLightStatus::YELLOW:
      default:
        traffic_lights_.at(tl_status.id - tl_id_min_).passable = false;
        break;
    }
  }

  tl_passable_msg_.data = CheckPassable();
  tl_passable_pub_.publish(tl_passable_msg_);

}

Box TrafficLightPerception::TlInfoToBox(const carla_msgs::CarlaTrafficLightInfo &info) {
  Box box;
  // TODO: Transform traffic light pose and volume to box;
  // info.transform
  // info.trigger_volume
  return box;
}

bool TrafficLightPerception::CheckPassable() {
  // TODO: Use odom_ and traffic_lights_ to determine traffic passable
  //  See: https://github.com/carla-simulator/ros-bridge/blob/master/carla_ad_agent/src/carla_ad_agent/agent.py

  return true;
}

