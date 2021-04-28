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
    tf2::Vector3 t(tl_info.transform.position.x, tl_info.transform.position.y, tl_info.transform.position.z);
    tf2::Quaternion q(tl_info.transform.orientation.x,
                      tl_info.transform.orientation.y,
                      tl_info.transform.orientation.z,
                      tl_info.transform.orientation.w);
    tf2::Transform transform(q, t);
    traffic_lights_.at(tl_info.id-tl_id_min_).box = TlInfoToBox(tl_info);
    traffic_lights_.at(tl_info.id-tl_id_min_).transform = transform;
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
  box.center = tf2::Vector3(info.trigger_volume.center.x, info.trigger_volume.center.y, info.trigger_volume.center.z);
  box.size = tf2::Vector3(info.trigger_volume.size.x, info.trigger_volume.size.y, info.trigger_volume.size.z);
  return box;
}

bool TrafficLightPerception::CheckPassable() {
  // TODO: Use odom_ and traffic_lights_ to determine traffic passable
  //  See: https://github.com/carla-simulator/ros-bridge/blob/master/carla_ad_agent/src/carla_ad_agent/agent.py

    double safe_distance = 1.0;  // adjustalbe

    bool should_stop = false;

    // get current vehicle position
    tf2::Vector3 vehicle_position(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);

    for(auto &tl : traffic_lights_){
      if (tl.passable) continue;

      auto tl_position = tl.transform.getOrigin();


      // judge the area the vehicle belongs to split by line connecting traffic light origin and box center
      auto tl_to_box_center = tl.transform * tl.box.center - tl_position;
      auto tl_to_vehicle = vehicle_position - tl_position;
      if (tl_to_box_center.cross(tl_to_vehicle).x() > 0 )  // the traffic light is on the other side
          continue;

      // transform odom to traffic light coordinate
      auto vehicle_position_new = tl.transform.inverse() * vehicle_position;
      auto vehicle_to_box_center = tl.box.center - vehicle_position_new;
      if (abs(vehicle_to_box_center.x()) <= safe_distance + tl.box.size.x() / 2 &&
          abs(vehicle_to_box_center.y()) <= safe_distance + tl.box.size.y() / 2 &&
          abs(vehicle_to_box_center.z()) <= safe_distance + tl.box.size.z() / 2)
          should_stop = true;

  }

  return should_stop;
}

