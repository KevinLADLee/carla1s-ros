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

  if(publish_viz_){
    tl_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/carla/"+role_name_+"/traffic_light_markers", 10);
  }

}

void TrafficLightPerception::OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
  odom_ = *odom_msg;
//  tl_passable_msg_.data = CheckPassable();
//  tl_passable_pub_.publish(tl_passable_msg_);
}


void TrafficLightPerception::TrafficLightInfoCallback(const carla_msgs::CarlaTrafficLightInfoListConstPtr &info_msg) {

  if(traffic_lights_info_received_){
    return;
  }
  traffic_lights_info_received_ = true;


  // TODO:The IDs are known not to start from 0.
  //  In order to prevent ID discontinuity,
  //  here we need to get the maximum and minimum value of IDs.
  auto min_it = std::min_element(info_msg->traffic_lights.begin(),
                                 info_msg->traffic_lights.end(),
                                 TrafficIdComp);
  auto max_it = std::max_element(info_msg->traffic_lights.begin(),
                                 info_msg->traffic_lights.end(),
                                 TrafficIdComp);
  tl_id_min_ = min_it->id;
  tl_id_max_ = max_it->id;

  auto size = (tl_id_max_ - tl_id_min_) + 1;
  traffic_lights_.resize(size);
  if(publish_viz_){
    InitMarkers(size);
  }
//  traffic_lights_.resize(info_msg->traffic_lights.size());

  for(auto &tl_info : info_msg->traffic_lights){
    auto current_id = tl_info.id-tl_id_min_;
    traffic_lights_.at(current_id).id = tl_info.id;

    tf::Transform tl_trans;
    tl_trans.setOrigin(tf::Vector3(tl_info.transform.position.x, tl_info.transform.position.y, tl_info.transform.position.z));
    tf::Quaternion tl_q;
    tf::quaternionMsgToTF(tl_info.transform.orientation, tl_q);
    tl_trans.setRotation(tl_q);
    traffic_lights_.at(current_id).transform = tl_trans;

    tf::Vector3 tl_trigger_volume(tl_info.trigger_volume.center.x,
                                  tl_info.trigger_volume.center.y,
                                  tl_info.trigger_volume.center.z);

    auto tl_trigger_volume_pos = tl_trans * tl_trigger_volume;
    traffic_lights_.at(current_id).box.box_trans.setOrigin(tl_trigger_volume_pos);
    traffic_lights_.at(current_id).box.box_trans.setRotation(tl_q);
    traffic_lights_.at(current_id).box.size = tf::Vector3(tl_info.trigger_volume.size.x,
                                                          tl_info.trigger_volume.size.y,
                                                          tl_info.trigger_volume.size.z);

    if(publish_viz_){
      tl_viz_marker_vec_msgs_.markers.at(current_id) = (CreateMarker(traffic_lights_.at(current_id)));
    }
  }

}

void TrafficLightPerception::TrafficLightStatusCallback(const carla_msgs::CarlaTrafficLightStatusListConstPtr &status_msg) {
  if(!traffic_lights_info_received_){
    return;
  }

  for(auto &tl_status : status_msg->traffic_lights){
    auto current_id = tl_status.id - tl_id_min_;
    switch (tl_status.state) {
      case carla_msgs::CarlaTrafficLightStatus::GREEN:
        if(publish_viz_){
          tl_viz_marker_vec_msgs_.markers.at(current_id).color.r = 0.0;
          tl_viz_marker_vec_msgs_.markers.at(current_id).color.g = 1.0;
          tl_viz_marker_vec_msgs_.markers.at(current_id).color.b = 0.0;
        }
        traffic_lights_.at(current_id).passable = true;
        break;
      case carla_msgs::CarlaTrafficLightStatus::OFF:
        if(publish_viz_){
          tl_viz_marker_vec_msgs_.markers.at(current_id).color.r = 0.0;
          tl_viz_marker_vec_msgs_.markers.at(current_id).color.g = 0.0;
          tl_viz_marker_vec_msgs_.markers.at(current_id).color.b = 0.0;
        }
        traffic_lights_.at(current_id).passable = true;
        break;
      case carla_msgs::CarlaTrafficLightStatus::RED:
        if(publish_viz_){
          tl_viz_marker_vec_msgs_.markers.at(current_id).color.r = 1.0;
          tl_viz_marker_vec_msgs_.markers.at(current_id).color.g = 0.0;
          tl_viz_marker_vec_msgs_.markers.at(current_id).color.b = 0.0;
        }
        traffic_lights_.at(current_id).passable = false;
        break;
      case carla_msgs::CarlaTrafficLightStatus::YELLOW:
      default:
        if(publish_viz_){
          tl_viz_marker_vec_msgs_.markers.at(current_id).color.r = 1.0;
          tl_viz_marker_vec_msgs_.markers.at(current_id).color.g = 1.0;
          tl_viz_marker_vec_msgs_.markers.at(current_id).color.b = 0.0;
        }
        traffic_lights_.at(current_id).passable = false;
        break;
    }
  }

  if(publish_viz_) {
    tl_viz_pub_.publish(tl_viz_marker_vec_msgs_);
  }
}

void TrafficLightPerception::InitMarkers(unsigned int size) {
  tl_viz_marker_vec_msgs_.markers.resize(size);
  int i = 0;
  for(auto &marker : tl_viz_marker_vec_msgs_.markers){
    marker.header.frame_id = "map";
    marker.id = i;
    marker.color.a = 0.1;
    marker.pose.orientation.w = 1;
    marker.scale.x = 1;
    i++;
  }
}

//bool TrafficLightPerception::CheckPassable() {
//  // TODO: Use odom_ and traffic_lights_ to determine traffic passable
//  //  See: https://github.com/carla-simulator/ros-bridge/blob/master/carla_ad_agent/src/carla_ad_agent/agent.py
//
//    double safe_distance = 1.0;  // adjustalbe
//
//    bool should_stop = false;
//
//    // get current vehicle position
//    tf2::Vector3 vehicle_position(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);
//
//    for(auto &tl : traffic_lights_){
//      if (tl.passable) continue;
//
//      auto tl_position = tl.transform.getOrigin();
//
//      // judge the area the vehicle belongs to split by line connecting traffic light origin and box center
//      auto tl_to_box_center = tl.transform * tl.box.center - tl_position;
//      auto tl_to_vehicle = vehicle_position - tl_position;
//      if (tl_to_box_center.cross(tl_to_vehicle).x() > 0 )  // the traffic light is on the other side
//          continue;
//
//      // transform odom to traffic light coordinate
//      auto vehicle_position_new = tl.transform.inverse() * vehicle_position;
//      auto vehicle_to_box_center = tl.box.center - vehicle_position_new;
//      if (abs(vehicle_to_box_center.x()) <= safe_distance + tl.box.size.x() / 2 &&
//          abs(vehicle_to_box_center.y()) <= safe_distance + tl.box.size.y() / 2 &&
//          abs(vehicle_to_box_center.z()) <= safe_distance + tl.box.size.z() / 2)
//          should_stop = true;
//
//  }
//
//  return should_stop;
//}



