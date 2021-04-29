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
    InitMarkerColors();
    tl_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/carla/"+role_name_+"/traffic_light_markers", 10);
  }

  ROS_INFO("[TrafficLightPerception] Initialization success!");
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

  auto size = info_msg->traffic_lights.size();
  traffic_lights_.reserve(size);

  for(auto &tl_info : info_msg->traffic_lights){
    auto current_id = tl_info.id;

    tf::Transform tl_trans = PoseMsgToTfTransform(tl_info.transform);

    tf::Transform tl_box_trans = BoxinTrafficToMap(tl_info.trigger_volume.center, tl_trans);
    traffic_lights_.emplace_back(current_id, tl_trans, tl_box_trans, tl_info.trigger_volume.size);

    if(publish_viz_){
       CreateMarker(traffic_lights_.back());
    }
  }

  ROS_INFO("Got %zu traffic lights info!", traffic_lights_.size());

}

void TrafficLightPerception::TrafficLightStatusCallback(const carla_msgs::CarlaTrafficLightStatusListConstPtr &status_msg) {
  if(!traffic_lights_info_received_){
    return;
  }

  for(auto &tl_status : status_msg->traffic_lights){
    auto current_id = tl_status.id;
    auto tl_idx = FindElementById<TrafficLight>(traffic_lights_, current_id);
    if(tl_idx < 0){
      std::cerr << "tl not found!" << std::endl;
      continue;
    }

    auto tl_it = traffic_lights_.begin() + tl_idx;
    switch (tl_status.state) {
      case carla_msgs::CarlaTrafficLightStatus::GREEN:
        tl_it->passable = true;
        UpdateMarker(current_id, tl_status.state);
        break;
      case carla_msgs::CarlaTrafficLightStatus::OFF:
        tl_it->passable = true;
        break;
      case carla_msgs::CarlaTrafficLightStatus::RED:
        tl_it->passable = false;
        UpdateMarker(current_id, tl_status.state);
        break;
      case carla_msgs::CarlaTrafficLightStatus::YELLOW:
      default:
        tl_it->passable = false;
        UpdateMarker(current_id, tl_status.state);
        break;
    }
  }

  if(publish_viz_) {
    tl_viz_pub_.publish(tl_viz_marker_vec_msgs_);
  }
}

void TrafficLightPerception::InitMarkerColors() {
  color_map_[TrafficLight::Status::RED] = std_msgs::ColorRGBA();
  color_map_[TrafficLight::Status::RED].r = 1.0;
  color_map_[TrafficLight::Status::RED].a = 1.0;

  color_map_[TrafficLight::Status::GREEN] = std_msgs::ColorRGBA();
  color_map_[TrafficLight::Status::GREEN].g = 1.0;
  color_map_[TrafficLight::Status::GREEN].a = 1.0;

  color_map_[TrafficLight::Status::YELLOW] = std_msgs::ColorRGBA();
  color_map_[TrafficLight::Status::YELLOW].r = 1.0;
  color_map_[TrafficLight::Status::YELLOW].g = 1.0;
  color_map_[TrafficLight::Status::YELLOW].a = 1.0;
}

tf::Transform TrafficLightPerception::PoseMsgToTfTransform(const geometry_msgs::Pose &pose) {
  tf::Transform tl_trans;
  tl_trans.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
  tf::Quaternion tl_q;
  tf::quaternionMsgToTF(pose.orientation, tl_q);
  tl_trans.setRotation(tl_q);
  return tl_trans;
}

void TrafficLightPerception::CreateMarker(const TrafficLight& tl){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.type = visualization_msgs::Marker::CUBE;
  marker.id = tl.id;
  marker.color = color_map_.find(TrafficLight::RED)->second;

  poseTFToMsg(tl.box.box_trans, marker.pose);

  marker.scale.x = tl.box.size.x;
  marker.scale.y = tl.box.size.y;
  marker.scale.z = tl.box.size.z;
  tl_viz_marker_vec_msgs_.markers.push_back(marker);
}

tf::Transform TrafficLightPerception::BoxinTrafficToMap(const geometry_msgs::Vector3 &box_pos_in_traffic,
                                                        const tf::Transform &tl_transform_in_map) {
  tf::Transform tl_box_trans;
  tf::Vector3 tl_box;
  tf::vector3MsgToTF(box_pos_in_traffic, tl_box);
  auto tl_box_pos_in_map = tl_transform_in_map * tl_box;
  tl_box_trans.setOrigin(tl_box_pos_in_map);
  tl_box_trans.setRotation(tl_transform_in_map.getRotation());
  return tl_box_trans;
}

void TrafficLightPerception::UpdateMarker(unsigned int id, unsigned char status) {

  if(!publish_viz_){
    return;
  }

  auto tl_marker_idx = FindElementById<visualization_msgs::Marker>(tl_viz_marker_vec_msgs_.markers, id);
  if(tl_marker_idx > 0){
    tl_viz_marker_vec_msgs_.markers.at(tl_marker_idx).color = color_map_.find(status)->second;
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




