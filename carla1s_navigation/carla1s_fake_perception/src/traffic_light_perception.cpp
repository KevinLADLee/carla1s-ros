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
  tl_passable_msg_.data = CheckPassable();
  tl_passable_pub_.publish(tl_passable_msg_);
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

    tf::Transform tl_box_trans_in_map = BoxinTrafficToMap(tl_info.trigger_volume.center, tl_trans);
    traffic_lights_.emplace_back(current_id, tl_trans, tl_box_trans_in_map, tl_info.trigger_volume.size);

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

  visualization_msgs::Marker text_marker;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.header.frame_id = "map";
  text_marker.id = tl.id * 10;
  text_marker.color.r = 1;
  text_marker.color.g = 1;
  text_marker.color.b = 1;
  text_marker.color.a = 1;
  poseTFToMsg(tl.transform, text_marker.pose);
  std::stringstream ss;
  ss << "ID " << tl.id;
  text_marker.text = ss.str();
  text_marker.scale.z = tl.box.size.z;
  tl_viz_marker_vec_msgs_.markers.push_back(text_marker);
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


bool TrafficLightPerception::CheckPassable() {

  double safe_distance = 3.5;

  bool passable = true;

  // get current vehicle position
  tf::Transform vehicle_trans_in_map = PoseMsgToTfTransform(odom_.pose.pose);
  auto vehicle_trans_in_map_inv = vehicle_trans_in_map.inverse();

  for(auto &tl : traffic_lights_){
    if (tl.passable) continue;

    auto tl_point_in_map = tl.transform.getOrigin();
    auto tl_point_in_vehicle = vehicle_trans_in_map_inv * tl_point_in_map;

    // Check if the traffic light is in front of the right side of the vehicle

    if(tl_point_in_vehicle.x() > 0 && tl_point_in_vehicle.y() < 0) {

      auto diff_x = tl.box.box_trans.getOrigin().x() - vehicle_trans_in_map.getOrigin().x();
      auto diff_y = tl.box.box_trans.getOrigin().y() - vehicle_trans_in_map.getOrigin().y();
      auto dist = std::hypot(diff_x, diff_y);

      if (dist > safe_distance) {
        passable = true;
      } else {
        std::cout << "Traffic Light Red! " << "ID: " << tl.id << " dist: " << dist << std::endl;
        passable = false;
        return passable;
      }
    } else{
      continue;
    }

  }

  return passable;
}




