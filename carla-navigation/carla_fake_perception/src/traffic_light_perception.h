
#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_FAKE_PERCEPTION_SRC_TRAFFIC_LIGHT_PERCEPTION_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_FAKE_PERCEPTION_SRC_TRAFFIC_LIGHT_PERCEPTION_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

#include <carla_msgs/CarlaTrafficLightInfoList.h>
#include <carla_msgs/CarlaTrafficLightStatusList.h>

#include <tf/tf.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

static bool TrafficIdComp(const carla_msgs::CarlaTrafficLightInfo& tl1, const carla_msgs::CarlaTrafficLightInfo& tl2){
  return tl1.id < tl2.id;
}

struct Box{
  tf::Transform box_trans;
//  tf::Vector3 pose_in_global;  // center to traffic light
  tf::Vector3 size;  // maybe parallel to coordinates of traffic light
};

struct TrafficLight{
  TrafficLight() = default;
  TrafficLight(unsigned int id,
               geometry_msgs::Pose transform,
               const Box &box,
               bool passable) : id(id), box(box), passable(false){};
  unsigned int id = 0;
  tf::Transform transform;
  Box box;
  bool passable = false;
};

static visualization_msgs::Marker CreateMarker(const TrafficLight& tl){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.type = visualization_msgs::Marker::CUBE;
  marker.id = tl.id;
  marker.color.a = 1.0;

  poseTFToMsg(tl.box.box_trans, marker.pose);

  marker.scale.x = tl.box.size.x();
  marker.scale.y = tl.box.size.y();
  marker.scale.z = tl.box.size.z();

  return marker;
};





class TrafficLightPerception {
 public:
  TrafficLightPerception();

  void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);

  void TrafficLightInfoCallback(const carla_msgs::CarlaTrafficLightInfoListConstPtr &info_msg);

  void TrafficLightStatusCallback(const carla_msgs::CarlaTrafficLightStatusListConstPtr &status_msg);

 private:
  void InitMarkers(unsigned int size);

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

  bool publish_viz_ = true;
  ros::Publisher tl_viz_pub_;
  visualization_msgs::MarkerArray tl_viz_marker_vec_msgs_;

};

#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_FAKE_PERCEPTION_SRC_TRAFFIC_LIGHT_PERCEPTION_H_
