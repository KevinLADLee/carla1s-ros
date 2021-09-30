
#ifndef CARLA1S_ROS_CARLA1S_NAVIGATION_CARLA1S_FAKE_PERCEPTION_SRC_OBJECT_DETECTION_H_
#define CARLA1S_ROS_CARLA1S_NAVIGATION_CARLA1S_FAKE_PERCEPTION_SRC_OBJECT_DETECTION_H_

#include <iostream>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <derived_object_msgs/ObjectArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <carla_msgs/CarlaEgoVehicleInfo.h>

class ObjectDetection {
  using OdomObjectsSyncPolicy = message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, derived_object_msgs::ObjectArray>;
 public:
  ObjectDetection();

  bool LoadParam();

  void OdomObjectsCallback(const nav_msgs::OdometryConstPtr &odom_ptr,
                           const derived_object_msgs::ObjectArrayConstPtr &object_array_ptr);

 private:
  double RosPoseDistanceSquare(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2);

  double TwistToVehicleSpeed(const geometry_msgs::Twist &twist);

  void InitMarker();

  void PublishMarker(const derived_object_msgs::Object &object);

 private:
  std::string role_name;
  double max_distance = 0;
  double max_dist_square = 0;
  int vehicle_id = 0;

  ros::NodeHandle nh_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
  std::shared_ptr<message_filters::Subscriber<derived_object_msgs::ObjectArray>> objects_sub_;
  std::shared_ptr<message_filters::Synchronizer<OdomObjectsSyncPolicy>> msg_sync_;
  ros::Publisher viz_pub_;

  derived_object_msgs::Object current_object_;
  visualization_msgs::MarkerArray marker_array_;


};

#endif //CARLA1S_ROS_CARLA1S_NAVIGATION_CARLA1S_FAKE_PERCEPTION_SRC_OBJECT_DETECTION_H_
