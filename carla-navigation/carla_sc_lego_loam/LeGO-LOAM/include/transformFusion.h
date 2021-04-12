//
// Created by kevinlad on 2021/4/1.
//

#ifndef SRC_SC_LEGO_LOAM_LEGO_LOAM_INCLUDE_TRANSFORMFUSION_H_
#define SRC_SC_LEGO_LOAM_LEGO_LOAM_INCLUDE_TRANSFORMFUSION_H_

#include "common.h"

class TransformFusion{
 public:
  TransformFusion();
  void transformAssociateToMap();
  void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry);
  void odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped);
 private:

  ros::NodeHandle nh;

  ros::Publisher pubLaserOdometry2;
  ros::Subscriber subLaserOdometry;
  ros::Subscriber subOdomAftMapped;


  nav_msgs::Odometry laserOdometry2;
  tf::StampedTransform laserOdometryTrans2;
  tf::TransformBroadcaster tfBroadcaster2;

  tf::StampedTransform map_2_camera_init_Trans;
  tf::TransformBroadcaster tfBroadcasterMap2CameraInit;

  tf::StampedTransform camera_2_base_link_Trans;
  tf::TransformBroadcaster tfBroadcasterCamera2Baselink;

  float transformSum[6];
  float transformIncre[6];
  float transformMapped[6];
  float transformBefMapped[6];
  float transformAftMapped[6];

  std_msgs::Header currentHeader;
};

#endif //SRC_SC_LEGO_LOAM_LEGO_LOAM_INCLUDE_TRANSFORMFUSION_H_
