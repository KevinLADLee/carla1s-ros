//
// Created by kevinlad on 2021/3/30.
//

#ifndef SRC_IMAGEPROJECTION_H
#define SRC_IMAGEPROJECTION_H

#include "common.h"
#include "cloud_msgs/cloud_info.h"

class ImageProjection {
 public:
  ImageProjection();

  void getParamFromRos();

  void allocateMemory();

  void resetParameters();

  void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);

  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);

  void findStartEndAngle();

  void projectPointCloud();

  void groundRemoval();

  void cloudSegmentation();

  void labelComponents(int row, int col);

  void publishCloud();

 private:

  ros::NodeHandle nh;

  ros::Subscriber subLaserCloud;

  ros::Publisher pubFullCloud;
  ros::Publisher pubFullInfoCloud;

  ros::Publisher pubGroundCloud;
  ros::Publisher pubSegmentedCloud;
  ros::Publisher pubSegmentedCloudPure;
  ros::Publisher pubSegmentedCloudInfo;
  ros::Publisher pubOutlierCloud;

  // Parameters
  std::string pointcloud_topic;
  int N_SCAN = 16;
  int Horizon_SCAN = 1800;
  bool use_cloud_ring = false;
  bool cloud_ring_revert;
  float lidar_angle_resolution_x;
  float lidar_angle_resolution_y;
  float lidar_angle_bottom;
  int lidar_ground_scan_index;
  float lidar_min_range;
  float lidar_mount_angle;
  float segment_alpha_x;
  float segment_alpha_y;
  float segment_theta = 60.0/180.0*M_PI;
  int segment_valid_point_num;
  int segment_valid_line_num;


  pcl::PointCloud<PointType>::Ptr laserCloudIn;
  pcl::PointCloud<PointXYZITR>::Ptr laserCloudInRing;

  pcl::PointCloud<PointType>::Ptr fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix
  pcl::PointCloud<PointType>::Ptr fullInfoCloud; // same as fullCloud, but with intensity - range

  pcl::PointCloud<PointType>::Ptr groundCloud;
  pcl::PointCloud<PointType>::Ptr segmentedCloud;
  pcl::PointCloud<PointType>::Ptr segmentedCloudPure;
  pcl::PointCloud<PointType>::Ptr outlierCloud;

  PointType nanPoint; // fill in fullCloud at each iteration

  cv::Mat rangeMat; // range matrix for range image
  cv::Mat labelMat; // label matrix for segmentaiton marking
  cv::Mat groundMat; // ground matrix for ground cloud marking
  int labelCount;

  float startOrientation;
  float endOrientation;

  cloud_msgs::cloud_info segMsg; // info of segmented cloud
  std_msgs::Header cloudHeader;

  std::vector<std::pair<int8_t, int8_t> > neighborIterator; // neighbor iterator for segmentaiton process

  uint16_t *allPushedIndX; // array for tracking points of a segmented object
  uint16_t *allPushedIndY;

  uint16_t *queueIndX; // array for breadth-first search process of segmentation, for speed
  uint16_t *queueIndY;
};

#endif //SRC_IMAGEPROJECTION_H
