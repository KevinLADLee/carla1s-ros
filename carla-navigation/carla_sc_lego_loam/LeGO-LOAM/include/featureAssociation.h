//
// Created by kevinlad on 2021/3/31.
//

#ifndef SRC_SC_LEGO_LOAM_LEGO_LOAM_SRC_FEATUREASSOCIATION_H_
#define SRC_SC_LEGO_LOAM_LEGO_LOAM_SRC_FEATUREASSOCIATION_H_

#include "common.h"
#include "cloud_msgs/cloud_info.h"

class FeatureAssociation {
 public:
  FeatureAssociation();

  void getParamFromRos();

  void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
  void outlierCloudHandler(const sensor_msgs::PointCloud2ConstPtr& msgIn);
  void laserCloudInfoHandler(const cloud_msgs::cloud_infoConstPtr& msgIn);
  void findCorrespondingCornerFeatures(int iterCount);
  void findCorrespondingSurfFeatures(int iterCount);
  bool calculateTransformationSurf(int iterCount);
  bool calculateTransformationCorner(int iterCount);
  bool calculateTransformation(int iterCount);

  void AccumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz,
                                              float &ox, float &oy, float &oz);
  void adjustOutlierCloud();

  void TransformToStart(PointType const * const pi, PointType * const po);
  void TransformToEnd(PointType const * const pi, PointType * const po);

  void initializationValue();
  void runFeatureAssociation();
  void adjustDistortion();
  void calculateSmoothness();
  void markOccludedPoints();
  void extractFeatures();
  void publishCloud();
  void checkSystemInitialization();
  void updateTransformation();
  void integrateTransformation();
  void publishOdometry();
  void publishCloudsLast();

 private:

  ros::NodeHandle nh;

  // Parameters
  int N_SCAN = 16;
  int Horizon_SCAN = 1800;
  float scan_period = 0.1;
  float edge_threshold = 0.1;
  float surf_threshold = 0.1;
  float nearest_feature_search_sq_dist = 15.0f;

  ros::Subscriber subLaserCloud;
  ros::Subscriber subLaserCloudInfo;
  ros::Subscriber subOutlierCloud;
  ros::Subscriber subImu;

  ros::Publisher pubCornerPointsSharp;
  ros::Publisher pubCornerPointsLessSharp;
  ros::Publisher pubSurfPointsFlat;
  ros::Publisher pubSurfPointsLessFlat;

  pcl::PointCloud<PointType>::Ptr segmentedCloud;
  pcl::PointCloud<PointType>::Ptr outlierCloud;

  pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
  pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
  pcl::PointCloud<PointType>::Ptr surfPointsFlat;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;

  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScanDS;

  pcl::VoxelGrid<PointType> downSizeFilter;

  double timeScanCur;
  double timeNewSegmentedCloud;
  double timeNewSegmentedCloudInfo;
  double timeNewOutlierCloud;

  bool newSegmentedCloud;
  bool newSegmentedCloudInfo;
  bool newOutlierCloud;

  cloud_msgs::cloud_info segInfo;
  std_msgs::Header cloudHeader;

  int systemInitCount;
  bool systemInited;

  std::vector<smoothness_t> cloudSmoothness;
  float *cloudCurvature;
  int *cloudNeighborPicked;
  int *cloudLabel;

  ros::Publisher pubLaserCloudCornerLast;
  ros::Publisher pubLaserCloudSurfLast;
  ros::Publisher pubLaserOdometry;
  ros::Publisher pubOutlierCloudLast;

  int skipFrameNum;
  bool systemInitedLM;

  int laserCloudCornerLastNum;
  int laserCloudSurfLastNum;

  int *pointSelCornerInd;
  float *pointSearchCornerInd1;
  float *pointSearchCornerInd2;

  int *pointSelSurfInd;
  float *pointSearchSurfInd1;
  float *pointSearchSurfInd2;
  float *pointSearchSurfInd3;

  float transformCur[6];
  float transformSum[6];

  pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
  pcl::PointCloud<PointType>::Ptr laserCloudOri;
  pcl::PointCloud<PointType>::Ptr coeffSel;

  pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast;
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast;

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  PointType pointOri, pointSel, tripod1, tripod2, tripod3, pointProj, coeff;

  nav_msgs::Odometry laserOdometry;

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform laserOdometryTrans;

  bool isDegenerate;
  cv::Mat matP;

  int frameCount;



};

#endif //SRC_SC_LEGO_LOAM_LEGO_LOAM_SRC_FEATUREASSOCIATION_H_
