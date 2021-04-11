//
// Created by kevinlad on 2021/3/31.
//

#ifndef SRC_SC_LEGO_LOAM_LEGO_LOAM_INCLUDE_MAPOPTIMIZATION_H_
#define SRC_SC_LEGO_LOAM_LEGO_LOAM_INCLUDE_MAPOPTIMIZATION_H_

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/nonlinear/ISAM2.h>

#include "Scancontext.h"
//#include "utility.h"
#include "common.h"

using namespace gtsam;

class mapOptimization{
 public:
  mapOptimization();

  void run();

  void getParamFromRos();

  void allocateMemory();

  void transformAssociateToMap();

  void extractSurroundingKeyFrames();

  void downsampleCurrentScan();

  void scan2MapOptimization();

  void saveKeyFramesAndFactor();

  void correctPoses();

  void publishTF();

  void publishKeyPosesAndFrames();

  void clearCloud();

  void transformUpdate();
  void updatePointAssociateToMapSinCos();
  void pointAssociateToMap(PointType const * const pi, PointType * const po);
  void updateTransformPointCloudSinCos(PointTypePose *tIn);
  pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn);
  pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn,
                                                      PointTypePose* transformIn);

  void laserCloudOutlierLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg);
  void laserCloudRawHandler(const sensor_msgs::PointCloud2ConstPtr& msg);
  void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg);
  void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg);
  void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry);
  PointTypePose trans2PointTypePose(float transformIn[]);
  void visualizeGlobalMapThread();
  void publishGlobalMap();
  void loopClosureThread();
  bool detectLoopClosure();
  void performLoopClosure();
  Pose3 pclPointTogtsamPose3(PointTypePose thisPoint);
  Eigen::Affine3f pclPointToAffine3fCameraToLidar(PointTypePose thisPoint);
  void cornerOptimization(int iterCount);
  void surfOptimization(int iterCount);
  bool LMOptimization(int iterCount);

 private:

  NonlinearFactorGraph gtSAMgraph;
  Values initialEstimate;
  Values optimizedEstimate;
  ISAM2 *isam;
  Values isamCurrentEstimate;

  noiseModel::Diagonal::shared_ptr priorNoise;
  noiseModel::Diagonal::shared_ptr odometryNoise;
  noiseModel::Diagonal::shared_ptr constraintNoise;
  noiseModel::Base::shared_ptr robustNoiseModel;

  ros::NodeHandle nh;

  // Parameters
  std::string pointcloud_topic = "/passthrough/output";
  std::string file_directory = "/tmp/";
  bool loop_closure_enable_flag = true;
  float mapping_process_interval = 0.3;
  int history_keyframe_search_num = 25;
  float history_key_frame_search_radius = 20.0;
  float history_keyframe_fitness_score = 1.5;
  float global_map_visualization_search_radius = 1500.0;
  int surrounding_keyframe_search_num = 50;
  float surrounding_keyframe_search_radius = 50.0;
  float downsize_map_poses_factor = 0.6;
  float downsize_map_frames_factor = 0.1;

  ros::Publisher pubLaserCloudSurround;
  ros::Publisher pubOdomAftMapped;
  ros::Publisher pubKeyPoses;

  ros::Publisher pubHistoryKeyFrames;
  ros::Publisher pubIcpKeyFrames;
  ros::Publisher pubRecentKeyFrames;
  ros::Publisher pubRegisteredCloud;

  ros::Subscriber subLaserCloudRaw;
  ros::Subscriber subLaserCloudCornerLast;
  ros::Subscriber subLaserCloudSurfLast;
  ros::Subscriber subOutlierCloudLast;
  ros::Subscriber subLaserOdometry;
  ros::Subscriber subImu;

  nav_msgs::Odometry odomAftMapped;
  tf::StampedTransform aftMappedTrans;
  tf::TransformBroadcaster tfBroadcaster;

  vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
  vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
  vector<pcl::PointCloud<PointType>::Ptr> outlierCloudKeyFrames;

  deque<pcl::PointCloud<PointType>::Ptr> recentCornerCloudKeyFrames;
  deque<pcl::PointCloud<PointType>::Ptr> recentSurfCloudKeyFrames;
  deque<pcl::PointCloud<PointType>::Ptr> recentOutlierCloudKeyFrames;
  int latestFrameID;

  vector<int> surroundingExistingKeyPosesID;
  deque<pcl::PointCloud<PointType>::Ptr> surroundingCornerCloudKeyFrames;
  deque<pcl::PointCloud<PointType>::Ptr> surroundingSurfCloudKeyFrames;
  deque<pcl::PointCloud<PointType>::Ptr> surroundingOutlierCloudKeyFrames;

  PointType previousRobotPosPoint;
  PointType currentRobotPosPoint;

  pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
  pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;


  pcl::PointCloud<PointType>::Ptr surroundingKeyPoses;
  pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS;

  pcl::PointCloud<PointType>::Ptr laserCloudRaw;
  pcl::PointCloud<PointType>::Ptr laserCloudRawDS;
  pcl::PointCloud<PointType>::Ptr laserCloudCornerLast; // corner feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; // surf feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; // downsampled corner featuer set from odoOptimization
  pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS; // downsampled surf featuer set from odoOptimization

  pcl::PointCloud<PointType>::Ptr laserCloudOutlierLast; // corner feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr laserCloudOutlierLastDS; // corner feature set from odoOptimization

  pcl::PointCloud<PointType>::Ptr laserCloudSurfTotalLast; // surf feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr laserCloudSurfTotalLastDS; // downsampled corner featuer set from odoOptimization

  pcl::PointCloud<PointType>::Ptr laserCloudOri;
  pcl::PointCloud<PointType>::Ptr coeffSel;

  pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
  pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

  pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

  pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

  pcl::PointCloud<PointType>::Ptr RSlatestSurfKeyFrameCloud; // giseop, RS: radius search
  pcl::PointCloud<PointType>::Ptr RSnearHistorySurfKeyFrameCloud;
  pcl::PointCloud<PointType>::Ptr RSnearHistorySurfKeyFrameCloudDS;

  pcl::PointCloud<PointType>::Ptr nearHistoryCornerKeyFrameCloud;
  pcl::PointCloud<PointType>::Ptr nearHistoryCornerKeyFrameCloudDS;
  pcl::PointCloud<PointType>::Ptr SCnearHistorySurfKeyFrameCloud;
  pcl::PointCloud<PointType>::Ptr SCnearHistorySurfKeyFrameCloudDS;

  pcl::PointCloud<PointType>::Ptr latestCornerKeyFrameCloud;
  pcl::PointCloud<PointType>::Ptr SClatestSurfKeyFrameCloud; // giseop, SC: scan context
  pcl::PointCloud<PointType>::Ptr latestSurfKeyFrameCloudDS;

  pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap;
  pcl::PointCloud<PointType>::Ptr globalMapKeyPoses;
  pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS;
  pcl::PointCloud<PointType>::Ptr globalMapKeyFrames;
  pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS;

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  pcl::VoxelGrid<PointType> downSizeFilterScancontext;
  pcl::VoxelGrid<PointType> downSizeFilterCorner;
  pcl::VoxelGrid<PointType> downSizeFilterSurf;
  pcl::VoxelGrid<PointType> downSizeFilterOutlier;
  pcl::VoxelGrid<PointType> downSizeFilterHistoryKeyFrames; // for histor key frames of loop closure
  pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization
  pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
  pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization

  double timeLaserCloudCornerLast;
  double timeLaserCloudSurfLast;
  double timeLaserOdometry;
  double timeLaserCloudOutlierLast;
  double timeLastGloalMapPublish;

  bool newLaserCloudCornerLast;
  bool newLaserCloudSurfLast;
  bool newLaserOdometry;
  bool newLaserCloudOutlierLast;


  float transformLast[6];
  float transformSum[6];
  float transformIncre[6];
  float transformTobeMapped[6];
  float transformBefMapped[6];
  float transformAftMapped[6];

  std::mutex mtx;

  double timeLastProcessing;

  PointType pointOri, pointSel, pointProj, coeff;

  cv::Mat matA0;
  cv::Mat matB0;
  cv::Mat matX0;

  cv::Mat matA1;
  cv::Mat matD1;
  cv::Mat matV1;

  bool isDegenerate;
  cv::Mat matP;

  int laserCloudCornerFromMapDSNum;
  int laserCloudSurfFromMapDSNum;
  int laserCloudCornerLastDSNum;
  int laserCloudSurfLastDSNum;
  int laserCloudOutlierLastDSNum;
  int laserCloudSurfTotalLastDSNum;

  bool potentialLoopFlag;
  double timeSaveFirstCurrentScanForLoopClosure;
  int RSclosestHistoryFrameID;
  int SCclosestHistoryFrameID; // giseop
  int latestFrameIDLoopCloure;
  float yawDiffRad;

  bool aLoopIsClosed;

  float cRoll, sRoll, cPitch, sPitch, cYaw, sYaw, tX, tY, tZ;
  float ctRoll, stRoll, ctPitch, stPitch, ctYaw, stYaw, tInX, tInY, tInZ;

  // // loop detector
  SCManager scManager;

};


#endif //SRC_SC_LEGO_LOAM_LEGO_LOAM_INCLUDE_MAPOPTIMIZATION_H_
