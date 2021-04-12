// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following papers:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.

#include "featureAssociation.h"

FeatureAssociation::FeatureAssociation() : nh("~")
{

  getParamFromRos();
  subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/segmented_cloud", 1, &FeatureAssociation::laserCloudHandler, this);
  subLaserCloudInfo = nh.subscribe<cloud_msgs::cloud_info>("/segmented_cloud_info", 1, &FeatureAssociation::laserCloudInfoHandler, this);
  subOutlierCloud = nh.subscribe<sensor_msgs::PointCloud2>("/outlier_cloud", 1, &FeatureAssociation::outlierCloudHandler, this);

  pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 1);
  pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 1);
  pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 1);
  pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 1);

  pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2);
  pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2);
  pubOutlierCloudLast = nh.advertise<sensor_msgs::PointCloud2>("/outlier_cloud_last", 2);
  pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/laser_odom_to_init", 5);

  initializationValue();
}

void FeatureAssociation::getParamFromRos() {
  nh.param<int>("lidar_channels", N_SCAN, 128);
  nh.param<int>("lidar_horizon_scans", Horizon_SCAN, 1800);
  nh.param<float>("scan_period", scan_period, 0.1);
  nh.param<float>("edge_threshold", edge_threshold, 0.1);
  nh.param<float>("surf_threshold", surf_threshold, 0.1);
  nh.param<float>("nearest_feature_search_sq_dist", nearest_feature_search_sq_dist, 15.0f);
}

void FeatureAssociation::initializationValue()
{
  cloudCurvature = new float[N_SCAN*Horizon_SCAN];
  cloudNeighborPicked = new int[N_SCAN*Horizon_SCAN];
  cloudLabel = new int[N_SCAN*Horizon_SCAN];

  pointSelCornerInd = new int[N_SCAN*Horizon_SCAN];
  pointSearchCornerInd1 = new float[N_SCAN*Horizon_SCAN];
  pointSearchCornerInd2 = new float[N_SCAN*Horizon_SCAN];

  pointSelSurfInd = new int[N_SCAN*Horizon_SCAN];
  pointSearchSurfInd1 = new float[N_SCAN*Horizon_SCAN];
  pointSearchSurfInd2 = new float[N_SCAN*Horizon_SCAN];
  pointSearchSurfInd3 = new float[N_SCAN*Horizon_SCAN];

  cloudSmoothness.resize(N_SCAN*Horizon_SCAN);

  downSizeFilter.setLeafSize(0.2, 0.2, 0.2);

  segmentedCloud.reset(new pcl::PointCloud<PointType>());
  outlierCloud.reset(new pcl::PointCloud<PointType>());

  cornerPointsSharp.reset(new pcl::PointCloud<PointType>());
  cornerPointsLessSharp.reset(new pcl::PointCloud<PointType>());
  surfPointsFlat.reset(new pcl::PointCloud<PointType>());
  surfPointsLessFlat.reset(new pcl::PointCloud<PointType>());

  surfPointsLessFlatScan.reset(new pcl::PointCloud<PointType>());
  surfPointsLessFlatScanDS.reset(new pcl::PointCloud<PointType>());

  timeScanCur = 0;
  timeNewSegmentedCloud = 0;
  timeNewSegmentedCloudInfo = 0;
  timeNewOutlierCloud = 0;

  newSegmentedCloud = false;
  newSegmentedCloudInfo = false;
  newOutlierCloud = false;

  systemInitCount = 0;
  systemInited = false;

  skipFrameNum = 1;

  for (int i = 0; i < 6; ++i){
    transformCur[i] = 0;
    transformSum[i] = 0;
  }

  systemInitedLM = false;

  laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
  laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
  laserCloudOri.reset(new pcl::PointCloud<PointType>());
  coeffSel.reset(new pcl::PointCloud<PointType>());

  kdtreeCornerLast.reset(new pcl::KdTreeFLANN<PointType>());
  kdtreeSurfLast.reset(new pcl::KdTreeFLANN<PointType>());

  laserOdometry.header.frame_id = "camera_init";
  laserOdometry.child_frame_id = "/laser_odom";

  laserOdometryTrans.frame_id_ = "camera_init";
  laserOdometryTrans.child_frame_id_ = "/laser_odom";

  isDegenerate = false;
  matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

  frameCount = skipFrameNum;
}

void FeatureAssociation::laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

  cloudHeader = laserCloudMsg->header;

  timeScanCur = cloudHeader.stamp.toSec();
  timeNewSegmentedCloud = timeScanCur;

  segmentedCloud->clear();
  pcl::fromROSMsg(*laserCloudMsg, *segmentedCloud);

  newSegmentedCloud = true;
}

void FeatureAssociation::outlierCloudHandler(const sensor_msgs::PointCloud2ConstPtr& msgIn){

  timeNewOutlierCloud = msgIn->header.stamp.toSec();

  outlierCloud->clear();
  pcl::fromROSMsg(*msgIn, *outlierCloud);

  newOutlierCloud = true;
}

void FeatureAssociation::laserCloudInfoHandler(const cloud_msgs::cloud_infoConstPtr& msgIn)
{
  timeNewSegmentedCloudInfo = msgIn->header.stamp.toSec();
  segInfo = *msgIn;
  newSegmentedCloudInfo = true;
}

void FeatureAssociation::adjustDistortion()
{
  bool halfPassed = false;
  int cloudSize = segmentedCloud->points.size();

  PointType point;

  for (int i = 0; i < cloudSize; i++) {

    point.x = segmentedCloud->points[i].y;
    point.y = segmentedCloud->points[i].z;
    point.z = segmentedCloud->points[i].x;

    float ori = -atan2(point.x, point.z);
    if (!halfPassed) {
      if (ori < segInfo.startOrientation - M_PI / 2)
        ori += 2 * M_PI;
      else if (ori > segInfo.startOrientation + M_PI * 3 / 2)
        ori -= 2 * M_PI;

      if (ori - segInfo.startOrientation > M_PI)
        halfPassed = true;
    } else {
      ori += 2 * M_PI;

      if (ori < segInfo.endOrientation - M_PI * 3 / 2)
        ori += 2 * M_PI;
      else if (ori > segInfo.endOrientation + M_PI / 2)
        ori -= 2 * M_PI;
    }

    float relTime = (ori - segInfo.startOrientation) / segInfo.orientationDiff;
    point.intensity = int(segmentedCloud->points[i].intensity) + scan_period * relTime;

    segmentedCloud->points[i] = point;
  }
}

void FeatureAssociation::calculateSmoothness()
{
  int cloudSize = segmentedCloud->points.size();
  for (int i = 5; i < cloudSize - 5; i++) {

    float diffRange = segInfo.segmentedCloudRange[i-5] + segInfo.segmentedCloudRange[i-4]
        + segInfo.segmentedCloudRange[i-3] + segInfo.segmentedCloudRange[i-2]
        + segInfo.segmentedCloudRange[i-1] - segInfo.segmentedCloudRange[i] * 10
        + segInfo.segmentedCloudRange[i+1] + segInfo.segmentedCloudRange[i+2]
        + segInfo.segmentedCloudRange[i+3] + segInfo.segmentedCloudRange[i+4]
        + segInfo.segmentedCloudRange[i+5];

    cloudCurvature[i] = diffRange*diffRange;

    cloudNeighborPicked[i] = 0;
    cloudLabel[i] = 0;

    cloudSmoothness[i].value = cloudCurvature[i];
    cloudSmoothness[i].ind = i;
  }
}

void FeatureAssociation::markOccludedPoints()
{
  int cloudSize = segmentedCloud->points.size();

  for (int i = 5; i < cloudSize - 6; ++i){

    float depth1 = segInfo.segmentedCloudRange[i];
    float depth2 = segInfo.segmentedCloudRange[i+1];
    int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[i+1] - segInfo.segmentedCloudColInd[i]));

    if (columnDiff < 10){

      if (depth1 - depth2 > 0.3){
        cloudNeighborPicked[i - 5] = 1;
        cloudNeighborPicked[i - 4] = 1;
        cloudNeighborPicked[i - 3] = 1;
        cloudNeighborPicked[i - 2] = 1;
        cloudNeighborPicked[i - 1] = 1;
        cloudNeighborPicked[i] = 1;
      }else if (depth2 - depth1 > 0.3){
        cloudNeighborPicked[i + 1] = 1;
        cloudNeighborPicked[i + 2] = 1;
        cloudNeighborPicked[i + 3] = 1;
        cloudNeighborPicked[i + 4] = 1;
        cloudNeighborPicked[i + 5] = 1;
        cloudNeighborPicked[i + 6] = 1;
      }
    }

    float diff1 = std::abs(float(segInfo.segmentedCloudRange[i-1] - segInfo.segmentedCloudRange[i]));
    float diff2 = std::abs(float(segInfo.segmentedCloudRange[i+1] - segInfo.segmentedCloudRange[i]));

    if (diff1 > 0.02 * segInfo.segmentedCloudRange[i] && diff2 > 0.02 * segInfo.segmentedCloudRange[i])
      cloudNeighborPicked[i] = 1;
  }
}

void FeatureAssociation::extractFeatures()
{
  cornerPointsSharp->clear();
  cornerPointsLessSharp->clear();
  surfPointsFlat->clear();
  surfPointsLessFlat->clear();

  for (int i = 0; i < N_SCAN; i++) {

    surfPointsLessFlatScan->clear();

    for (int j = 0; j < 6; j++) {

      int sp = (segInfo.startRingIndex[i] * (6 - j)    + segInfo.endRingIndex[i] * j) / 6;
      int ep = (segInfo.startRingIndex[i] * (5 - j)    + segInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

      if (sp >= ep)
        continue;

      std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = cloudSmoothness[k].ind;
        if (cloudNeighborPicked[ind] == 0 &&
            cloudCurvature[ind] > edge_threshold &&
            segInfo.segmentedCloudGroundFlag[ind] == false) {

          largestPickedNum++;
          if (largestPickedNum <= 2) {
            cloudLabel[ind] = 2;
            cornerPointsSharp->push_back(segmentedCloud->points[ind]);
            cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
          } else if (largestPickedNum <= 20) {
            cloudLabel[ind] = 1;
            cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
          } else {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l - 1]));
            if (columnDiff > 10)
              break;
            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l + 1]));
            if (columnDiff > 10)
              break;
            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      int smallestPickedNum = 0;
      for (int k = sp; k <= ep; k++) {
        int ind = cloudSmoothness[k].ind;
        if (cloudNeighborPicked[ind] == 0 &&
            cloudCurvature[ind] < surf_threshold &&
            segInfo.segmentedCloudGroundFlag[ind] == true) {

          cloudLabel[ind] = -1;
          surfPointsFlat->push_back(segmentedCloud->points[ind]);

          smallestPickedNum++;
          if (smallestPickedNum >= 4) {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {

            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l - 1]));
            if (columnDiff > 10)
              break;

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {

            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l + 1]));
            if (columnDiff > 10)
              break;

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        if (cloudLabel[k] <= 0) {
          surfPointsLessFlatScan->push_back(segmentedCloud->points[k]);
        }
      }
    }

    surfPointsLessFlatScanDS->clear();
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.filter(*surfPointsLessFlatScanDS);

    *surfPointsLessFlat += *surfPointsLessFlatScanDS;
  }
}

void FeatureAssociation::AccumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz,
                                            float &ox, float &oy, float &oz)
{
  /*R_wl=[ccy 0 scy;0 1 0;-scy 0 ccy]*[1 0 0;0 ccx -scx;0 scx ccx]*[ccz -scz 0;scz ccz 0;0 0 1];（表示以world为参考坐标系）
  *R_cl=[clz -slz 0;slz clz 0;0 0 1]*[1 0 0;0 clx -slx;0 slx clx]*[cly 0 sly;0 1 0;-sly 0 cly];（表示以current为参考坐标系）
  *R_wc=R_wl*(R_cl).';
  *最后求出来(-sin(rx))=cos(cx)*cos(cz)*sin(lx) - cos(lx)*cos(ly)*sin(cx) - cos(cx)*cos(lx)*sin(cz)*sin(ly)
  *而程序中是(-sin(rx))= cos(lx)*cos(cx)*sin(ly)*sin(cz) - cos(cx)*cos(cz)*sin(lx) - cos(lx)*cos(ly)*sin(cx);（程序里的srx=(-sin(rx))）
  *可以发现两个公式之间差了lx,ly,lz的负号，所以accumulateRotation()函数传入的是transform[0]~[2]的负值
  *至于为什么-sinx等于上式，可以通过看R_wl，发现第二行第三列的元素为-sinx，因此两个旋转矩阵相乘后，对应位置上的元素就对应着新的pitch角的sin 值
  */
  float srx = cos(lx)*cos(cx)*sin(ly)*sin(cz) - cos(cx)*cos(cz)*sin(lx) - cos(lx)*cos(ly)*sin(cx);
  ox = -asin(srx);

  float srycrx = sin(lx)*(cos(cy)*sin(cz) - cos(cz)*sin(cx)*sin(cy)) + cos(lx)*sin(ly)*(cos(cy)*cos(cz)
      + sin(cx)*sin(cy)*sin(cz)) + cos(lx)*cos(ly)*cos(cx)*sin(cy);
  float crycrx = cos(lx)*cos(ly)*cos(cx)*cos(cy) - cos(lx)*sin(ly)*(cos(cz)*sin(cy)
      - cos(cy)*sin(cx)*sin(cz)) - sin(lx)*(sin(cy)*sin(cz) + cos(cy)*cos(cz)*sin(cx));
  oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

  float srzcrx = sin(cx)*(cos(lz)*sin(ly) - cos(ly)*sin(lx)*sin(lz)) + cos(cx)*sin(cz)*(cos(ly)*cos(lz)
      + sin(lx)*sin(ly)*sin(lz)) + cos(lx)*cos(cx)*cos(cz)*sin(lz);
  float crzcrx = cos(lx)*cos(lz)*cos(cx)*cos(cz) - cos(cx)*sin(cz)*(cos(ly)*sin(lz)
      - cos(lz)*sin(lx)*sin(ly)) - sin(cx)*(sin(ly)*sin(lz) + cos(ly)*cos(lz)*sin(lx));
  oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
}

void FeatureAssociation::publishCloud()
{
  sensor_msgs::PointCloud2 laserCloudOutMsg;

  if (pubCornerPointsSharp.getNumSubscribers() != 0){
    pcl::toROSMsg(*cornerPointsSharp, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = cloudHeader.stamp;
    laserCloudOutMsg.header.frame_id = "camera";
    pubCornerPointsSharp.publish(laserCloudOutMsg);
  }

  if (pubCornerPointsLessSharp.getNumSubscribers() != 0){
    pcl::toROSMsg(*cornerPointsLessSharp, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = cloudHeader.stamp;
    laserCloudOutMsg.header.frame_id = "camera";
    pubCornerPointsLessSharp.publish(laserCloudOutMsg);
  }

  if (pubSurfPointsFlat.getNumSubscribers() != 0){
    pcl::toROSMsg(*surfPointsFlat, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = cloudHeader.stamp;
    laserCloudOutMsg.header.frame_id = "camera";
    pubSurfPointsFlat.publish(laserCloudOutMsg);
  }

  if (pubSurfPointsLessFlat.getNumSubscribers() != 0){
    pcl::toROSMsg(*surfPointsLessFlat, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = cloudHeader.stamp;
    laserCloudOutMsg.header.frame_id = "camera";
    pubSurfPointsLessFlat.publish(laserCloudOutMsg);
  }
}

void FeatureAssociation::TransformToStart(PointType const * const pi, PointType * const po)
{
  float s = 10 * (pi->intensity - int(pi->intensity));

  float rx = s * transformCur[0];
  float ry = s * transformCur[1];
  float rz = s * transformCur[2];
  float tx = s * transformCur[3];
  float ty = s * transformCur[4];
  float tz = s * transformCur[5];

  float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
  float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
  float z1 = (pi->z - tz);

  float x2 = x1;
  float y2 = cos(rx) * y1 + sin(rx) * z1;
  float z2 = -sin(rx) * y1 + cos(rx) * z1;

  po->x = cos(ry) * x2 - sin(ry) * z2;
  po->y = y2;
  po->z = sin(ry) * x2 + cos(ry) * z2;
  po->intensity = pi->intensity;
}

void FeatureAssociation::TransformToEnd(PointType const * const pi, PointType * const po)
{
  float s = 10 * (pi->intensity - int(pi->intensity));

  float rx = s * transformCur[0];
  float ry = s * transformCur[1];
  float rz = s * transformCur[2];
  float tx = s * transformCur[3];
  float ty = s * transformCur[4];
  float tz = s * transformCur[5];

  float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
  float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
  float z1 = (pi->z - tz);

  float x2 = x1;
  float y2 = cos(rx) * y1 + sin(rx) * z1;
  float z2 = -sin(rx) * y1 + cos(rx) * z1;

  float x3 = cos(ry) * x2 - sin(ry) * z2;
  float y3 = y2;
  float z3 = sin(ry) * x2 + cos(ry) * z2;

  rx = transformCur[0];
  ry = transformCur[1];
  rz = transformCur[2];
  tx = transformCur[3];
  ty = transformCur[4];
  tz = transformCur[5];

  float x4 = cos(ry) * x3 + sin(ry) * z3;
  float y4 = y3;
  float z4 = -sin(ry) * x3 + cos(ry) * z3;

  float x5 = x4;
  float y5 = cos(rx) * y4 - sin(rx) * z4;
  float z5 = sin(rx) * y4 + cos(rx) * z4;

  float x6 = cos(rz) * x5 - sin(rz) * y5 + tx;
  float y6 = sin(rz) * x5 + cos(rz) * y5 + ty;
  float z6 = z5 + tz;

  po->x = x6;
  po->y = y6;
  po->z = z6;
  po->intensity = int(pi->intensity);
}

void FeatureAssociation::findCorrespondingCornerFeatures(int iterCount){

  int cornerPointsSharpNum = cornerPointsSharp->points.size();

  for (int i = 0; i < cornerPointsSharpNum; i++) {

    TransformToStart(&cornerPointsSharp->points[i], &pointSel);

    if (iterCount % 5 == 0) {

      kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
      int closestPointInd = -1, minPointInd2 = -1;

      if (pointSearchSqDis[0] < nearest_feature_search_sq_dist) {
        closestPointInd = pointSearchInd[0];
        int closestPointScan = int(laserCloudCornerLast->points[closestPointInd].intensity);

        float pointSqDis, minPointSqDis2 = nearest_feature_search_sq_dist;
        for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) {
          if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan + 2.5) {
            break;
          }

          pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
              (laserCloudCornerLast->points[j].x - pointSel.x) +
              (laserCloudCornerLast->points[j].y - pointSel.y) *
                  (laserCloudCornerLast->points[j].y - pointSel.y) +
              (laserCloudCornerLast->points[j].z - pointSel.z) *
                  (laserCloudCornerLast->points[j].z - pointSel.z);

          if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan) {
            if (pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          }
        }
        for (int j = closestPointInd - 1; j >= 0; j--) {
          if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan - 2.5) {
            break;
          }

          pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
              (laserCloudCornerLast->points[j].x - pointSel.x) +
              (laserCloudCornerLast->points[j].y - pointSel.y) *
                  (laserCloudCornerLast->points[j].y - pointSel.y) +
              (laserCloudCornerLast->points[j].z - pointSel.z) *
                  (laserCloudCornerLast->points[j].z - pointSel.z);

          if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan) {
            if (pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          }
        }
      }

      pointSearchCornerInd1[i] = closestPointInd;
      pointSearchCornerInd2[i] = minPointInd2;
    }

    if (pointSearchCornerInd2[i] >= 0) {

      tripod1 = laserCloudCornerLast->points[pointSearchCornerInd1[i]];
      tripod2 = laserCloudCornerLast->points[pointSearchCornerInd2[i]];

      float x0 = pointSel.x;
      float y0 = pointSel.y;
      float z0 = pointSel.z;
      float x1 = tripod1.x;
      float y1 = tripod1.y;
      float z1 = tripod1.z;
      float x2 = tripod2.x;
      float y2 = tripod2.y;
      float z2 = tripod2.z;

      float m11 = ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1));
      float m22 = ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1));
      float m33 = ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1));

      float a012 = sqrt(m11 * m11  + m22 * m22 + m33 * m33);

      float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

      float la =  ((y1 - y2)*m11 + (z1 - z2)*m22) / a012 / l12;

      float lb = -((x1 - x2)*m11 - (z1 - z2)*m33) / a012 / l12;

      float lc = -((x1 - x2)*m22 + (y1 - y2)*m33) / a012 / l12;

      float ld2 = a012 / l12;

      float s = 1;
      if (iterCount >= 5) {
        s = 1 - 1.8 * fabs(ld2);
      }

      if (s > 0.1 && ld2 != 0) {
        coeff.x = s * la;
        coeff.y = s * lb;
        coeff.z = s * lc;
        coeff.intensity = s * ld2;

        laserCloudOri->push_back(cornerPointsSharp->points[i]);
        coeffSel->push_back(coeff);
      }
    }
  }
}

void FeatureAssociation::findCorrespondingSurfFeatures(int iterCount){

  int surfPointsFlatNum = surfPointsFlat->points.size();

  for (int i = 0; i < surfPointsFlatNum; i++) {

    TransformToStart(&surfPointsFlat->points[i], &pointSel);

    if (iterCount % 5 == 0) {

      kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
      int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;

      if (pointSearchSqDis[0] < nearest_feature_search_sq_dist) {
        closestPointInd = pointSearchInd[0];
        int closestPointScan = int(laserCloudSurfLast->points[closestPointInd].intensity);

        float pointSqDis, minPointSqDis2 = nearest_feature_search_sq_dist, minPointSqDis3 = nearest_feature_search_sq_dist;
        for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
          if (int(laserCloudSurfLast->points[j].intensity) > closestPointScan + 2.5) {
            break;
          }

          pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
              (laserCloudSurfLast->points[j].x - pointSel.x) +
              (laserCloudSurfLast->points[j].y - pointSel.y) *
                  (laserCloudSurfLast->points[j].y - pointSel.y) +
              (laserCloudSurfLast->points[j].z - pointSel.z) *
                  (laserCloudSurfLast->points[j].z - pointSel.z);

          if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScan) {
            if (pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          } else {
            if (pointSqDis < minPointSqDis3) {
              minPointSqDis3 = pointSqDis;
              minPointInd3 = j;
            }
          }
        }
        for (int j = closestPointInd - 1; j >= 0; j--) {
          if (int(laserCloudSurfLast->points[j].intensity) < closestPointScan - 2.5) {
            break;
          }

          pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
              (laserCloudSurfLast->points[j].x - pointSel.x) +
              (laserCloudSurfLast->points[j].y - pointSel.y) *
                  (laserCloudSurfLast->points[j].y - pointSel.y) +
              (laserCloudSurfLast->points[j].z - pointSel.z) *
                  (laserCloudSurfLast->points[j].z - pointSel.z);

          if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScan) {
            if (pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          } else {
            if (pointSqDis < minPointSqDis3) {
              minPointSqDis3 = pointSqDis;
              minPointInd3 = j;
            }
          }
        }
      }

      pointSearchSurfInd1[i] = closestPointInd;
      pointSearchSurfInd2[i] = minPointInd2;
      pointSearchSurfInd3[i] = minPointInd3;
    }

    if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0) {

      tripod1 = laserCloudSurfLast->points[pointSearchSurfInd1[i]];
      tripod2 = laserCloudSurfLast->points[pointSearchSurfInd2[i]];
      tripod3 = laserCloudSurfLast->points[pointSearchSurfInd3[i]];

      float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z)
          - (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
      float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x)
          - (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
      float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y)
          - (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
      float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

      float ps = sqrt(pa * pa + pb * pb + pc * pc);

      pa /= ps;
      pb /= ps;
      pc /= ps;
      pd /= ps;

      float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

      float s = 1;
      if (iterCount >= 5) {
        s = 1 - 1.8 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
                                                + pointSel.y * pointSel.y + pointSel.z * pointSel.z));
      }

      if (s > 0.1 && pd2 != 0) {
        coeff.x = s * pa;
        coeff.y = s * pb;
        coeff.z = s * pc;
        coeff.intensity = s * pd2;

        laserCloudOri->push_back(surfPointsFlat->points[i]);
        coeffSel->push_back(coeff);
      }
    }
  }
}

bool FeatureAssociation::calculateTransformationSurf(int iterCount){

  int pointSelNum = laserCloudOri->points.size();

  cv::Mat matA(pointSelNum, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matAt(3, pointSelNum, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));

  float srx = sin(transformCur[0]);
  float crx = cos(transformCur[0]);
  float sry = sin(transformCur[1]);
  float cry = cos(transformCur[1]);
  float srz = sin(transformCur[2]);
  float crz = cos(transformCur[2]);
  float tx = transformCur[3];
  float ty = transformCur[4];
  float tz = transformCur[5];

  float a1 = crx*sry*srz; float a2 = crx*crz*sry; float a3 = srx*sry; float a4 = tx*a1 - ty*a2 - tz*a3;
  float a5 = srx*srz; float a6 = crz*srx; float a7 = ty*a6 - tz*crx - tx*a5;
  float a8 = crx*cry*srz; float a9 = crx*cry*crz; float a10 = cry*srx; float a11 = tz*a10 + ty*a9 - tx*a8;

  float b1 = -crz*sry - cry*srx*srz; float b2 = cry*crz*srx - sry*srz;
  float b5 = cry*crz - srx*sry*srz; float b6 = cry*srz + crz*srx*sry;

  float c1 = -b6; float c2 = b5; float c3 = tx*b6 - ty*b5; float c4 = -crx*crz; float c5 = crx*srz; float c6 = ty*c5 + tx*-c4;
  float c7 = b2; float c8 = -b1; float c9 = tx*-b2 - ty*-b1;

  for (int i = 0; i < pointSelNum; i++) {

    pointOri = laserCloudOri->points[i];
    coeff = coeffSel->points[i];

    float arx = (-a1*pointOri.x + a2*pointOri.y + a3*pointOri.z + a4) * coeff.x
        + (a5*pointOri.x - a6*pointOri.y + crx*pointOri.z + a7) * coeff.y
        + (a8*pointOri.x - a9*pointOri.y - a10*pointOri.z + a11) * coeff.z;

    float arz = (c1*pointOri.x + c2*pointOri.y + c3) * coeff.x
        + (c4*pointOri.x - c5*pointOri.y + c6) * coeff.y
        + (c7*pointOri.x + c8*pointOri.y + c9) * coeff.z;

    float aty = -b6 * coeff.x + c4 * coeff.y + b2 * coeff.z;

    float d2 = coeff.intensity;

    matA.at<float>(i, 0) = arx;
    matA.at<float>(i, 1) = arz;
    matA.at<float>(i, 2) = aty;
    matB.at<float>(i, 0) = -0.05 * d2;
  }

  cv::transpose(matA, matAt);
  matAtA = matAt * matA;
  matAtB = matAt * matB;
  cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

  if (iterCount == 0) {
    cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));

    cv::eigen(matAtA, matE, matV);
    matV.copyTo(matV2);

    isDegenerate = false;
    float eignThre[3] = {10, 10, 10};
    for (int i = 2; i >= 0; i--) {
      if (matE.at<float>(0, i) < eignThre[i]) {
        for (int j = 0; j < 3; j++) {
          matV2.at<float>(i, j) = 0;
        }
        isDegenerate = true;
      } else {
        break;
      }
    }
    matP = matV.inv() * matV2;
  }

  if (isDegenerate) {
    cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
    matX.copyTo(matX2);
    matX = matP * matX2;
  }

  transformCur[0] += matX.at<float>(0, 0);
  transformCur[2] += matX.at<float>(1, 0);
  transformCur[4] += matX.at<float>(2, 0);

  for(int i=0; i<6; i++){
    if(isnan(transformCur[i]))
      transformCur[i]=0;
  }

  float deltaR = sqrt(
      pow(rad2deg(matX.at<float>(0, 0)), 2) +
          pow(rad2deg(matX.at<float>(1, 0)), 2));
  float deltaT = sqrt(
      pow(matX.at<float>(2, 0) * 100, 2));

  if (deltaR < 0.1 && deltaT < 0.1) {
    return false;
  }
  return true;
}

bool FeatureAssociation::calculateTransformationCorner(int iterCount){

  int pointSelNum = laserCloudOri->points.size();

  cv::Mat matA(pointSelNum, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matAt(3, pointSelNum, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));

  float srx = sin(transformCur[0]);
  float crx = cos(transformCur[0]);
  float sry = sin(transformCur[1]);
  float cry = cos(transformCur[1]);
  float srz = sin(transformCur[2]);
  float crz = cos(transformCur[2]);
  float tx = transformCur[3];
  float ty = transformCur[4];
  float tz = transformCur[5];

  float b1 = -crz*sry - cry*srx*srz; float b2 = cry*crz*srx - sry*srz; float b3 = crx*cry; float b4 = tx*-b1 + ty*-b2 + tz*b3;
  float b5 = cry*crz - srx*sry*srz; float b6 = cry*srz + crz*srx*sry; float b7 = crx*sry; float b8 = tz*b7 - ty*b6 - tx*b5;

  float c5 = crx*srz;

  for (int i = 0; i < pointSelNum; i++) {

    pointOri = laserCloudOri->points[i];
    coeff = coeffSel->points[i];

    float ary = (b1*pointOri.x + b2*pointOri.y - b3*pointOri.z + b4) * coeff.x
        + (b5*pointOri.x + b6*pointOri.y - b7*pointOri.z + b8) * coeff.z;

    float atx = -b5 * coeff.x + c5 * coeff.y + b1 * coeff.z;

    float atz = b7 * coeff.x - srx * coeff.y - b3 * coeff.z;

    float d2 = coeff.intensity;

    matA.at<float>(i, 0) = ary;
    matA.at<float>(i, 1) = atx;
    matA.at<float>(i, 2) = atz;
    matB.at<float>(i, 0) = -0.05 * d2;
  }

  cv::transpose(matA, matAt);
  matAtA = matAt * matA;
  matAtB = matAt * matB;
  cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

  if (iterCount == 0) {
    cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));

    cv::eigen(matAtA, matE, matV);
    matV.copyTo(matV2);

    isDegenerate = false;
    float eignThre[3] = {10, 10, 10};
    for (int i = 2; i >= 0; i--) {
      if (matE.at<float>(0, i) < eignThre[i]) {
        for (int j = 0; j < 3; j++) {
          matV2.at<float>(i, j) = 0;
        }
        isDegenerate = true;
      } else {
        break;
      }
    }
    matP = matV.inv() * matV2;
  }

  if (isDegenerate) {
    cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
    matX.copyTo(matX2);
    matX = matP * matX2;
  }

  transformCur[1] += matX.at<float>(0, 0);
  transformCur[3] += matX.at<float>(1, 0);
  transformCur[5] += matX.at<float>(2, 0);

  for(int i=0; i<6; i++){
    if(isnan(transformCur[i]))
      transformCur[i]=0;
  }

  float deltaR = sqrt(
      pow(rad2deg(matX.at<float>(0, 0)), 2));
  float deltaT = sqrt(
      pow(matX.at<float>(1, 0) * 100, 2) +
          pow(matX.at<float>(2, 0) * 100, 2));

  if (deltaR < 0.1 && deltaT < 0.1) {
    return false;
  }
  return true;
}

bool FeatureAssociation::calculateTransformation(int iterCount){

  int pointSelNum = laserCloudOri->points.size();

  cv::Mat matA(pointSelNum, 6, CV_32F, cv::Scalar::all(0));
  cv::Mat matAt(6, pointSelNum, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
  cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

  float srx = sin(transformCur[0]);
  float crx = cos(transformCur[0]);
  float sry = sin(transformCur[1]);
  float cry = cos(transformCur[1]);
  float srz = sin(transformCur[2]);
  float crz = cos(transformCur[2]);
  float tx = transformCur[3];
  float ty = transformCur[4];
  float tz = transformCur[5];

  float a1 = crx*sry*srz; float a2 = crx*crz*sry; float a3 = srx*sry; float a4 = tx*a1 - ty*a2 - tz*a3;
  float a5 = srx*srz; float a6 = crz*srx; float a7 = ty*a6 - tz*crx - tx*a5;
  float a8 = crx*cry*srz; float a9 = crx*cry*crz; float a10 = cry*srx; float a11 = tz*a10 + ty*a9 - tx*a8;

  float b1 = -crz*sry - cry*srx*srz; float b2 = cry*crz*srx - sry*srz; float b3 = crx*cry; float b4 = tx*-b1 + ty*-b2 + tz*b3;
  float b5 = cry*crz - srx*sry*srz; float b6 = cry*srz + crz*srx*sry; float b7 = crx*sry; float b8 = tz*b7 - ty*b6 - tx*b5;

  float c1 = -b6; float c2 = b5; float c3 = tx*b6 - ty*b5; float c4 = -crx*crz; float c5 = crx*srz; float c6 = ty*c5 + tx*-c4;
  float c7 = b2; float c8 = -b1; float c9 = tx*-b2 - ty*-b1;

  for (int i = 0; i < pointSelNum; i++) {

    pointOri = laserCloudOri->points[i];
    coeff = coeffSel->points[i];

    float arx = (-a1*pointOri.x + a2*pointOri.y + a3*pointOri.z + a4) * coeff.x
        + (a5*pointOri.x - a6*pointOri.y + crx*pointOri.z + a7) * coeff.y
        + (a8*pointOri.x - a9*pointOri.y - a10*pointOri.z + a11) * coeff.z;

    float ary = (b1*pointOri.x + b2*pointOri.y - b3*pointOri.z + b4) * coeff.x
        + (b5*pointOri.x + b6*pointOri.y - b7*pointOri.z + b8) * coeff.z;

    float arz = (c1*pointOri.x + c2*pointOri.y + c3) * coeff.x
        + (c4*pointOri.x - c5*pointOri.y + c6) * coeff.y
        + (c7*pointOri.x + c8*pointOri.y + c9) * coeff.z;

    float atx = -b5 * coeff.x + c5 * coeff.y + b1 * coeff.z;

    float aty = -b6 * coeff.x + c4 * coeff.y + b2 * coeff.z;

    float atz = b7 * coeff.x - srx * coeff.y - b3 * coeff.z;

    float d2 = coeff.intensity;

    matA.at<float>(i, 0) = arx;
    matA.at<float>(i, 1) = ary;
    matA.at<float>(i, 2) = arz;
    matA.at<float>(i, 3) = atx;
    matA.at<float>(i, 4) = aty;
    matA.at<float>(i, 5) = atz;
    matB.at<float>(i, 0) = -0.05 * d2;
  }

  cv::transpose(matA, matAt);
  matAtA = matAt * matA;
  matAtB = matAt * matB;
  cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

  if (iterCount == 0) {
    cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

    cv::eigen(matAtA, matE, matV);
    matV.copyTo(matV2);

    isDegenerate = false;
    float eignThre[6] = {10, 10, 10, 10, 10, 10};
    for (int i = 5; i >= 0; i--) {
      if (matE.at<float>(0, i) < eignThre[i]) {
        for (int j = 0; j < 6; j++) {
          matV2.at<float>(i, j) = 0;
        }
        isDegenerate = true;
      } else {
        break;
      }
    }
    matP = matV.inv() * matV2;
  }

  if (isDegenerate) {
    cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
    matX.copyTo(matX2);
    matX = matP * matX2;
  }

  transformCur[0] += matX.at<float>(0, 0);
  transformCur[1] += matX.at<float>(1, 0);
  transformCur[2] += matX.at<float>(2, 0);
  transformCur[3] += matX.at<float>(3, 0);
  transformCur[4] += matX.at<float>(4, 0);
  transformCur[5] += matX.at<float>(5, 0);

  for(int i=0; i<6; i++){
    if(isnan(transformCur[i]))
      transformCur[i]=0;
  }

  float deltaR = sqrt(
      pow(rad2deg(matX.at<float>(0, 0)), 2) +
          pow(rad2deg(matX.at<float>(1, 0)), 2) +
          pow(rad2deg(matX.at<float>(2, 0)), 2));
  float deltaT = sqrt(
      pow(matX.at<float>(3, 0) * 100, 2) +
          pow(matX.at<float>(4, 0) * 100, 2) +
          pow(matX.at<float>(5, 0) * 100, 2));

  if (deltaR < 0.1 && deltaT < 0.1) {
    return false;
  }
  return true;
}

void FeatureAssociation::checkSystemInitialization(){

  pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
  cornerPointsLessSharp = laserCloudCornerLast;
  laserCloudCornerLast = laserCloudTemp;

  laserCloudTemp = surfPointsLessFlat;
  surfPointsLessFlat = laserCloudSurfLast;
  laserCloudSurfLast = laserCloudTemp;

  kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
  kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

  laserCloudCornerLastNum = laserCloudCornerLast->points.size();
  laserCloudSurfLastNum = laserCloudSurfLast->points.size();

  sensor_msgs::PointCloud2 laserCloudCornerLast2;
  pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
  laserCloudCornerLast2.header.stamp = cloudHeader.stamp;
  laserCloudCornerLast2.header.frame_id = "camera";
  pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

  sensor_msgs::PointCloud2 laserCloudSurfLast2;
  pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
  laserCloudSurfLast2.header.stamp = cloudHeader.stamp;
  laserCloudSurfLast2.header.frame_id = "camera";
  pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

  systemInitedLM = true;
}


void FeatureAssociation::updateTransformation(){

  if (laserCloudCornerLastNum < 10 || laserCloudSurfLastNum < 100)
    return;

  for (int iterCount1 = 0; iterCount1 < 25; iterCount1++) {
    laserCloudOri->clear();
    coeffSel->clear();

    findCorrespondingSurfFeatures(iterCount1);

    if (laserCloudOri->points.size() < 10)
      continue;
    if (!calculateTransformationSurf(iterCount1))
      break;
  }

  for (int iterCount2 = 0; iterCount2 < 25; iterCount2++) {

    laserCloudOri->clear();
    coeffSel->clear();

    findCorrespondingCornerFeatures(iterCount2);

    if (laserCloudOri->points.size() < 10)
      continue;
    if (!calculateTransformationCorner(iterCount2))
      break;
  }
}

void FeatureAssociation::integrateTransformation(){
  float rx, ry, rz, tx, ty, tz;
  AccumulateRotation(transformSum[0], transformSum[1], transformSum[2],
                     -transformCur[0], -transformCur[1], -transformCur[2], rx, ry, rz);

  float x1 = cos(rz) * (transformCur[3] - 0)
      - sin(rz) * (transformCur[4]);
  float y1 = sin(rz) * (transformCur[3] - 0)
      + cos(rz) * (transformCur[4]);
  float z1 = transformCur[5];

  float x2 = x1;
  float y2 = cos(rx) * y1 - sin(rx) * z1;
  float z2 = sin(rx) * y1 + cos(rx) * z1;

  tx = transformSum[3] - (cos(ry) * x2 + sin(ry) * z2);
  ty = transformSum[4] - y2;
  tz = transformSum[5] - (-sin(ry) * x2 + cos(ry) * z2);

  transformSum[0] = rx;
  transformSum[1] = ry;
  transformSum[2] = rz;
  transformSum[3] = tx;
  transformSum[4] = ty;
  transformSum[5] = tz;
}

void FeatureAssociation::publishOdometry(){
  geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformSum[2], -transformSum[0], -transformSum[1]);

  laserOdometry.header.stamp = cloudHeader.stamp;
  laserOdometry.pose.pose.orientation.x = -geoQuat.y;
  laserOdometry.pose.pose.orientation.y = -geoQuat.z;
  laserOdometry.pose.pose.orientation.z = geoQuat.x;
  laserOdometry.pose.pose.orientation.w = geoQuat.w;
  laserOdometry.pose.pose.position.x = transformSum[3];
  laserOdometry.pose.pose.position.y = transformSum[4];
  laserOdometry.pose.pose.position.z = transformSum[5];
  pubLaserOdometry.publish(laserOdometry);

  laserOdometryTrans.stamp_ = cloudHeader.stamp;
  laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
  laserOdometryTrans.setOrigin(tf::Vector3(transformSum[3], transformSum[4], transformSum[5]));
  tfBroadcaster.sendTransform(laserOdometryTrans);
}

void FeatureAssociation::adjustOutlierCloud(){
  PointType point;
  int cloudSize = outlierCloud->points.size();
  for (int i = 0; i < cloudSize; ++i)
  {
    point.x = outlierCloud->points[i].y;
    point.y = outlierCloud->points[i].z;
    point.z = outlierCloud->points[i].x;
    point.intensity = outlierCloud->points[i].intensity;
    outlierCloud->points[i] = point;
  }
}


void FeatureAssociation::publishCloudsLast(){

  int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
  for (int i = 0; i < cornerPointsLessSharpNum; i++) {
    TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
  }


  int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
  for (int i = 0; i < surfPointsLessFlatNum; i++) {
    TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
  }

  pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
  cornerPointsLessSharp = laserCloudCornerLast;
  laserCloudCornerLast = laserCloudTemp;

  laserCloudTemp = surfPointsLessFlat;
  surfPointsLessFlat = laserCloudSurfLast;
  laserCloudSurfLast = laserCloudTemp;

  laserCloudCornerLastNum = laserCloudCornerLast->points.size();
  laserCloudSurfLastNum = laserCloudSurfLast->points.size();

  if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
    kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
    kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
  }

  frameCount++;

  if (frameCount >= skipFrameNum + 1) {

    frameCount = 0;

    adjustOutlierCloud();
    sensor_msgs::PointCloud2 outlierCloudLast2;
    pcl::toROSMsg(*outlierCloud, outlierCloudLast2);
    outlierCloudLast2.header.stamp = cloudHeader.stamp;
    outlierCloudLast2.header.frame_id = "camera";
    pubOutlierCloudLast.publish(outlierCloudLast2);

    sensor_msgs::PointCloud2 laserCloudCornerLast2;
    pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
    laserCloudCornerLast2.header.stamp = cloudHeader.stamp;
    laserCloudCornerLast2.header.frame_id = "camera";
    pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

    sensor_msgs::PointCloud2 laserCloudSurfLast2;
    pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
    laserCloudSurfLast2.header.stamp = cloudHeader.stamp;
    laserCloudSurfLast2.header.frame_id = "camera";
    pubLaserCloudSurfLast.publish(laserCloudSurfLast2);
  }
}

void FeatureAssociation::runFeatureAssociation()
{

  if (newSegmentedCloud && newSegmentedCloudInfo && newOutlierCloud &&
      std::abs(timeNewSegmentedCloudInfo - timeNewSegmentedCloud) < 0.05 &&
      std::abs(timeNewOutlierCloud - timeNewSegmentedCloud) < 0.05){

    newSegmentedCloud = false;
    newSegmentedCloudInfo = false;
    newOutlierCloud = false;
  }else{
    return;
  }
  /**
      1. Feature Extraction
  */
  adjustDistortion();

  calculateSmoothness();

  markOccludedPoints();

  extractFeatures();

  publishCloud(); // cloud for visualization

  /**
  2. Feature Association
  */
  if (!systemInitedLM) {
    checkSystemInitialization();
    return;
  }

  updateTransformation();

  integrateTransformation();

  publishOdometry();

  publishCloudsLast(); // cloud to mapOptimization
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lego_loam");

  ROS_INFO("\033[1;32m---->\033[0m Feature Association Started.");

  FeatureAssociation FA;

  ros::Rate rate(200);
  while (ros::ok())
  {
    ros::spinOnce();

    FA.runFeatureAssociation();

    rate.sleep();
  }

  ros::spin();
  return 0;
}
