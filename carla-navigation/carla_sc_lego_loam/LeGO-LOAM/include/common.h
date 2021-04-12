//
// Created by kevinlad on 2021/3/31.
//

#ifndef SC_LEGO_LOAM_INCLUDE_COMMON_H_
#define SC_LEGO_LOAM_INCLUDE_COMMON_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl_ros/point_cloud.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <memory>
#include <thread>

using PointType = pcl::PointXYZI;


struct PointXYZITR
{
  PCL_ADD_POINT4D;                    // quad-word XYZ
  std::uint8_t  intensity;                 ///< laser intensity reading
  double timestamp;
  std::uint16_t ring;                      ///< laser ring number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZITR,
                                  (float, x, x)
                                      (float, y, y)
                                      (float, z, z)
                                      (float, intensity, intensity)
                                      (double, timestamp, timestamp)
                                      (std::uint16_t, ring, ring))


struct PointXYZIRPYT
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

using PointTypePose = PointXYZIRPYT;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                       (float, z, z) (float, intensity, intensity)
                                       (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                       (double, time, time)
)

double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}

double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}

struct smoothness_t{
  float value;
  size_t ind;
};

struct by_value{
  bool operator()(smoothness_t const &left, smoothness_t const &right) {
    return left.value < right.value;
  }
};

#endif // SC_LEGO_LOAM_INCLUDE_COMMON_H_
