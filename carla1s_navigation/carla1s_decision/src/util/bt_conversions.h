// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_
#define NAV2_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_

#include <string>

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>

#include <carla_nav_types/common_types.h>

namespace BT
{

// The follow templates are required when using these types as parameters
// in our BT XML files. They parse the strings in the XML into their corresponding
// data type.

/**
 * @brief Parse XML string to geometry_msgs::msg::Point
 * @param key XML string
 * @return geometry_msgs::msg::Point
 */
template<>
inline geometry_msgs::Point convertFromString(const StringView key)
{
  // three real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 3) {
    throw std::runtime_error("invalid number of fields for point attribute)");
  } else {
    geometry_msgs::Point position;
    position.x = BT::convertFromString<double>(parts[0]);
    position.y = BT::convertFromString<double>(parts[1]);
    position.z = BT::convertFromString<double>(parts[2]);
    return position;
  }
}

/**
 * @brief Parse XML string to geometry_msgs::msg::Quaternion
 * @param key XML string
 * @return geometry_msgs::msg::Quaternion
 */
template<>
inline geometry_msgs::Quaternion convertFromString(const StringView key)
{
  // four real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 4) {
    throw std::runtime_error("invalid number of fields for orientation attribute)");
  } else {
    geometry_msgs::Quaternion orientation;
    orientation.x = BT::convertFromString<double>(parts[0]);
    orientation.y = BT::convertFromString<double>(parts[1]);
    orientation.z = BT::convertFromString<double>(parts[2]);
    orientation.w = BT::convertFromString<double>(parts[3]);
    return orientation;
  }
}

/**
 * @brief Parse XML string to geometry_msgs::PoseStamped
 * @param key XML string
 * @return geometry_msgs::PoseStamped
 */
template<>
inline geometry_msgs::PoseStamped convertFromString(const StringView key)
{
  // 7 real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');

  if(parts.size() == 9){
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(BT::convertFromString<int64_t>(parts[0]));
    pose_stamped.header.frame_id = BT::convertFromString<std::string>(parts[1]);
    pose_stamped.pose.position.x = BT::convertFromString<double>(parts[2]);
    pose_stamped.pose.position.y = BT::convertFromString<double>(parts[3]);
    pose_stamped.pose.position.z = BT::convertFromString<double>(parts[4]);
    pose_stamped.pose.orientation.x = BT::convertFromString<double>(parts[5]);
    pose_stamped.pose.orientation.y = BT::convertFromString<double>(parts[6]);
    pose_stamped.pose.orientation.z = BT::convertFromString<double>(parts[7]);
    pose_stamped.pose.orientation.w = BT::convertFromString<double>(parts[8]);
    return pose_stamped;
  }else if(parts.size() == 3){
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose.position.x = BT::convertFromString<double>(parts[0]);
    pose_stamped.pose.position.y = BT::convertFromString<double>(parts[1]);
    auto yaw = BT::convertFromString<double>(parts[2]);
    tf2::Quaternion q;
    q.setRPY(0,0,yaw);
    tf2::convert(q, pose_stamped.pose.orientation);
    return pose_stamped;
  }
  else {
    throw std::runtime_error("invalid number of fields for PoseStamped attribute)");
  }
}

/**
 * @brief Parse XML string to std::chrono::milliseconds
 * @param key XML string
 * @return std::chrono::milliseconds
 */
template<>
inline std::chrono::milliseconds convertFromString<std::chrono::milliseconds>(const StringView key)
{
  return std::chrono::milliseconds(std::stoul(key.data()));
}

template<>
inline Pose2d convertFromString(const StringView key)
{
  // three real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 3) {
    throw std::runtime_error("invalid number of fields for point attribute)");
  } else {
    Pose2d pose;
    pose.x = BT::convertFromString<double>(parts[0]);
    pose.y = BT::convertFromString<double>(parts[1]);
    pose.yaw = BT::convertFromString<double>(parts[2]);
    return pose;
  }
}

}  // namespace BT

#endif  // NAV2_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_
