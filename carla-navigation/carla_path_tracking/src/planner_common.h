#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PLANNER_COMMON_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PLANNER_COMMON_H_

#include <vector>
#include <memory>

#include <carla_nav_types/common_types.h>
#include <carla_nav_types/conversions.h>
#include <carla_nav_math/math.h>

enum NodeState{
  IDLE,
  RUNNING,
  PAUSE,
  SUCCESS,
  FAILURE
};








#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PLANNER_COMMON_H_
